/*
 * usb_realtime.c
 *
 * libusb transport scaffold for GWRP.
 */

#include "../../support/minimig/gw_usb.h"
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#define GW_VENDOR_ID      0x1209
#define GW_PRODUCT_ID     0x4d69
#define GW_EP_OUT         0x02
#define GW_EP_IN          0x83
#define GW_USB_IFACE      1
#define GW_TIMEOUT_MS     2000
#define GW_RX_BUF_SIZE    (128u * 1024u)
#define GW_DATA_LEN_MAX   4096u

#define GWRP_MAGIC         0x4757455au
#define GWRP_DATA_FLAG_LAST 0x01u
#define GW_RT_ACK_SHORT_TAG 0xD1u

#define CMD_SET_PARAMS     4u
#define PARAMS_DELAYS      0u
#define CMD_RT_MODE_SELECT 23u
#define CMD_RT_READ_TRACK  24u
#define CMD_RT_GET_STATUS  25u
#define CMD_RT_ABORT       26u
#define CMD_RT_SET_BUS_TYPE 27u
#define CMD_RT_SELECT      28u
#define CMD_RT_DESELECT    29u
#define CMD_RT_MOTOR       30u
#define CMD_RT_SEEK        31u
#define ACK_OKAY           0u
#define RT_MODE_USB        1u
#define BUS_SHUGART        2u
#define GW_RT_ABORT_REASON_TRANSPORT_RESET 2u

#define PAULA_SELECT_DELAY_US 10u
#define PAULA_STEP_DELAY_US 3000u
#define PAULA_SEEK_SETTLE_MS 18u
#define PAULA_MOTOR_DELAY_MS 500u
#define PAULA_WATCHDOG_MS 2000u
#define PAULA_PRE_WRITE_US 100u
#define PAULA_POST_WRITE_US 1000u
#define PAULA_INDEX_MASK_US 200u
#define GW_RT_DELAY_PARAM_COUNT 8u
#define RT_DELAY_PAYLOAD_LEN (1u + (GW_RT_DELAY_PARAM_COUNT * 2u))

#define GW_RT_STATUS_FLAG_ACK_DRAIN_ACTIVE 0x0008u
#define GW_RT_STATUS_FLAG_TERMINAL_CAUSE_SHIFT 1u
#define GW_RT_STATUS_FLAG_TERMINAL_CAUSE_MASK (0x3u << GW_RT_STATUS_FLAG_TERMINAL_CAUSE_SHIFT)
#define GW_RT_STATUS_FLAG_INDEX_DIAG_VALID 0x0080u
#define GW_RT_STATUS_FLAG_REJECT_REASON_SHIFT 8u
#define GW_RT_STATUS_FLAG_REJECT_REASON_MASK (0x0fu << GW_RT_STATUS_FLAG_REJECT_REASON_SHIFT)
#define GW_RT_STATUS_FLAG_INFO_SHIFT_SHIFT 12u
#define GW_RT_STATUS_FLAG_INFO_SHIFT_MASK (0x0fu << GW_RT_STATUS_FLAG_INFO_SHIFT_SHIFT)

#define GW_RT_STATUS_DIAG_PACKED_MARKER 0x80000000u
#define GW_RT_STATUS_DIAG_DMA_RING_MASK 0x000000ffu
#define GW_RT_STATUS_DIAG_DMA_NDTR_MASK 0x0000ffffu
#define GW_RT_STATUS_DIAG_DMA_TIM_CR1_MASK 0x0000000fu
#define GW_RT_STATUS_DIAG_SYNTH_CNT_MASK 0x00000007u
#define GW_RT_STATUS_DIAG_DMA_IRQ_MASK 0x0000ffffu
#define GW_RT_STATUS_DIAG_RESET_LIGHT_MASK 0x0000007fu
#define GW_RT_STATUS_DIAG_RESET_LIGHT_SHIFT 24u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint32_t chunk_idx;
    uint16_t len;
    uint8_t flags;
    uint8_t reserved;
} gw_rt_data_hdr;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint8_t status;
    uint8_t reserved;
    uint16_t len;
} gw_rt_ack_frame;

typedef struct {
    uint8_t mode;
    uint8_t active;
    uint8_t last_seq;
    uint8_t last_ack;
    uint16_t tx_window;
    uint16_t flags;
    uint32_t cmd_count;
    uint32_t abort_count;
    uint32_t nack_count;
    uint32_t overflow_count;
    uint32_t prep_us_last;
    uint32_t first_data_us_last;
    uint32_t final_ack_us_last;
    uint32_t prep_fail_count;
    uint32_t ack_path_delay_counter;
    uint32_t ctrl_q_len;
    uint32_t ack_queued;
    uint32_t ctrl_q_len_raw;
    uint32_t ack_queued_raw;
    uint32_t decoder_events_seen;
    uint32_t decoder_sync_count;
    uint32_t decoder_crc_error_count;
    uint32_t decoder_reject_count;
    uint32_t decoder_reject_reason;
    int32_t decoder_info_best_shift;
    uint32_t decoder_terminal_cause;
    uint32_t index_diag_valid;
    uint32_t index_irq_seen_count;
    uint32_t index_count;
    uint32_t index_rdata_cnt;
    uint32_t index_last_irq_us;
    uint32_t decoder_raw_len;
    uint32_t usb_write_fail_count;
    uint32_t pending_cmd_ack_len;
    uint32_t pending_cmd_ack_reason;
    uint32_t usb_xfer_errors;
    uint32_t dma_event_count;
    uint32_t dma_prod;
    uint32_t dma_cons;
    uint32_t synth_injected_count;
    uint32_t dma_ndtr;
    uint32_t dma_irq_flags;
    uint32_t dma_tim_cr1;
    uint32_t reset_light_count;
    uint32_t payload_len;
} gw_rt_status;

typedef struct {
    gw_usb_ctx_t usb;
    uint8_t *rx_buf;
    size_t rx_len;
} gw_usb_rt_ctx;

enum {
    GW_RT_OK = 0,
    GW_RT_ERR_PARAM = -1,
    GW_RT_ERR_IO = -2,
    GW_RT_ERR_TIMEOUT = -3,
    GW_RT_ERR_PROTOCOL = -4,
    GW_RT_ERR_ACK = -5
};

enum {
    GW_USB_REALTIME_FRAME_DATA = 1,
    GW_USB_REALTIME_FRAME_ACK = 2
};

static int env_truthy(const char *name, int fallback)
{
    const char *raw = getenv(name);

    if (!raw || !*raw)
        return fallback;

    return strcmp(raw, "1") == 0
        || strcasecmp(raw, "true") == 0
        || strcasecmp(raw, "yes") == 0
        || strcasecmp(raw, "on") == 0;
}

static uint16_t env_u16_default(const char *name, uint16_t fallback)
{
    const char *raw = getenv(name);
    char *end = NULL;
    unsigned long value;

    if (!raw || !*raw)
        return fallback;

    errno = 0;
    value = strtoul(raw, &end, 0);
    if (errno != 0 || end == raw || (end && *end != '\0') || value > 0xffffu)
        return fallback;

    return (uint16_t)value;
}

static uint32_t env_u32_default(const char *name, uint32_t fallback)
{
    const char *raw = getenv(name);
    char *end = NULL;
    unsigned long value;

    if (!raw || !*raw)
        return fallback;

    errno = 0;
    value = strtoul(raw, &end, 0);
    if (errno != 0 || end == raw || (end && *end != '\0') || value > 0xffffffffu)
        return fallback;

    return (uint32_t)value;
}

static uint32_t rt_control_timeout_ms(void)
{
    uint32_t timeout_ms = env_u32_default("MFC_GW_RT_MECH_TIMEOUT_MS", 4000u);

    if (timeout_ms < 500u)
        timeout_ms = 500u;
    if (timeout_ms > 10000u)
        timeout_ms = 10000u;

    return timeout_ms;
}

static uint16_t le16_load(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t le32_load(const uint8_t *p)
{
    return (uint32_t)p[0]
        | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16)
        | ((uint32_t)p[3] << 24);
}

static uint32_t decode_terminal_cause(uint16_t flags)
{
    return (uint32_t)((flags & GW_RT_STATUS_FLAG_TERMINAL_CAUSE_MASK)
                      >> GW_RT_STATUS_FLAG_TERMINAL_CAUSE_SHIFT);
}

static uint32_t decode_reject_reason(uint16_t flags)
{
    return (uint32_t)((flags & GW_RT_STATUS_FLAG_REJECT_REASON_MASK)
                      >> GW_RT_STATUS_FLAG_REJECT_REASON_SHIFT);
}

static int32_t decode_info_best_shift(uint16_t flags)
{
    return (int32_t)(((flags & GW_RT_STATUS_FLAG_INFO_SHIFT_MASK)
                      >> GW_RT_STATUS_FLAG_INFO_SHIFT_SHIFT) - 8);
}

static uint32_t decode_index_diag_valid(uint16_t flags)
{
    return (flags & GW_RT_STATUS_FLAG_INDEX_DIAG_VALID) ? 1u : 0u;
}

static void build_delay_params_payload(uint8_t payload[RT_DELAY_PAYLOAD_LEN])
{
    const uint16_t params[GW_RT_DELAY_PARAM_COUNT] = {
        env_u16_default("MFC_GW_SELECT_DELAY_US", PAULA_SELECT_DELAY_US),
        env_u16_default("MFC_GW_STEP_DELAY_US", PAULA_STEP_DELAY_US),
        env_u16_default("MFC_GW_SEEK_SETTLE_MS", PAULA_SEEK_SETTLE_MS),
        env_u16_default("MFC_GW_MOTOR_DELAY_MS", PAULA_MOTOR_DELAY_MS),
        env_u16_default("MFC_GW_WATCHDOG_MS", PAULA_WATCHDOG_MS),
        env_u16_default("MFC_GW_PRE_WRITE_US", PAULA_PRE_WRITE_US),
        env_u16_default("MFC_GW_POST_WRITE_US", PAULA_POST_WRITE_US),
        env_u16_default("MFC_GW_INDEX_MASK_US", PAULA_INDEX_MASK_US)
    };
    size_t i;

    payload[0] = PARAMS_DELAYS;
    for (i = 0u; i < GW_RT_DELAY_PARAM_COUNT; ++i) {
        payload[1u + (i * 2u)] = (uint8_t)(params[i] & 0xffu);
        payload[2u + (i * 2u)] = (uint8_t)((params[i] >> 8) & 0xffu);
    }
}

static void rx_drain(gw_usb_rt_ctx *ctx, uint32_t timeout_ms, unsigned int rounds);

static void best_effort_abort_and_flush(gw_usb_rt_ctx *ctx)
{
    uint8_t packet[3] = { CMD_RT_ABORT, 3u, GW_RT_ABORT_REASON_TRANSPORT_RESET };

    if (!ctx || !ctx->usb.available)
        return;

    (void)gw_usb_bulk_write(&ctx->usb, packet, sizeof(packet), 100u);
    rx_drain(ctx, 30u, 16u);
    ctx->rx_len = 0u;
}

static int cdc_acm_prime(gw_usb_rt_ctx *ctx, uint8_t ctrl_iface, uint32_t timeout_ms)
{
    (void)ctx;
    (void)ctrl_iface;
    (void)timeout_ms;
    return GW_RT_OK;
}

static void rx_drain(gw_usb_rt_ctx *ctx, uint32_t timeout_ms, unsigned int rounds)
{
    uint8_t scratch[256];
    unsigned int i;

    if (!ctx || !ctx->usb.available)
        return;

    for (i = 0u; i < rounds; ++i) {
        size_t transferred = 0u;

        if (gw_usb_bulk_read_some(&ctx->usb,
                                  scratch,
                                  sizeof(scratch),
                                  timeout_ms,
                                  &transferred) != 0)
            break;
        if (transferred == 0u)
            break;
    }

    ctx->rx_len = 0u;
}

static int rx_fill_once(gw_usb_rt_ctx *ctx, uint32_t timeout_ms)
{
    size_t transferred = 0u;

    if (!ctx || !ctx->usb.available)
        return GW_RT_ERR_IO;
    if (ctx->rx_len >= GW_RX_BUF_SIZE)
        return GW_RT_ERR_PROTOCOL;

    if (gw_usb_bulk_read_some(&ctx->usb,
                              ctx->rx_buf + ctx->rx_len,
                              GW_RX_BUF_SIZE - ctx->rx_len,
                              timeout_ms,
                              &transferred) != 0) {
        if (errno == ETIMEDOUT || errno == EAGAIN)
            return GW_RT_ERR_TIMEOUT;
        return GW_RT_ERR_IO;
    }
    if (transferred == 0u)
        return GW_RT_ERR_IO;

    ctx->rx_len += transferred;
    return GW_RT_OK;
}

static int rx_ensure(gw_usb_rt_ctx *ctx, size_t min_len, uint32_t timeout_ms)
{
    while (ctx->rx_len < min_len) {
        int rc = rx_fill_once(ctx, timeout_ms);

        if (rc != GW_RT_OK)
            return rc;
    }

    return GW_RT_OK;
}

static int rx_discard(gw_usb_rt_ctx *ctx, size_t len)
{
    if (len > ctx->rx_len)
        return GW_RT_ERR_PARAM;

    memmove(ctx->rx_buf, ctx->rx_buf + len, ctx->rx_len - len);
    ctx->rx_len -= len;
    return GW_RT_OK;
}

static int send_cmd(gw_usb_rt_ctx *ctx,
                    uint8_t cmd,
                    const uint8_t *payload,
                    size_t payload_len,
                    uint8_t *ack_code,
                    uint32_t timeout_ms)
{
    uint8_t packet[64];
    int rc;

    if (!ctx || !ctx->usb.available)
        return GW_RT_ERR_IO;
    if (payload_len > (sizeof(packet) - 2u))
        return GW_RT_ERR_PARAM;

    if (ctx->rx_len != 0u)
        ctx->rx_len = 0u;

    packet[0] = cmd;
    packet[1] = (uint8_t)(payload_len + 2u);
    if (payload_len != 0u)
        memcpy(packet + 2, payload, payload_len);

    if (gw_usb_bulk_write(&ctx->usb,
                          packet,
                          payload_len + 2u,
                          timeout_ms) != 0)
        return GW_RT_ERR_IO;

    rc = rx_ensure(ctx, 2u, timeout_ms);
    if (rc != GW_RT_OK)
        return rc;

    if (ctx->rx_buf[0] != cmd)
        return GW_RT_ERR_PROTOCOL;

    if (ack_code)
        *ack_code = ctx->rx_buf[1];

    rc = rx_discard(ctx, 2u);
    if (rc != GW_RT_OK)
        return rc;

    if (ack_code && *ack_code != ACK_OKAY)
        return GW_RT_ERR_ACK;

    if (cmd != CMD_RT_GET_STATUS)
        ctx->rx_len = 0u;

    return GW_RT_OK;
}

int gw_usb_realtime_send_control(gw_usb_rt_ctx *ctx,
                                 uint8_t cmd,
                                 const uint8_t *payload,
                                 size_t payload_len,
                                 uint32_t timeout_ms)
{
    uint8_t ack = 0u;

    return send_cmd(ctx, cmd, payload, payload_len, &ack, timeout_ms);
}

int gw_usb_realtime_set_params_delays(gw_usb_rt_ctx *ctx)
{
    uint8_t payload[RT_DELAY_PAYLOAD_LEN];

    build_delay_params_payload(payload);
    return gw_usb_realtime_send_control(ctx,
                                        CMD_SET_PARAMS,
                                        payload,
                                        sizeof(payload),
                                        rt_control_timeout_ms());
}

int gw_usb_realtime_set_bus_type(gw_usb_rt_ctx *ctx, uint8_t type)
{
    return gw_usb_realtime_send_control(ctx,
                                        CMD_RT_SET_BUS_TYPE,
                                        &type,
                                        1u,
                                        rt_control_timeout_ms());
}

int gw_usb_realtime_select(gw_usb_rt_ctx *ctx, uint8_t unit)
{
    return gw_usb_realtime_send_control(ctx,
                                        CMD_RT_SELECT,
                                        &unit,
                                        1u,
                                        rt_control_timeout_ms());
}

int gw_usb_realtime_deselect(gw_usb_rt_ctx *ctx)
{
    return gw_usb_realtime_send_control(ctx,
                                        CMD_RT_DESELECT,
                                        NULL,
                                        0u,
                                        rt_control_timeout_ms());
}

int gw_usb_realtime_motor(gw_usb_rt_ctx *ctx, uint8_t unit, uint8_t on_off)
{
    uint8_t payload[2] = { unit, (uint8_t)(on_off ? 1u : 0u) };

    return gw_usb_realtime_send_control(ctx,
                                        CMD_RT_MOTOR,
                                        payload,
                                        sizeof(payload),
                                        rt_control_timeout_ms());
}

int gw_usb_realtime_seek(gw_usb_rt_ctx *ctx, int16_t cyl)
{
    uint8_t payload[2];

    payload[0] = (uint8_t)(cyl & 0xff);
    payload[1] = (uint8_t)((cyl >> 8) & 0xff);
    return gw_usb_realtime_send_control(ctx,
                                        CMD_RT_SEEK,
                                        payload,
                                        sizeof(payload),
                                        rt_control_timeout_ms());
}

int gw_usb_realtime_open(gw_usb_rt_ctx *ctx)
{
    gw_usb_open_params_t params;

    if (!ctx)
        return GW_RT_ERR_PARAM;

    memset(ctx, 0, sizeof(*ctx));
    ctx->rx_buf = (uint8_t *)malloc(GW_RX_BUF_SIZE);
    if (!ctx->rx_buf)
        return GW_RT_ERR_IO;

    memset(&params, 0, sizeof(params));
    params.vid = GW_VENDOR_ID;
    params.pid = GW_PRODUCT_ID;
    params.in_ep = GW_EP_IN;
    params.out_ep = GW_EP_OUT;
    params.interface_number = GW_USB_IFACE;
    params.alt_setting = 0u;
    params.timeout_ms = (uint16_t)GW_TIMEOUT_MS;

    if (gw_usb_open(&ctx->usb, &params) != 0)
        return GW_RT_ERR_IO;

    /* Mirror the proven HPS transport: after interface claim/USB reconfig,
     * let the GW firmware settle before issuing the first RT control packet. */
    sleep(1);

    /* Clear any stale RT stream/final-ack state left by an interrupted prior
     * session before sending MODE_SELECT or mechanics control. */
    best_effort_abort_and_flush(ctx);

    if (GW_USB_IFACE != 0 && env_truthy("GW_RT_CDC_PRIME", 1)) {
        int prime_rc = cdc_acm_prime(ctx, 0u, GW_TIMEOUT_MS);

        if (prime_rc != GW_RT_OK && env_truthy("GW_RT_CDC_PRIME_STRICT", 0))
            return prime_rc;
    }

    rx_drain(ctx, 20u, 8u);

    return GW_RT_OK;
}

void gw_usb_realtime_close(gw_usb_rt_ctx *ctx)
{
    if (!ctx)
        return;

    rx_drain(ctx, 20u, 4u);

    if (ctx->usb.available || ctx->usb.claimed)
        (void)gw_usb_close(&ctx->usb);

    free(ctx->rx_buf);
    memset(ctx, 0, sizeof(*ctx));
}

int gw_usb_realtime_mode_select(gw_usb_rt_ctx *ctx, uint8_t mode)
{
    uint8_t payload[2] = { mode, 0u };
    uint8_t ack = 0u;
    return send_cmd(ctx,
                    CMD_RT_MODE_SELECT,
                    payload,
                    sizeof(payload),
                    &ack,
                    rt_control_timeout_ms());
}

int gw_usb_realtime_read_track(gw_usb_rt_ctx *ctx,
                               uint8_t cyl,
                               uint8_t head,
                               uint8_t flags,
                               uint8_t seq)
{
    uint8_t packet[6] = {
        CMD_RT_READ_TRACK,
        6u,
        cyl,
        head,
        flags,
        seq
    };
    int rc;

    if (!ctx || !ctx->usb.available)
        return GW_RT_ERR_IO;

    ctx->rx_len = 0u;

    if (gw_usb_bulk_write(&ctx->usb,
                          packet,
                          sizeof(packet),
                          GW_TIMEOUT_MS) != 0)
        return GW_RT_ERR_IO;

    rc = rx_ensure(ctx, 3u, GW_TIMEOUT_MS);
    if (rc != GW_RT_OK)
        return rc;
    if (ctx->rx_buf[0] != GW_RT_ACK_SHORT_TAG || ctx->rx_buf[1] != seq)
        return GW_RT_ERR_PROTOCOL;
    if (ctx->rx_buf[2] != RT_MODE_USB)
        return GW_RT_ERR_ACK;

    return rx_discard(ctx, 3u);
}

int gw_usb_realtime_get_status(gw_usb_rt_ctx *ctx, gw_rt_status *status)
{
    uint8_t ack = 0u;
    uint8_t raw[88];
    size_t payload_len;
    uint32_t tail_timeout_ms = 20u;
    int rc;

    if (!ctx || !status)
        return GW_RT_ERR_PARAM;

    memset(status, 0, sizeof(*status));

    rc = send_cmd(ctx, CMD_RT_GET_STATUS, NULL, 0u, &ack, GW_TIMEOUT_MS);
    if (rc != GW_RT_OK)
        return rc;

    rc = rx_ensure(ctx, 20u, GW_TIMEOUT_MS);
    if (rc != GW_RT_OK)
        return rc;

    if (ctx->rx_len >= 44u && ctx->rx_len < sizeof(raw)) {
        for (;;) {
            if (ctx->rx_len >= sizeof(raw))
                break;
            rc = rx_fill_once(ctx, tail_timeout_ms);
            if (rc == GW_RT_ERR_TIMEOUT)
                break;
            if (rc != GW_RT_OK)
                return rc;
        }
    }

    if (ctx->rx_len >= sizeof(raw)) {
        payload_len = sizeof(raw);
    } else if (ctx->rx_len >= 64u) {
        payload_len = 64u;
    } else if (ctx->rx_len >= 60u) {
        payload_len = 60u;
    } else if (ctx->rx_len >= 52u) {
        payload_len = 52u;
    } else if (ctx->rx_len >= 44u) {
        payload_len = 44u;
    } else {
        payload_len = 20u;
    }

    memset(raw, 0, sizeof(raw));
    memcpy(raw, ctx->rx_buf, payload_len);
    status->payload_len = (uint32_t)payload_len;
    status->mode = raw[0];
    status->active = raw[1];
    status->last_seq = raw[2];
    status->last_ack = raw[3];

    if (payload_len >= 44u) {
        status->tx_window = le16_load(raw + 4u);
        status->flags = le16_load(raw + 6u);
        status->decoder_terminal_cause = decode_terminal_cause(status->flags);
        status->decoder_reject_reason = decode_reject_reason(status->flags);
        status->decoder_info_best_shift = decode_info_best_shift(status->flags);
        status->index_diag_valid = decode_index_diag_valid(status->flags);
        status->prep_us_last = le32_load(raw + 24u);
        status->first_data_us_last = le32_load(raw + 28u);
        status->final_ack_us_last = le32_load(raw + 32u);
        status->prep_fail_count = le32_load(raw + 36u);
        status->ack_path_delay_counter = le32_load(raw + 40u);
    }

    if (payload_len >= 52u) {
        uint32_t raw_ctrl_q = le32_load(raw + 44u);
        uint32_t raw_ack_q = le32_load(raw + 48u);

        status->ctrl_q_len_raw = raw_ctrl_q;
        status->ack_queued_raw = raw_ack_q;
        if (((raw_ctrl_q & GW_RT_STATUS_DIAG_PACKED_MARKER) != 0u)
            && ((raw_ack_q & GW_RT_STATUS_DIAG_PACKED_MARKER) != 0u)) {
            status->dma_prod = raw_ctrl_q & GW_RT_STATUS_DIAG_DMA_RING_MASK;
            status->dma_cons = raw_ack_q & GW_RT_STATUS_DIAG_DMA_RING_MASK;
            status->dma_ndtr = (raw_ctrl_q >> 8) & GW_RT_STATUS_DIAG_DMA_NDTR_MASK;
            status->dma_tim_cr1 = (raw_ctrl_q >> 24) & GW_RT_STATUS_DIAG_DMA_TIM_CR1_MASK;
            status->synth_injected_count = (raw_ctrl_q >> 28) & GW_RT_STATUS_DIAG_SYNTH_CNT_MASK;
            status->dma_irq_flags = (raw_ack_q >> 8) & GW_RT_STATUS_DIAG_DMA_IRQ_MASK;
            status->reset_light_count = (raw_ack_q >> GW_RT_STATUS_DIAG_RESET_LIGHT_SHIFT)
                                      & GW_RT_STATUS_DIAG_RESET_LIGHT_MASK;
        } else {
            status->ctrl_q_len = raw_ctrl_q;
            status->ack_queued = raw_ack_q;
        }
    }

    if (payload_len >= 88u) {
        status->cmd_count = le32_load(raw + 8u);
        status->abort_count = le32_load(raw + 12u);
        status->nack_count = le32_load(raw + 16u);
        status->overflow_count = le32_load(raw + 20u);
        status->decoder_events_seen = le32_load(raw + 52u);
        status->decoder_sync_count = le32_load(raw + 56u);
        status->usb_write_fail_count = le32_load(raw + 60u);
        status->pending_cmd_ack_len = le32_load(raw + 64u);
        status->pending_cmd_ack_reason = le32_load(raw + 68u);
        status->usb_xfer_errors = le32_load(raw + 72u);
        status->dma_event_count = le32_load(raw + 76u);
        status->dma_prod = le32_load(raw + 80u);
        status->dma_cons = le32_load(raw + 84u);
    } else if (payload_len >= 60u) {
        if (status->index_diag_valid) {
            status->index_irq_seen_count = le32_load(raw + 8u);
            status->index_count = le32_load(raw + 12u);
            status->index_rdata_cnt = le32_load(raw + 16u);
            status->index_last_irq_us = le32_load(raw + 20u);
        } else {
            status->overflow_count = 0u;
            status->decoder_sync_count = le16_load(raw + 52u);
            status->decoder_crc_error_count = le16_load(raw + 54u);
            status->decoder_reject_count = le16_load(raw + 56u);
            status->decoder_raw_len = le16_load(raw + 58u);
        }
    }

    return rx_discard(ctx, payload_len);
}

int gw_usb_realtime_abort(gw_usb_rt_ctx *ctx)
{
    uint8_t ack = 0u;

    return send_cmd(ctx, CMD_RT_ABORT, NULL, 0u, &ack, GW_TIMEOUT_MS);
}

int gw_usb_realtime_next_frame(gw_usb_rt_ctx *ctx,
                               gw_rt_data_hdr *data,
                               gw_rt_ack_frame *ack,
                               uint8_t **payload_out)
{
    uint32_t magic;
    int rc;

    if (!ctx)
        return GW_RT_ERR_PARAM;

    if (data)
        memset(data, 0, sizeof(*data));
    if (ack)
        memset(ack, 0, sizeof(*ack));
    if (payload_out)
        *payload_out = NULL;

    for (;;) {
        rc = rx_ensure(ctx, 4u, GW_TIMEOUT_MS);
        if (rc != GW_RT_OK)
            return rc;

        magic = le32_load(ctx->rx_buf);
        if (magic != GWRP_MAGIC) {
            rc = rx_discard(ctx, 1u);
            if (rc != GW_RT_OK)
                return rc;
            continue;
        }

        if (ctx->rx_len >= sizeof(gw_rt_data_hdr)) {
            uint16_t data_len = le16_load(ctx->rx_buf + 12u);
            size_t total = sizeof(gw_rt_data_hdr) + (size_t)data_len;

            if (data_len > 0u) {
                if (data_len > GW_DATA_LEN_MAX)
                    return GW_RT_ERR_PROTOCOL;
                rc = rx_ensure(ctx, total, GW_TIMEOUT_MS);
                if (rc != GW_RT_OK)
                    return rc;
                if (data) {
                    memcpy(data, ctx->rx_buf, sizeof(*data));
                }
                if (payload_out)
                    *payload_out = ctx->rx_buf + sizeof(gw_rt_data_hdr);
                return GW_USB_REALTIME_FRAME_DATA;
            }
        }

        if (ctx->rx_len >= sizeof(gw_rt_ack_frame)
            && ctx->rx_buf[9u] == 0u
            && le16_load(ctx->rx_buf + 10u) == 0u) {
            if (ack) {
                memcpy(ack, ctx->rx_buf, sizeof(*ack));
            }
            return GW_USB_REALTIME_FRAME_ACK;
        }

        rc = rx_discard(ctx, 1u);
        if (rc != GW_RT_OK)
            return rc;
    }
}

int gw_usb_realtime_consume_data(gw_usb_rt_ctx *ctx, const gw_rt_data_hdr *hdr)
{
    if (!ctx || !hdr)
        return GW_RT_ERR_PARAM;
    return rx_discard(ctx, sizeof(gw_rt_data_hdr) + (size_t)hdr->len);
}

int gw_usb_realtime_consume_ack(gw_usb_rt_ctx *ctx)
{
    if (!ctx)
        return GW_RT_ERR_PARAM;
    return rx_discard(ctx, sizeof(gw_rt_ack_frame));
}
