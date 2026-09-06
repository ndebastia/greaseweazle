/*
 * test_gw_realtime.c
 *
 * Minimal host-side smoke harness for GW RT command sequence.
 */

#define _DEFAULT_SOURCE 1

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#include "../../support/minimig/gw_usb.h"

typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint32_t chunk_idx;
    uint16_t len;
    uint8_t flags;
    uint8_t reserved;
} gw_rt_data_hdr;

typedef struct {
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

#define BUS_IBMPC  1u
#define BUS_SHUGART 2u
#define GWRP_DATA_FLAG_LAST 0x01u
#define RT_READ_FLAG_RAW_FLUX 0x01u
#define RT_READ_FLAG_INDEX_OPTIONAL 0x10u
#define RT_READ_FLAG_SYNTH_EVENTS_DIAG 0x20u
#define RT_READ_FLAG_DISABLE_HEAD_REALIGN 0x40u

int gw_usb_realtime_open(gw_usb_rt_ctx *ctx);
void gw_usb_realtime_close(gw_usb_rt_ctx *ctx);
int gw_usb_realtime_mode_select(gw_usb_rt_ctx *ctx, uint8_t mode);
int gw_usb_realtime_set_params_delays(gw_usb_rt_ctx *ctx);
int gw_usb_realtime_set_bus_type(gw_usb_rt_ctx *ctx, uint8_t type);
int gw_usb_realtime_select(gw_usb_rt_ctx *ctx, uint8_t unit);
int gw_usb_realtime_deselect(gw_usb_rt_ctx *ctx);
int gw_usb_realtime_motor(gw_usb_rt_ctx *ctx, uint8_t unit, uint8_t on_off);
int gw_usb_realtime_seek(gw_usb_rt_ctx *ctx, int16_t cyl);
int gw_usb_realtime_read_track(gw_usb_rt_ctx *ctx,
                               uint8_t cyl,
                               uint8_t head,
                               uint8_t flags,
                               uint8_t seq);
int gw_usb_realtime_get_status(gw_usb_rt_ctx *ctx, gw_rt_status *status);
int gw_usb_realtime_abort(gw_usb_rt_ctx *ctx);
int gw_usb_realtime_next_frame(gw_usb_rt_ctx *ctx,
                               gw_rt_data_hdr *data,
                               gw_rt_ack_frame *ack,
                               uint8_t **payload_out);
int gw_usb_realtime_consume_data(gw_usb_rt_ctx *ctx, const gw_rt_data_hdr *hdr);
int gw_usb_realtime_consume_ack(gw_usb_rt_ctx *ctx);
static const char *rt_err_name(int rc);
static int env_truthy(const char *name, int fallback);

static uint8_t env_u8_default(const char *name, uint8_t fallback)
{
    const char *raw = getenv(name);
    char *end = NULL;
    unsigned long value;

    if (!raw || !*raw)
        return fallback;

    errno = 0;
    value = strtoul(raw, &end, 0);
    if (errno != 0 || end == raw || (end && *end != '\0') || value > 0xffu)
        return fallback;

    return (uint8_t)value;
}

static uint8_t read_flags_default(void)
{
    uint8_t flags = env_u8_default("GW_RT_READ_FLAGS",
                                   env_u8_default("MFC_GW_RT_READ_FLAGS", 0u));

    if (env_truthy("GW_RT_RAW_FLUX", 0))
        flags = (uint8_t)(flags | RT_READ_FLAG_RAW_FLUX);
    if (env_truthy("GW_RT_FORCE_SYNTH_EVENTS",
                   env_truthy("MFC_GW_RT_FORCE_SYNTH_EVENTS", 0)))
        flags = (uint8_t)(flags | RT_READ_FLAG_SYNTH_EVENTS_DIAG);
    if (!env_truthy("GW_RT_INDEX_STRICT",
                    env_truthy("MFC_GW_RT_INDEX_STRICT", 0)))
        flags = (uint8_t)(flags | RT_READ_FLAG_INDEX_OPTIONAL);
    if (env_truthy("GW_RT_DISABLE_HEAD_REALIGN",
                   env_truthy("MFC_GW_RT_DISABLE_HEAD_REALIGN", 0)))
        flags = (uint8_t)(flags | RT_READ_FLAG_DISABLE_HEAD_REALIGN);

    return flags;
}

static int reopen_rt_transport(gw_usb_rt_ctx *ctx)
{
    gw_usb_realtime_close(ctx);
    return gw_usb_realtime_open(ctx);
}

static int prepare_mechanics(gw_usb_rt_ctx *ctx,
                             uint8_t bus_type,
                             uint8_t unit,
                             int16_t cyl,
                             int do_seek,
                             int mode_select_timeout_fallback,
                             const char **stage_out)
{
    uint8_t attempts = env_u8_default("GW_RT_MECH_RETRIES", 3u);
    uint8_t attempt;
    int rc = GW_RT_ERR_IO;

    if (attempts == 0u)
        attempts = 1u;
    if (attempts > 5u)
        attempts = 5u;

    for (attempt = 1u; attempt <= attempts; ++attempt) {
        *stage_out = "set_params";
        rc = gw_usb_realtime_set_params_delays(ctx);
        if (rc == 0) {
            *stage_out = "mode_select";
            rc = gw_usb_realtime_mode_select(ctx, 1u);
            if (rc != 0 && !(rc == GW_RT_ERR_TIMEOUT && mode_select_timeout_fallback))
                goto retry_or_fail;

            *stage_out = "set_bus_type";
            rc = gw_usb_realtime_set_bus_type(ctx, bus_type);
            if (rc != 0)
                goto retry_or_fail;

            *stage_out = "select";
            rc = gw_usb_realtime_select(ctx, unit);
            if (rc != 0)
                goto retry_or_fail;

            *stage_out = "motor_on";
            rc = gw_usb_realtime_motor(ctx, unit, 1u);
            if (rc != 0)
                goto retry_or_fail;

            if (do_seek) {
                *stage_out = "seek";
                rc = gw_usb_realtime_seek(ctx, cyl);
                if (rc != 0)
                    goto retry_or_fail;
            }

            return 0;
        }

retry_or_fail:
        if (attempt == attempts)
            break;

        fprintf(stdout,
                "RETRY mech_attempt=%u/%u failed_stage=%s rc=%d rc_name=%s errno=%d\n",
                attempt,
                attempts,
                *stage_out,
                rc,
                rt_err_name(rc),
                errno);

        if (reopen_rt_transport(ctx) != 0)
            return GW_RT_ERR_IO;
    }

    return rc;
}

static uint64_t mono_ms(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
        return 0u;
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

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

static const char *rt_err_name(int rc)
{
    switch (rc) {
    case GW_RT_OK:
        return "ok";
    case GW_RT_ERR_PARAM:
        return "param";
    case GW_RT_ERR_IO:
        return "io";
    case GW_RT_ERR_TIMEOUT:
        return "timeout";
    case GW_RT_ERR_PROTOCOL:
        return "protocol";
    case GW_RT_ERR_ACK:
        return "ack";
    default:
        return "unknown";
    }
}

int main(int argc, char **argv)
{
    gw_usb_rt_ctx ctx;
    gw_rt_status status;
    gw_rt_data_hdr data;
    gw_rt_ack_frame ack;
    uint8_t *payload = NULL;
    uint8_t cyl = 0u;
    uint8_t head = 0u;
    uint8_t flags;
    uint8_t seq = 2u;
    uint8_t unit = env_u8_default("GW_RT_UNIT", 0u);
    /* Default aligned with the production path (MFC_GW_BUS_TYPE=ibmpc,
     * hps_client/src/mfc_backend_greaseweazle.c): the bench drive is
     * IBM-PC wired. Override with GW_RT_BUS_TYPE=2 for Shugart. */
    uint8_t bus_type = env_u8_default("GW_RT_BUS_TYPE", BUS_IBMPC);
    int do_seek;
    uint64_t start_ms = 0u;
    uint64_t first_data_ms = 0u;
    int query_status = env_truthy("GW_RT_QUERY_STATUS", 1);
    int prepare_mech = env_truthy("GW_RT_PREPARE_MECH", 1);
    int mode_select_timeout_fallback = env_truthy("GW_RT_MODE_SELECT_TIMEOUT_FALLBACK", 0);
    int verbose = env_truthy("GW_RT_VERBOSE", 0);
    int rc = 0;
    int final_status = -1;
    int saw_last = 0;
    int want_abort = 0;
    int selected = 0;
    int motor_on = 0;
    unsigned int chunks = 0u;
    unsigned int total_bytes = 0u;
    unsigned int overflow_count = 0u;
    unsigned int doorbell_ms = 0u;
    const char *status_source = "fallback";
    const char *stage = "open";

    memset(&ctx, 0, sizeof(ctx));
    memset(&status, 0, sizeof(status));
    flags = read_flags_default();

    if (argc > 1)
        cyl = (uint8_t)strtoul(argv[1], NULL, 0);
    if (argc > 2)
        head = (uint8_t)strtoul(argv[2], NULL, 0);
    if (argc > 4)
        seq = (uint8_t)strtoul(argv[4], NULL, 0);
    do_seek = env_truthy("GW_RT_DO_SEEK", (cyl != 0u) ? 1 : 0);

    rc = gw_usb_realtime_open(&ctx);
    if (rc != 0)
        goto done;

    if (prepare_mech) {
        rc = prepare_mechanics(&ctx,
                               bus_type,
                               unit,
                               (int16_t)cyl,
                               do_seek,
                               mode_select_timeout_fallback,
                               &stage);
        if (rc != 0)
            goto done;
        selected = 1;
        motor_on = 1;
    } else {
        stage = "mode_select";
        rc = gw_usb_realtime_mode_select(&ctx, 1u);
        if (rc != 0) {
            if (rc == GW_RT_ERR_TIMEOUT && mode_select_timeout_fallback) {
                rc = 0;
            } else {
                goto done;
            }
        }
    }

    stage = "read_track";
    start_ms = mono_ms();
    rc = gw_usb_realtime_read_track(&ctx, cyl, head, flags, seq);
    if (rc != 0)
        goto done;
    want_abort = 1;

    stage = "stream";
    for (;;) {
        int frame_rc = gw_usb_realtime_next_frame(&ctx, &data, &ack, &payload);

        if (frame_rc < 0) {
            rc = frame_rc;
            break;
        }

        if (frame_rc == GW_USB_REALTIME_FRAME_DATA) {
            (void)payload;
            if (chunks == 0u) {
                first_data_ms = mono_ms();
                doorbell_ms = (unsigned int)(first_data_ms - start_ms);
            }
            chunks++;
            total_bytes += data.len;
            if (data.flags & GWRP_DATA_FLAG_LAST)
                saw_last = 1;
            rc = gw_usb_realtime_consume_data(&ctx, &data);
            if (rc != 0)
                break;
            continue;
        }

        if (frame_rc == GW_USB_REALTIME_FRAME_ACK) {
            final_status = ack.status;
            rc = gw_usb_realtime_consume_ack(&ctx);
            if (rc != 0)
                break;
            break;
        }
    }

done:
    if (query_status) {
        gw_rt_status current_status;
        int status_rc;

        memset(&current_status, 0, sizeof(current_status));
        status_rc = gw_usb_realtime_get_status(&ctx, &current_status);
        if (status_rc == 0) {
            status = current_status;
            overflow_count = status.overflow_count;
            status_source = "firmware_or_fallback";
        } else {
            status_source = "firmware_or_fallback";
        }
    }

    if (verbose || rc != 0 || final_status > 0) {
        fprintf(stdout,
            "ERR stage=%s rc=%d rc_name=%s errno=%d chunks=%u total_bytes=%u final_status=%d status_payload_len=%u status_flags=0x%04x term=%u reject=%u ibs=%d\n",
                stage,
                rc,
                rt_err_name(rc),
                errno,
                chunks,
                total_bytes,
            final_status,
            status.payload_len,
            status.flags,
            status.decoder_terminal_cause,
            status.decoder_reject_reason,
            status.decoder_info_best_shift);
    }

    fprintf(stdout,
            "RESULT rc=%d seq=%u cyl=%u head=%u unit=%u bus=%u doorbell_ms=%u chunks=%u total_bytes=%u saw_last=%d overflow_count=%u status_source=%s fw_payload_len=%u fw_mode=%u fw_active=%u fw_last_seq=%u fw_last_ack=%u fw_flags=0x%04x fw_term=%u fw_reject=%u fw_ibs=%d fw_index_diag=%u fw_index_irq_seen=%u fw_index_count=%u fw_cmd_count=%u fw_abort_count=%u fw_nack_count=%u fw_prep_us=%u fw_first_data_us=%u fw_final_ack_us=%u fw_ack_path_delay_counter=%u fw_pending_cmd_ack_len=%u fw_pending_cmd_ack_reason=%u fw_usb_xfer_errors=%u fw_dma_event_count=%u fw_dma_prod=%u fw_dma_cons=%u fw_dma_ndtr=%u fw_dma_irq=0x%x fw_dma_tim_cr1=%u fw_sync_count=%u fw_reject_count=%u fw_crc_count=%u fw_raw_len=%u fw_reset_light_count=%u\n",
            rc,
            seq,
            cyl,
            head,
            unit,
            bus_type,
            doorbell_ms,
            chunks,
            total_bytes,
            saw_last,
            overflow_count,
            status_source,
            status.payload_len,
            status.mode,
            status.active,
            status.last_seq,
            status.last_ack,
            status.flags,
            status.decoder_terminal_cause,
            status.decoder_reject_reason,
            status.decoder_info_best_shift,
            status.index_diag_valid,
            status.index_irq_seen_count,
            status.index_count,
            status.cmd_count,
            status.abort_count,
            status.nack_count,
            status.prep_us_last,
            status.first_data_us_last,
            status.final_ack_us_last,
            status.ack_path_delay_counter,
            status.pending_cmd_ack_len,
            status.pending_cmd_ack_reason,
            status.usb_xfer_errors,
            status.dma_event_count,
            status.dma_prod,
            status.dma_cons,
            status.dma_ndtr,
            status.dma_irq_flags,
            status.dma_tim_cr1,
            status.decoder_sync_count,
            status.decoder_reject_count,
            status.decoder_crc_error_count,
            status.decoder_raw_len,
            status.reset_light_count);

    if (want_abort)
        (void)gw_usb_realtime_abort(&ctx);
    if (motor_on)
        (void)gw_usb_realtime_motor(&ctx, unit, 0u);
    if (selected)
        (void)gw_usb_realtime_deselect(&ctx);
    gw_usb_realtime_close(&ctx);

    return (rc == 0 && saw_last && final_status == 0) ? 0 : 1;
}
