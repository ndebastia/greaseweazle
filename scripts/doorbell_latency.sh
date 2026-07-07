#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
HOST_TOOL="${ROOT_DIR}/tools/test_gw_realtime.c"
OUT_JSON="${OUT_JSON:-/tmp/gw_doorbell_latency.jsonl}"
ROUNDS="${ROUNDS:-20}"

echo "[doorbell] rounds=${ROUNDS} out=${OUT_JSON}"

echo '{"event":"doorbell_start"}' > "${OUT_JSON}"
for i in $(seq 1 "${ROUNDS}"); do
  now_ms="$(date +%s%3N)"
  echo "{\"event\":\"doorbell_sample\",\"iter\":${i},\"host_submit_ms\":${now_ms}}" >> "${OUT_JSON}"
done
echo '{"event":"doorbell_end"}' >> "${OUT_JSON}"

echo "[doorbell] done"
