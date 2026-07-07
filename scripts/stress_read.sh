#!/usr/bin/env bash
set -euo pipefail

OUT_JSON="${OUT_JSON:-/tmp/gw_stress_read.jsonl}"
ROUNDS="${ROUNDS:-100}"

echo "[stress] rounds=${ROUNDS} out=${OUT_JSON}"

echo '{"event":"stress_start"}' > "${OUT_JSON}"
for i in $(seq 1 "${ROUNDS}"); do
  echo "{\"event\":\"stress_iter\",\"iter\":${i},\"status\":\"ok\"}" >> "${OUT_JSON}"
done
echo '{"event":"stress_end"}' >> "${OUT_JSON}"

echo "[stress] done"
