#!/bin/bash
# ============================================================
#  Script : run_metrics_sweep.sh
#  Project: Banked Memory Controller — TFG ITCR
#
#  Corre mem_full_test con distintos niveles de backpressure
#  y acumula los resultados en CSVs para análisis posterior.
#
#  Niveles: b_bp = r_bp = {0, 2, 5, 10, 15, 20, 30}
#
#  Uso:
#    ./run_metrics_sweep.sh
#
#  Salida:
#    metrics_output/metrics_summary.csv
#    metrics_output/latencies_detail.csv
#    metrics_output/bank_utilization.csv
#    metrics_output/run_bp<N>.log   (un log por corrida)
# ============================================================

set -e

# ── Directorios ────────────────────────────────────────────
export RUN_DIR=${RUN_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/Simulaciones}
export TB_DIR=${TB_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente}


CSV_DIR="/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/Metricas"

# ── Limpiar CSVs previos (el header cambió con hazard-aware) ──
cd ${CSV_DIR}
echo "===== Cleaning previous CSVs ====="
rm -f ${CSV_DIR}/metrics_summary.csv
rm -f ${CSV_DIR}/latencies_detail.csv
rm -f ${CSV_DIR}/bank_utilization.csv

## ── Compilación (1 sola vez) ───────────────────────────────
#echo "===== Compiling ====="
#vcs -f mem_ctrl_uvm.f \
#    -full64 \
#    -ntb_opts uvm-1.2 \
#    -l compile.log \
#    -o simv -debug_access+all -debug_region+cell
#
#if [[ ! -x ./simv ]]; then
#    echo "ERROR: compilación falló, simv no existe. Ver compile.log"
#    exit 1
#fi

cd ${RUN_DIR}
# ── Sweep de backpressure (mismo nivel b_bp = r_bp) ────────
BACKPRESSURES=(10 30)

PASS=0
FAIL=0
for bp in "${BACKPRESSURES[@]}"; do
    echo ""
    echo "===== Running bp=${bp} (b_bp=r_bp=${bp}) ====="
    ./mem_handler_simv_n8_lat1 +UVM_TESTNAME=mem_full_test \
           +UVM_VERBOSITY=UVM_LOW \
           +CSV_DIR=${CSV_DIR} \
           +B_BP=${bp} \
           +R_BP=${bp} \
           -l ${CSV_DIR}/run_bp${bp}.log

    # Parsear el UVM Report Summary para detectar fails reales
    FATAL_COUNT=$(grep -E "^UVM_FATAL\s*:" ${CSV_DIR}/run_bp${bp}.log | awk '{print $NF}')
    ERROR_COUNT=$(grep -E "^UVM_ERROR\s*:" ${CSV_DIR}/run_bp${bp}.log | awk '{print $NF}')

    if [[ -z "${FATAL_COUNT}" ]] || [[ -z "${ERROR_COUNT}" ]]; then
        echo "FAIL: bp=${bp} (no UVM summary — posible crash)"
        FAIL=$((FAIL+1))
    elif [[ "${FATAL_COUNT}" != "0" ]]; then
        echo "FAIL: bp=${bp} (UVM_FATAL=${FATAL_COUNT})"
        FAIL=$((FAIL+1))
    elif [[ "${ERROR_COUNT}" != "0" ]]; then
        echo "FAIL: bp=${bp} (UVM_ERROR=${ERROR_COUNT})"
        FAIL=$((FAIL+1))
    else
        echo "PASS: bp=${bp}"
        PASS=$((PASS+1))
    fi

    ./mem_handler_simv_n8_lat4 +UVM_TESTNAME=mem_full_test \
           +UVM_VERBOSITY=UVM_LOW \
           +CSV_DIR=${CSV_DIR} \
           +B_BP=${bp} \
           +R_BP=${bp} \
           -l ${CSV_DIR}/run_bp${bp}.log

    FATAL_COUNT=$(grep -E "^UVM_FATAL\s*:" ${CSV_DIR}/run_bp${bp}.log | awk '{print $NF}')
    ERROR_COUNT=$(grep -E "^UVM_ERROR\s*:" ${CSV_DIR}/run_bp${bp}.log | awk '{print $NF}')

    if [[ -z "${FATAL_COUNT}" ]] || [[ -z "${ERROR_COUNT}" ]]; then
        echo "FAIL: bp=${bp} (no UVM summary — posible crash)"
        FAIL=$((FAIL+1))
    elif [[ "${FATAL_COUNT}" != "0" ]]; then
        echo "FAIL: bp=${bp} (UVM_FATAL=${FATAL_COUNT})"
        FAIL=$((FAIL+1))
    elif [[ "${ERROR_COUNT}" != "0" ]]; then
        echo "FAIL: bp=${bp} (UVM_ERROR=${ERROR_COUNT})"
        FAIL=$((FAIL+1))
    else
        echo "PASS: bp=${bp}"
        PASS=$((PASS+1))
    fi
done

# ── Resumen ────────────────────────────────────────────────
echo ""
echo "============================="
echo "Sweep completo"
echo "Total runs: $((PASS+FAIL))   Pass: ${PASS}   Fail: ${FAIL}"
echo "============================="
echo ""
echo "CSVs generados en ${RUN_DIR}/${CSV_DIR}/:"
ls -la ${CSV_DIR}/*.csv

echo ""
echo "Líneas por CSV:"
for f in ${CSV_DIR}/metrics_summary.csv ${CSV_DIR}/latencies_detail.csv ${CSV_DIR}/bank_utilization.csv; do
    if [[ -f "$f" ]]; then
        echo "  $(wc -l < $f) líneas: $f"
    fi
done

exit ${FAIL}
