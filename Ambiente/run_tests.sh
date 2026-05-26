#!/bin/bash
# ============================================================
#  Script : run_tests.sh
#  Project: Banked Memory Controller — TFG ITCR
#
#  Uso:
#    ./run_tests.sh                    # corre todos los tests
#    ./run_tests.sh mem_smoke_test     # corre solo uno
#    ./run_tests.sh -coverage          # corre todos + reporte
# ============================================================

set -e

# ── Directorios (ajustar según layout local) ───────────────
export RTL_DIR=${RTL_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/RTL}
export SVA_DIR=${SVA_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/RTL/SVA}
export TB_DIR=${TB_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente}
export RUN_DIR=${RUN_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente}

# ── Lista oficial de tests ─────────────────────────────────
ALL_TESTS=(
    #"mem_smoke_test"
    "mem_full_test +PHASE_ONLY=GENERAL"
    #"mem_b_backpressure_test"
    #"mem_r_backpressure_test"
)

# ── Selección ──────────────────────────────────────────────
COVERAGE_FLAG=""
if [[ "$1" == "-coverage" ]]; then
    COVERAGE_FLAG="-cm line+cond+fsm+tgl+branch -cm_dir ${RUN_DIR}/cov.vdb"
    TESTS=("${ALL_TESTS[@]}")
elif [[ -n "$1" ]]; then
    TESTS=("$1")
else
    TESTS=("${ALL_TESTS[@]}")
fi

# ── Compilación ────────────────────────────────────────────
echo "===== Compiling ====="
cd ${RUN_DIR}
vcs -f mem_ctrl_uvm.f \
    -full64 \
    -ntb_opts uvm-1.2 \
    ${COVERAGE_FLAG} \
    -l compile.log \
    -o simv -debug_access+all -debug_region+cell

# ── Ejecución por test ─────────────────────────────────────
PASS=0
FAIL=0
for tst in "${TESTS[@]}"; do
    echo ""
    echo "===== Running ${tst} ====="
    ./simv +UVM_TESTNAME=${tst} \
           +UVM_VERBOSITY=UVM_LOW \
           -n ${COVERAGE_FLAG} \
           -l ${tst}.log

    # Parsear UVM Report Summary para detectar fails reales
    FATAL_COUNT=$(grep -E "^UVM_FATAL\s*:" ${tst}.log | awk '{print $NF}')
    ERROR_COUNT=$(grep -E "^UVM_ERROR\s*:" ${tst}.log | awk '{print $NF}')

    if [[ -z "${FATAL_COUNT}" ]] || [[ -z "${ERROR_COUNT}" ]]; then
        # El test no generó UVM Report Summary → crash o terminó mal
        echo "FAIL: ${tst} (no UVM summary — posible crash)"
        FAIL=$((FAIL+1))
    elif [[ "${FATAL_COUNT}" != "0" ]]; then
        echo "FAIL: ${tst} (UVM_FATAL=${FATAL_COUNT})"
        FAIL=$((FAIL+1))
    elif [[ "${ERROR_COUNT}" != "0" ]]; then
        echo "FAIL: ${tst} (UVM_ERROR=${ERROR_COUNT})"
        FAIL=$((FAIL+1))
    else
        echo "PASS: ${tst}"
        PASS=$((PASS+1))
    fi
done

# ── Resumen ────────────────────────────────────────────────
echo ""
echo "============================="
echo "Total: $((PASS+FAIL))   Pass: ${PASS}   Fail: ${FAIL}"
echo "============================="

# ── Reporte de coverage ────────────────────────────────────
if [[ -n "${COVERAGE_FLAG}" ]]; then
    echo ""
    echo "===== Generating coverage report ====="
    urg -dir cov.vdb -report cov_report -format both
    echo "Coverage report: ${RUN_DIR}/cov_report/dashboard.html"
fi

exit ${FAIL}
