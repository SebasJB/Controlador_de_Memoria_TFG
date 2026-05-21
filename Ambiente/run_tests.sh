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

mkdir -p ${RUN_DIR}

# ── Lista oficial de tests ─────────────────────────────────
ALL_TESTS=(
    mem_smoke_test
    mem_reset_in_flight_test
    mem_random_test
    mem_single_bank_test
    mem_multibank_parallel_test
    mem_conflict_same_bank_test
    mem_b_backpressure_test
    mem_r_backpressure_test
    mem_invalid_addr_test
    mem_rob_wrap_test
    mem_fifo_saturation_test
    mem_regression_test
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
vcs -f ../mem_ctrl_uvm.f \
    -full64 \
    -ntb_opts uvm-1.2 \
    ${COVERAGE_FLAG} \
    -l compile.log \
    -o simv

# ── Ejecución por test ─────────────────────────────────────
PASS=0
FAIL=0
for tst in "${TESTS[@]}"; do
    echo ""
    echo "===== Running ${tst} ====="
    ./simv +UVM_TESTNAME=${tst} \
           +UVM_VERBOSITY=UVM_LOW \
           ${COVERAGE_FLAG} \
           -l ${tst}.log

    if grep -q "UVM_FATAL\|UVM_ERROR" ${tst}.log; then
        if grep -q "UVM_FATAL" ${tst}.log; then
            echo "FAIL: ${tst} (UVM_FATAL)"
            FAIL=$((FAIL+1))
        else
            # uvm_errors >0 también es fail; salvo SLVERR que es info
            ERR_COUNT=$(grep -c "UVM_ERROR" ${tst}.log)
            if [[ ${ERR_COUNT} -gt 0 ]]; then
                echo "FAIL: ${tst} (${ERR_COUNT} UVM_ERROR)"
                FAIL=$((FAIL+1))
            fi
        fi
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
