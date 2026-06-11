#!/bin/bash
# ============================================================
#  Script  : run_tests.sh
#  Project : Banked Memory Controller — TFG ITCR
#
#  Ejecuta la regresión oficial del testplan: mem_full_test
#  sobre la matriz N_BANKS × READ_LATENCY (8 configuraciones).
#  Compila cada configuración llamando a compile_tests.sh,
#  ejecuta mem_full_test, y al final mergea cobertura en
#  cov_merged.vdb con reporte HTML.
#
#  Uso:
#    ./run_tests.sh                 # regresión completa (8 configs)
#    ./run_tests.sh debug           # tests adicionales sobre baseline
#                                    (smoke, b_backpressure, r_backpressure)
#    ./run_tests.sh CFG-A1          # corre una sola config específica
#    ./run_tests.sh merge-only      # solo mergea las .vdb ya existentes
# ============================================================

set -e

# ── Directorios ────────────────────────────────────────────
export RTL_DIR=${RTL_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/RTL}
export TB_DIR=${TB_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente}
export RUN_DIR=${RUN_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/Simulaciones}
export SCRIPT_DIR=${SCRIPT_DIR:-$(dirname "$(readlink -f "$0")")}

# ── Directorio raíz de métricas (un subdirectorio por config) ──
METRICS_ROOT="${RUN_DIR}/metrics"

# ── Matriz oficial: 8 configuraciones ──────────────────────
# Formato: "NAME N_BANKS READ_LATENCY"
declare -a CONFIGS=(
    "CFG-A1 4  1"
    "CFG-A4 4  4"
    "CFG-B1 8  1"
    "CFG-B4 8  4"
    "CFG-C1 16 1"
    "CFG-C4 16 4"
)

# ── Test funcional oficial ─────────────────────────────────
OFFICIAL_TEST="mem_full_test"

# ── Tests de debug (no parte del testplan oficial) ─────────
DEBUG_TESTS=(
    "mem_smoke_test"
    "mem_b_backpressure_test"
    "mem_r_backpressure_test"
)

# ── Selección de modo ──────────────────────────────────────
MODE="regression"
SELECTED_CFG=""

if [[ "$1" == "debug" ]]; then
    MODE="debug"
elif [[ "$1" == "merge-only" ]]; then
    MODE="merge-only"
elif [[ -n "$1" ]]; then
    MODE="single"
    SELECTED_CFG="$1"
fi

# ── Función: ejecuta un test sobre una configuración ───────
run_test() {
    local cfg_name=$1
    local n_banks=$2
    local read_lat=$3
    local test_name=$4

    local simv_name="mem_handler_simv_n${n_banks}_lat${read_lat}"
    local cov_dir="cov_n${n_banks}_lat${read_lat}.vdb"
    local log_name="${test_name}_n${n_banks}_lat${read_lat}.log"
    local metrics_dir="${METRICS_ROOT}/n${n_banks}_lat${read_lat}"

    echo ""
    echo "===== Running ${test_name} on ${cfg_name} (N=${n_banks}, LAT=${read_lat}) ====="

    cd ${RUN_DIR}
    mkdir -p "${metrics_dir}"

    if [[ ! -x "./${simv_name}" ]]; then
        echo "ERROR: ${simv_name} no existe. ¿Compilaste esta configuración?"
        return 1
    fi

    ./${simv_name} \
        +UVM_TESTNAME=${test_name} \
        +UVM_VERBOSITY=UVM_LOW \
        -cm line+cond+fsm+tgl+branch \
        -cm_dir ${RUN_DIR}/${cov_dir} \
        -cm_name ${test_name}_${cfg_name} \
        +uvm_set_config_string=*,csv_dir,${metrics_dir} \
        +uvm_set_config_int=*,read_latency_val,${read_lat} \
        -l ${log_name}

    # Parsear UVM Report Summary
    local fatal_count=$(grep -E "^UVM_FATAL\s*:" ${log_name} | awk '{print $NF}')
    local error_count=$(grep -E "^UVM_ERROR\s*:" ${log_name} | awk '{print $NF}')

    if [[ -z "${fatal_count}" ]] || [[ -z "${error_count}" ]]; then
        echo "FAIL: ${cfg_name}/${test_name} (no UVM summary — posible crash)"
        return 1
    elif [[ "${fatal_count}" != "0" ]]; then
        echo "FAIL: ${cfg_name}/${test_name} (UVM_FATAL=${fatal_count})"
        return 1
    elif [[ "${error_count}" != "0" ]]; then
        echo "FAIL: ${cfg_name}/${test_name} (UVM_ERROR=${error_count})"
        return 1
    else
        echo "PASS: ${cfg_name}/${test_name}"
        return 0
    fi
}

# ── Función: merge de cobertura ────────────────────────────
merge_coverage() {
    echo ""
    echo "===== Merging coverage databases ====="
    cd ${RUN_DIR}

    # Listar todas las .vdb existentes
    local vdb_list=""
    for cfg_entry in "${CONFIGS[@]}"; do
        local n_banks=$(echo ${cfg_entry} | awk '{print $2}')
        local read_lat=$(echo ${cfg_entry} | awk '{print $3}')
        local vdb_path="${RUN_DIR}/cov_n${n_banks}_lat${read_lat}.vdb"
        if [[ -d "${vdb_path}" ]]; then
            vdb_list+="${vdb_path} "
        else
            echo "WARNING: ${vdb_path} no existe, omitido del merge"
        fi
    done

    if [[ -z "${vdb_list}" ]]; then
        echo "ERROR: ninguna .vdb encontrada para mergear"
        return 1
    fi

    # Merge con urg + reporte HTML/text
    urg -dir ${vdb_list} \
        -dbname ${RUN_DIR}/cov_merged \
        -report ${RUN_DIR}/cov_report \
        -format both \
        -log ${RUN_DIR}/urg_merge.log

    echo ""
    echo "Cobertura merged en: ${RUN_DIR}/cov_merged"
    echo "Reporte HTML       : ${RUN_DIR}/cov_report/dashboard.html"
    echo "Log del merge      : ${RUN_DIR}/urg_merge.log"
}

# ── Función: concatena los CSVs por config en metrics_by_config.csv ───────
# Lee todos los metrics/n*_lat*/metrics_summary.csv, extrae n_banks y
# read_latency del nombre del directorio, y produce un CSV unificado.
# No requiere cambios en el scoreboard.
build_config_csv() {
    echo ""
    echo "===== Building metrics_by_config.csv ====="
    python3 - << PYEOF
import pandas as pd, glob, re, os, sys

root    = "${METRICS_ROOT}"
pattern = os.path.join(root, "n*_lat*", "metrics_summary.csv")
files   = sorted(glob.glob(pattern))

if not files:
    print(f"[WARN] No se encontraron CSVs en: {pattern}")
    sys.exit(0)

dfs = []
for f in files:
    m = re.search(r'n(\d+)_lat(\d+)', f)
    if not m:
        print(f"[WARN] Patrón n*_lat* no encontrado en: {f}")
        continue
    df = pd.read_csv(f)
    # Insertar columnas de config si el scoreboard no las escribió
    if 'n_banks' not in df.columns:
        df.insert(1, 'n_banks',      int(m.group(1)))
    if 'read_latency' not in df.columns:
        df.insert(2, 'read_latency', int(m.group(2)))
    dfs.append(df)
    print(f"  [{m.group(1)}B LAT{m.group(2)}] {f}")

if not dfs:
    print("[ERROR] Ningún CSV válido encontrado.")
    sys.exit(1)

combined = pd.concat(dfs, ignore_index=True)
output   = os.path.join(root, "metrics_by_config.csv")
combined.to_csv(output, index=False)
print(f"\n✓ {output}")
print(f"  {len(combined)} fila(s), {len(combined.columns)} columnas")
PYEOF
}

# ── Función: regresión completa ────────────────────────────
run_regression() {
    local pass=0
    local fail=0
    local failed_configs=()

    echo "============================="
    echo "REGRESIÓN COMPLETA — 8 configs"
    echo "============================="

    for cfg_entry in "${CONFIGS[@]}"; do
        local cfg_name=$(echo ${cfg_entry} | awk '{print $1}')
        local n_banks=$(echo ${cfg_entry} | awk '{print $2}')
        local read_lat=$(echo ${cfg_entry} | awk '{print $3}')

        # Compilar
        echo ""
        echo "──────────────────────────────────────"
        echo "Compilando ${cfg_name} (N=${n_banks}, LAT=${read_lat})"
        echo "──────────────────────────────────────"
        bash ${SCRIPT_DIR}/compile_tests.sh ${n_banks} ${read_lat}

        # Ejecutar mem_full_test
        if run_test ${cfg_name} ${n_banks} ${read_lat} ${OFFICIAL_TEST}; then
            pass=$((pass+1))
        else
            fail=$((fail+1))
            failed_configs+=("${cfg_name}")
        fi
    done

    echo ""
    echo "============================="
    echo "Regression results"
    echo "============================="
    echo "Total: $((pass+fail))   Pass: ${pass}   Fail: ${fail}"
    if [[ ${fail} -gt 0 ]]; then
        echo "Failed configs: ${failed_configs[@]}"
    fi
    echo "============================="

    build_config_csv

    return ${fail}
}

# ── Función: tests de debug sobre baseline ─────────────────
run_debug() {
    echo ""
    echo "===== Debug mode: corriendo tests adicionales sobre CFG-A1 ====="

    # Compilar baseline
    bash ${SCRIPT_DIR}/compile_tests.sh 4 1

    local pass=0
    local fail=0
    for test_name in "${DEBUG_TESTS[@]}"; do
        if run_test "CFG-A1" 4 1 ${test_name}; then
            pass=$((pass+1))
        else
            fail=$((fail+1))
        fi
    done

    echo ""
    echo "Debug results: Pass=${pass}, Fail=${fail}"
    return ${fail}
}

# ── Función: una sola configuración ────────────────────────
run_single() {
    local target=$1

    for cfg_entry in "${CONFIGS[@]}"; do
        local cfg_name=$(echo ${cfg_entry} | awk '{print $1}')
        local n_banks=$(echo ${cfg_entry} | awk '{print $2}')
        local read_lat=$(echo ${cfg_entry} | awk '{print $3}')

        if [[ "${cfg_name}" == "${target}" ]]; then
            bash ${SCRIPT_DIR}/compile_tests.sh ${n_banks} ${read_lat}
            run_test ${cfg_name} ${n_banks} ${read_lat} ${OFFICIAL_TEST}
            return $?
        fi
    done

    echo "ERROR: Configuración '${target}' no encontrada"
    echo "Configuraciones disponibles:"
    for cfg_entry in "${CONFIGS[@]}"; do
        echo "  $(echo ${cfg_entry} | awk '{print $1}')"
    done
    return 1
}

# ── Main dispatch ──────────────────────────────────────────
case ${MODE} in
    regression)
        run_regression
        REGRESSION_RC=$?
        #merge_coverage
        exit ${REGRESSION_RC}
        ;;
    debug)
        run_debug
        ;;
    merge-only)
        merge_coverage
        ;;
    single)
        run_single "${SELECTED_CFG}"
        ;;
esac