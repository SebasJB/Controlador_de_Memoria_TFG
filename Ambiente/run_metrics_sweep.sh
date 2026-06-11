#!/bin/bash
# ============================================================
#  Script : run_metrics_sweep.sh
#  Project: Banked Memory Controller — TFG ITCR
#
#  Corre mem_full_test con distintos niveles de backpressure
#  sobre CFG-B1 (N=8 LAT=1) y CFG-B4 (N=8 LAT=4).
#  Cada corrida escribe sus CSVs en su propio subdirectorio.
#  Al final concatena todos en metrics_sweep_all.csv.
#
#  Niveles BP: b_bp = r_bp = {0, 10, 30}
#
#  Uso:
#    ./run_metrics_sweep.sh
#
#  Salida (en METRICS_ROOT):
#    n8_lat1_bp0/   metrics_summary.csv  latencies_detail.csv  bank_utilization.csv
#    n8_lat1_bp10/  ...
#    n8_lat1_bp30/  ...
#    n8_lat4_bp0/   ...
#    n8_lat4_bp10/  ...
#    n8_lat4_bp30/  ...
#    metrics_sweep_all.csv   ← concat de todos los metrics_summary.csv
# ============================================================

set -e

# ── Directorios ────────────────────────────────────────────
export RUN_DIR=${RUN_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/Simulaciones}

# Raíz donde se crean los subdirectorios por corrida
METRICS_ROOT="/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/Metricas"

# ── Configuraciones del sweep ──────────────────────────────
# Formato: "N_BANKS READ_LATENCY"
declare -a SWEEP_CONFIGS=(
    "8 1"
    "8 4"
)

# Niveles de backpressure (b_bp = r_bp)
BACKPRESSURES=(0 10 30)

# ── Función: una corrida del sweep ─────────────────────────
run_sweep_test() {
    local n_banks=$1
    local read_lat=$2
    local bp=$3

    local simv_name="mem_handler_simv_n${n_banks}_lat${read_lat}"
    local metrics_dir="${METRICS_ROOT}/n${n_banks}_lat${read_lat}_bp${bp}"
    local log_file="${metrics_dir}/run.log"

    echo ""
    echo "  → simv=${simv_name}  bp=${bp}"

    mkdir -p "${metrics_dir}"
    cd "${RUN_DIR}"

    if [[ ! -x "./${simv_name}" ]]; then
        echo "  ERROR: ${simv_name} no existe"
        return 1
    fi

    ./${simv_name} \
        +UVM_TESTNAME=mem_full_test \
        +UVM_VERBOSITY=UVM_LOW \
        +uvm_set_config_string=*,csv_dir,${metrics_dir} \
        +uvm_set_config_int=*,b_bp_value,${bp} \
        +uvm_set_config_int=*,r_bp_value,${bp} \
        +uvm_set_config_int=*,read_latency_val,${read_lat} \
        +B_BP=${bp} \
        +R_BP=${bp} \
        -l "${log_file}"

    # ── Parsear UVM Report Summary ──────────────────────────
    local fatal=$(grep -E "^UVM_FATAL\s*:" "${log_file}" | awk '{print $NF}')
    local error=$(grep -E "^UVM_ERROR\s*:" "${log_file}" | awk '{print $NF}')

    if [[ -z "${fatal}" ]] || [[ -z "${error}" ]]; then
        echo "  FAIL: n${n_banks}_lat${read_lat}_bp${bp} (no UVM summary — posible crash)"
        return 1
    elif [[ "${fatal}" != "0" ]]; then
        echo "  FAIL: n${n_banks}_lat${read_lat}_bp${bp} (UVM_FATAL=${fatal})"
        return 1
    elif [[ "${error}" != "0" ]]; then
        echo "  FAIL: n${n_banks}_lat${read_lat}_bp${bp} (UVM_ERROR=${error})"
        return 1
    else
        echo "  PASS: n${n_banks}_lat${read_lat}_bp${bp}"
        return 0
    fi
}

# ── Función: concat de todos los metrics_summary.csv ───────
build_sweep_csv() {
    echo ""
    echo "===== Building metrics_sweep_all.csv ====="
    python3 - << PYEOF
import pandas as pd, glob, re, os, sys

root    = "${METRICS_ROOT}"
pattern = os.path.join(root, "n*_lat*_bp*", "metrics_summary.csv")
files   = sorted(glob.glob(pattern))

if not files:
    print(f"[WARN] No se encontraron CSVs en: {pattern}")
    sys.exit(0)

dfs = []
for f in files:
    m = re.search(r'n(\d+)_lat(\d+)_bp(\d+)', f)
    if not m:
        continue
    df = pd.read_csv(f)
    # Insertar columnas de contexto si el scoreboard no las escribió
    if 'n_banks' not in df.columns:
        df.insert(1, 'n_banks',      int(m.group(1)))
    if 'read_latency' not in df.columns:
        df.insert(2, 'read_latency', int(m.group(2)))
    if 'bp_level' not in df.columns:
        df.insert(3, 'bp_level',     int(m.group(3)))
    dfs.append(df)
    print(f"  [N={m.group(1)} LAT={m.group(2)} BP={m.group(3)}] leído")

if not dfs:
    print("[ERROR] Ningún CSV válido encontrado.")
    sys.exit(1)

combined = pd.concat(dfs, ignore_index=True)
output   = os.path.join(root, "metrics_sweep_all.csv")
combined.to_csv(output, index=False)
print(f"\n✓ {output}")
print(f"  {len(combined)} fila(s)  ·  {len(combined.columns)} columnas")
PYEOF
}

# ── Main ───────────────────────────────────────────────────
echo "============================================="
echo "SWEEP DE BACKPRESSURE — $(date '+%Y-%m-%d %H:%M')"
echo "Configs : ${SWEEP_CONFIGS[*]}"
echo "BP levels: ${BACKPRESSURES[*]}"
echo "Salida  : ${METRICS_ROOT}"
echo "============================================="

mkdir -p "${METRICS_ROOT}"

PASS=0
FAIL=0

for cfg_entry in "${SWEEP_CONFIGS[@]}"; do
    n_banks=$(echo ${cfg_entry} | awk '{print $1}')
    read_lat=$(echo ${cfg_entry} | awk '{print $2}')

    echo ""
    echo "── N=${n_banks} LAT=${read_lat} ──────────────────────────────"

    for bp in "${BACKPRESSURES[@]}"; do
        if run_sweep_test ${n_banks} ${read_lat} ${bp}; then
            PASS=$((PASS+1))
        else
            FAIL=$((FAIL+1))
        fi
    done
done

# ── Resumen ────────────────────────────────────────────────
echo ""
echo "============================================="
echo "Sweep completo"
echo "Total: $((PASS+FAIL))   Pass: ${PASS}   Fail: ${FAIL}"
echo "============================================="

build_sweep_csv

echo ""
echo "Subdirectorios generados:"
ls -d "${METRICS_ROOT}"/n*_lat*_bp* 2>/dev/null | while read d; do
    echo "  $d"
    ls "$d"/*.csv 2>/dev/null | awk '{printf "    %s\n", $0}'
done

exit ${FAIL}