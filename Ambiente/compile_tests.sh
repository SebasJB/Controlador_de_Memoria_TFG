#!/bin/bash
# ============================================================
#  Script  : compile_tests.sh
#  Project : Banked Memory Controller — TFG ITCR
#
#  Compila el simv para una configuración específica del DUT.
#  Cada combinación (N_BANKS, READ_LATENCY) produce:
#    - Binario: mem_handler_simv_n${N_BANKS}_lat${READ_LAT}
#    - Cobertura: cov_n${N_BANKS}_lat${READ_LAT}.vdb
#
#  Uso:
#    ./compile_tests.sh                  # default: N=4, LAT=1
#    ./compile_tests.sh 8                # N=8, LAT=1 (default)
#    ./compile_tests.sh 8 4              # N=8, LAT=4
#    ./compile_tests.sh 16 1             # N=16, LAT=1
# ============================================================

set -e

# ── Parámetros de configuración (con defaults) ─────────────
N_BANKS=${1:-4}
READ_LAT=${2:-1}
BANK_SIZE_BYTES=${BANK_SIZE_BYTES:-8192}    # constante, IP real

echo ""
echo "============================="
echo "Compilando configuración"
echo "  N_BANKS         = ${N_BANKS}"
echo "  READ_LATENCY    = ${READ_LAT}"
echo "  BANK_SIZE_BYTES = ${BANK_SIZE_BYTES}"
echo "============================="

# ── Directorios ────────────────────────────────────────────
export RTL_DIR=${RTL_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/RTL}
export SVA_DIR=${SVA_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/RTL/SVA}
export TB_DIR=${TB_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente}
export RUN_DIR=${RUN_DIR:-/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/Simulaciones}

# ── Nombres derivados ──────────────────────────────────────
SIMV_NAME="mem_handler_simv_n${N_BANKS}_lat${READ_LAT}"
COV_DIR_NAME="cov_n${N_BANKS}_lat${READ_LAT}.vdb"
COMPILE_LOG="compile_n${N_BANKS}_lat${READ_LAT}.log"

# ── Compilación ────────────────────────────────────────────
cd ${RUN_DIR}

vcs -f ${TB_DIR}/mem_ctrl_uvm.f \
    -full64 \
    -ntb_opts uvm-1.2 \
    -cm line+cond+fsm+tgl+branch \
    -cm_dir ${RUN_DIR}/${COV_DIR_NAME} \
    +define+TB_N_BANKS=${N_BANKS} \
    +define+TB_READ_LATENCY=${READ_LAT} \
    +define+TB_BANK_SIZE_BYTES=${BANK_SIZE_BYTES} \
    +define+TEST_N_BANKS=${N_BANKS} \
    +define+TEST_BANK_SIZE_BYTES=${BANK_SIZE_BYTES} \
    -l ${COMPILE_LOG} \
    -o ${SIMV_NAME} \
    -debug_access+all -debug_region+cell

echo ""
echo "Compilación OK"
echo "  Binario: ${RUN_DIR}/${SIMV_NAME}"
echo "  Cov DB : ${RUN_DIR}/${COV_DIR_NAME}"
echo "  Log    : ${RUN_DIR}/${COMPILE_LOG}"
