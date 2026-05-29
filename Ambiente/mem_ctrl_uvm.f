// ============================================================
//  Filelist : mem_ctrl_uvm.f
//  Project  : Banked Memory Controller — TFG ITCR
//  Uso      : vcs -f mem_ctrl_uvm.f
//
//  Orden:
//    1. RTL del DUT (orden de dependencias)
//    2. Interfaces de verificación (axi + probes)
//    3. Package UVM (auto-incluye todo el environment)
//    4. tb_top
// ============================================================

// ── RTL DUT ─────────────────────────────────────────────────
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/RTL/fifo.sv
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/RTL/addr_decoder.sv
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/RTL/axi4_lite_front_end.sv
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/RTL/sram_bank_controller.sv
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/RTL/scheduler.sv
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/RTL/wr_response_path.sv
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/RTL/rd_response_path.sv
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/RTL/mem_handler_top.sv

// ── SVA bind modules (opcional, comentar si no se desean) ──
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/SVA/scheduler_sva.sv
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/SVA/sram_bank_controller_sva.sv
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/SVA/wr_response_path_sva.sv
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/SVA/rd_response_path_sva.sv
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/SVA/axi4_lite_front_end_sva.sv

// ── Verification interfaces ────────────────────────────────
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/Mem_bank_interface.sv
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/probe_interfaces.sv

// ── UVM package + tb_top ───────────────────────────────────
//+incdir+${TB_DIR}
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/mem_ctrl_pkg.sv
/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/TB_top.sv

// ── Flags VCS ──────────────────────────────────────────────
-sverilog
+define+UVM_NO_DEPRECATED
-timescale=1ns/1ps
//-debug_access+all
+vcs+lic+wait
