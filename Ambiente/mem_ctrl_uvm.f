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
${RTL_DIR}/fifo.sv
${RTL_DIR}/addr_decoder.sv
${RTL_DIR}/axi4_lite_front_end.sv
${RTL_DIR}/sram_bank_controller.sv
${RTL_DIR}/scheduler.sv
${RTL_DIR}/wr_response_path.sv
${RTL_DIR}/rd_response_path.sv
${RTL_DIR}/mem_handler_top.sv

// ── SVA bind modules (opcional, comentar si no se desean) ──
${SVA_DIR}/scheduler_sva.sv
${SVA_DIR}/sram_bank_controller_sva.sv
${SVA_DIR}/wr_response_path_sva.sv
${SVA_DIR}/rd_response_path_sva.sv
${SVA_DIR}/axi4_lite_front_end_sva.sv

// ── Verification interfaces ────────────────────────────────
${TB_DIR}/Mem_bank_interface.sv
${TB_DIR}/probe_interfaces.sv

// ── UVM package + tb_top ───────────────────────────────────
+incdir+${TB_DIR}
${TB_DIR}/mem_ctrl_pkg.sv
${TB_DIR}/TB_top.sv

// ── Flags VCS ──────────────────────────────────────────────
-sverilog
+define+UVM_NO_DEPRECATED
-timescale=1ns/1ps
-debug_access+all
+vcs+lic+wait
