// ============================================================
//  Patch : tb_top — sentencias bind para probe interfaces
//
//  Cómo integrar:
//    Copiar este bloque DENTRO del módulo tb_top, justo ANTES
//    del initial que llama a run_test().
//
//  Las sentencias bind conectan señales internas del DUT a las
//  probe interfaces, sin modificar el RTL.
// ============================================================

    // ── Instanciar las probe interfaces ──────────────────
    scheduler_probe_if sched_probe (.clk(clk), .rst_n(rst_n));
    bank_probe_if #(.N_BANKS(N_BANKS)) bank_probe (.clk(clk), .rst_n(rst_n));
    fifo_probe_if #(.WR_DEPTH(WR_REQ_FIFO_DEPTH),
                    .RD_DEPTH(RD_REQ_FIFO_DEPTH)) fifo_probe (.clk(clk), .rst_n(rst_n));

    // ── Bind al scheduler ────────────────────────────────
    // Mapea señales internas a la probe interface.
    // Nota: requiere visibilidad por path hierarchical.
    assign sched_probe.grant_wr     = dut.u_scheduler.grant_wr;
    assign sched_probe.grant_rd     = dut.u_scheduler.grant_rd;
    assign sched_probe.same_bank    = dut.u_scheduler.same_bank;
    assign sched_probe.wr_req_pndng = dut.wr_req_pndng;
    assign sched_probe.rd_req_pndng = dut.rd_req_pndng;
    assign sched_probe.wr_resp_full = dut.wr_resp_full;
    assign sched_probe.rob_tag_free = dut.rob_tag_free;
    assign sched_probe.wr_discard   = dut.u_scheduler.wr_discard;
    assign sched_probe.rd_discard   = dut.u_scheduler.rd_discard;
    assign sched_probe.wr_err_cnt   = dut.u_scheduler.wr_err_cnt;
    assign sched_probe.rd_err_cnt   = dut.u_scheduler.rd_err_cnt;

    // ── Bind a los banks (generate por banco) ────────────
    generate
        for (genvar gb = 0; gb < N_BANKS; gb = gb + 1) begin : g_bank_probe
            assign bank_probe.bank_busy[gb]   = dut.bank_busy[gb];
            assign bank_probe.bank_fsm_st[gb] = dut.gen_bank[gb].u_bank.u_fsm.state;
        end
    endgenerate

    // ── Bind a las FIFOs REQ ─────────────────────────────
    // El campo count está expuesto en fifo_flops como reg interno
    assign fifo_probe.wr_req_count  = dut.u_wr_req_fifo.count;
    assign fifo_probe.rd_req_count  = dut.u_rd_req_fifo.count;
    assign fifo_probe.wr_resp_count = dut.u_wr_response_path.u_counter.cnt;

    // ── Publicar las probe vifs ──────────────────────────
    initial begin
        uvm_config_db#(virtual scheduler_probe_if)::set(
            null, "uvm_test_top.env.cov", "sched_vif", sched_probe);
        uvm_config_db#(virtual bank_probe_if #(N_BANKS))::set(
            null, "uvm_test_top.env.cov", "bank_vif", bank_probe);
        uvm_config_db#(virtual fifo_probe_if #(WR_REQ_FIFO_DEPTH, RD_REQ_FIFO_DEPTH))::set(
            null, "uvm_test_top.env.cov", "fifo_vif", fifo_probe);
    end
