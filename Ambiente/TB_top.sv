// ============================================================
//  File    : tb_top.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Verification Environment — TB top
//
//  Propósito:
//    Módulo top del testbench. Instancia:
//      - Clock generator
//      - Reset generator
//      - mem_bank_interface (AXI4-Lite)
//      - DUT: mem_handler_top (con sus parámetros reales)
//      - run_test() de UVM
//
//  Parámetros del DUT — fijados aquí, propagados al env vía
//  uvm_config_db. Cambiar acá impacta interfaz e instancias.
// ============================================================

`timescale 1ns/1ps 

module tb_top;

    import uvm_pkg::*;
    import mem_ctrl_pkg::*;

    // ── Defines parametrizables (sobreescribibles con +define+) ──
    `ifndef TB_N_BANKS
        `define TB_N_BANKS 4
    `endif
    `ifndef TB_BANK_SIZE_BYTES
        `define TB_BANK_SIZE_BYTES 8192
    `endif
    `ifndef TB_READ_LATENCY
        `define TB_READ_LATENCY 1
    `endif

    // ── Parámetros concretos del DUT ─────────────────────
    localparam int ADDR_W            = 32;
    localparam int DATA_W            = 32;
    localparam int N_BANKS           = `TB_N_BANKS;
    localparam int BANK_SIZE_BYTES   = `TB_BANK_SIZE_BYTES;
    localparam int READ_LATENCY      = `TB_READ_LATENCY;
    localparam int LAT_CNT_W         = 3;
    localparam int WR_REQ_FIFO_DEPTH = 8;
    localparam int RD_REQ_FIFO_DEPTH = 8;
    localparam int WR_RESP_DEPTH     = 8;
    localparam int ERR_CNT_W         = 16;

    // ── Clock & reset ────────────────────────────────────
    logic clk;
    logic rst_n;

    initial clk = 1'b0;
    always #5 clk = ~clk;   // 100 MHz

    initial begin
    axi_if.awvalid = 1'b0;
    axi_if.wvalid  = 1'b0;
    axi_if.bready  = 1'b0;
    axi_if.arvalid = 1'b0;
    axi_if.rready  = 1'b0;
    
    rst_n = 1'b0;
    repeat (16) @(posedge clk);
    rst_n = 1'b1;
    end

    // ── Interfaz AXI4-Lite ───────────────────────────────
    mem_bank_interface #(
        .ADDR_W(ADDR_W),
        .DATA_W(DATA_W)
    ) axi_if (
        .clk  (clk),
        .rst_n(rst_n)
    );

    // ── DUT: mem_handler_top ─────────────────────────────
    mem_handler_top #(
        .ADDR_W           (ADDR_W),
        .AXI_DATA_WIDTH   (DATA_W),
        .N_BANKS          (N_BANKS),
        .BANK_SIZE_BYTES  (BANK_SIZE_BYTES),
        .READ_LATENCY     (READ_LATENCY),
        .LAT_CNT_W        (LAT_CNT_W),
        .WR_REQ_FIFO_DEPTH(WR_REQ_FIFO_DEPTH),
        .RD_REQ_FIFO_DEPTH(RD_REQ_FIFO_DEPTH),
        .WR_RESP_DEPTH    (WR_RESP_DEPTH),
        .ERR_CNT_W        (ERR_CNT_W)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        // AW
        .s_axi_awvalid(axi_if.awvalid),
        .s_axi_awready(axi_if.awready),
        .s_axi_awaddr (axi_if.awaddr),
        // W
        .s_axi_wvalid (axi_if.wvalid),
        .s_axi_wready (axi_if.wready),
        .s_axi_wdata  (axi_if.wdata),
        .s_axi_wstrb  (axi_if.wstrb),
        // B
        .s_axi_bvalid (axi_if.bvalid),
        .s_axi_bresp  (axi_if.bresp),
        .s_axi_bready (axi_if.bready),
        // AR
        .s_axi_arvalid(axi_if.arvalid),
        .s_axi_arready(axi_if.arready),
        .s_axi_araddr (axi_if.araddr),
        // R
        .s_axi_rvalid (axi_if.rvalid),
        .s_axi_rdata  (axi_if.rdata),
        .s_axi_rresp  (axi_if.rresp),
        .s_axi_rready (axi_if.rready),
        // Métricas
        .wr_err_cnt   (),    // se conectan vía probe en cg/env
        .rd_err_cnt   ()
    );

    // ── Probe interfaces (bind no-intrusivo al DUT) ──────
    // Estas 3 interfaces dan acceso pasivo al coverage
    // subscriber a señales internas del scheduler, los bank
    // controllers y los contadores de FIFO, sin modificar RTL.
    scheduler_probe_if sched_probe (.clk(clk), .rst_n(rst_n));

    bank_probe_if #(
        .N_BANKS(N_BANKS)
    ) bank_probe (
        .clk  (clk),
        .rst_n(rst_n)
    );

    fifo_probe_if #(
        .WR_DEPTH     (WR_REQ_FIFO_DEPTH),
        .RD_DEPTH     (RD_REQ_FIFO_DEPTH),
        .WR_RESP_DEPTH(WR_RESP_DEPTH)
    ) fifo_probe (
        .clk  (clk),
        .rst_n(rst_n)
    );

    // ── Cross-hierarchical assigns al scheduler ──────────
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

    // ── Cross-hierarchical assigns a los bank controllers ─
    genvar gb;
    generate
        for (gb = 0; gb < N_BANKS; gb = gb + 1) begin : g_bank_probe
            assign bank_probe.bank_busy[gb]   = dut.bank_busy[gb];
            assign bank_probe.bank_fsm_st[gb] = dut.gen_bank[gb].u_bank.u_fsm.state;
        end
    endgenerate

    // ── Cross-hierarchical assigns a las FIFOs y al pending counter ─
    //assign fifo_probe.wr_req_count  = dut.u_wr_req_fifo.count;
    //assign fifo_probe.rd_req_count  = dut.u_rd_req_fifo.count;
    assign fifo_probe.wr_resp_count = dut.u_wr_response_path.u_counter.cnt;

    // ── UVM config & run ─────────────────────────────────
    initial begin
        // Publicar la interfaz AXI4-Lite para que los agentes la encuentren
        uvm_config_db#(virtual mem_bank_interface#(ADDR_W, DATA_W))::set(
            null, "uvm_test_top.*", "axi_vif", axi_if
        );

        // Publicar parámetros del DUT al env
        uvm_config_db#(int)::set(null, "uvm_test_top.*", "ADDR_W",          ADDR_W);
        uvm_config_db#(int)::set(null, "uvm_test_top.*", "DATA_W",          DATA_W);
        uvm_config_db#(int)::set(null, "uvm_test_top.*", "N_BANKS",         N_BANKS);
        uvm_config_db#(int)::set(null, "uvm_test_top.*", "BANK_SIZE_BYTES", BANK_SIZE_BYTES);

        // Publicar las probe vifs hacia el coverage subscriber.
        // Path: "uvm_test_top.env.cov*" — wildcard final por
        // robustez, alineado con el resto de los set() arriba.
        // CRÍTICO: los tipos virtuales acá deben coincidir
        // EXACTAMENTE con los tipos que el subscriber usa en
        // sus get(). El subscriber instancia:
        //   virtual fifo_probe_if #(WR_FIFO_DEPTH, RD_FIFO_DEPTH)
        // con solo 2 parámetros; por eso acá también usamos
        // 2 parámetros (WR_RESP_DEPTH toma su default).
        uvm_config_db#(virtual scheduler_probe_if)::set(
            null, "uvm_test_top.env.cov*", "sched_vif", sched_probe);
        uvm_config_db#(virtual bank_probe_if #(N_BANKS))::set(
            null, "uvm_test_top.env.cov*", "bank_vif", bank_probe);
        uvm_config_db#(virtual fifo_probe_if #(WR_REQ_FIFO_DEPTH, RD_REQ_FIFO_DEPTH))::set(
            null, "uvm_test_top.env.cov*", "fifo_vif", fifo_probe);

        run_test();
    end

    // ── Timeout global (escape de hangs en regression) ───
    initial begin
        #10_000_000;   // 10 ms = 1000k ciclos @100MHz
        `uvm_fatal("TB_TOP", "Global timeout reached")
    end

    initial begin
        $display("================================================");
        $display("[TB_TOP] PARAMS: N_BANKS=%0d  READ_LAT=%0d  BANK_SIZE=%0d",
                 N_BANKS, READ_LATENCY, BANK_SIZE_BYTES);
        $display("[TB_TOP] DUT inst: dut.N_BANKS=%0d  dut.READ_LATENCY=%0d",
                 dut.N_BANKS, dut.READ_LATENCY);
        $display("================================================");
    end

    // ── Waveform dump ────────────────────────────────────
    initial begin
        $dumpfile("tb_mem_ctrl.vcd");
        $dumpvars(0, tb_top);
        $dumpvars(0, tb_top.dut.u_rd_response_path.u_reorder_buffer);
    end

endmodule
