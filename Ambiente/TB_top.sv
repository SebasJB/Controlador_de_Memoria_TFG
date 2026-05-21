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

    // ── Parámetros concretos del DUT ─────────────────────
    localparam int ADDR_W            = 32;
    localparam int DATA_W            = 32;
    localparam int N_BANKS           = 4;
    localparam int BANK_SIZE_BYTES   = 1024;
    localparam int READ_LATENCY      = 1;
    localparam int LAT_CNT_W         = 8;
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
        rst_n = 1'b0;
        repeat (16) @(posedge clk);   // 16 ciclos de reset (margen seguro)
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

    // ── UVM config & run ─────────────────────────────────
    initial begin
        // Publicar la interfaz para que los agentes la encuentren
        uvm_config_db#(virtual mem_bank_interface#(ADDR_W, DATA_W))::set(
            null, "uvm_test_top.*", "axi_vif", axi_if
        );

        // Publicar parámetros del DUT al env
        uvm_config_db#(int)::set(null, "uvm_test_top.*", "ADDR_W",          ADDR_W);
        uvm_config_db#(int)::set(null, "uvm_test_top.*", "DATA_W",          DATA_W);
        uvm_config_db#(int)::set(null, "uvm_test_top.*", "N_BANKS",         N_BANKS);
        uvm_config_db#(int)::set(null, "uvm_test_top.*", "BANK_SIZE_BYTES", BANK_SIZE_BYTES);

        run_test();
    end

    // ── Timeout global (escape de hangs en regression) ───
    initial begin
        #5_000_000;   // 5 ms = 500k ciclos @100MHz
        `uvm_fatal("TB_TOP", "Global timeout reached")
    end

    // ── Waveform dump ────────────────────────────────────
    initial begin
        $dumpfile("tb_mem_ctrl.vcd");
        $dumpvars(0, tb_top);
    end

endmodule
