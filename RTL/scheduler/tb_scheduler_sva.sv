// ============================================================
//  File    : tb_scheduler_sva.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : Testbench para Scheduler + SVA bind
//
//  Compilar en VCS:
//    vcs -sverilog -timescale=1ns/1ps \
//        scheduler.sv scheduler_sva.sv tb_scheduler_sva.sv \
//        -o simv && ./simv
//
//  Parámetros del sistema (deben coincidir con el DUT):
//    N_BANKS=4  ADDR_W=32  AXI_DATA_WIDTH=32  BANK_SIZE_BYTES=1024
//
//  Convención de tiempo:
//    Los estímulos se aplican en la mitad negativa del ciclo.
//    Las verificaciones ocurren #1 DESPUÉS del siguiente posedge,
//    cuando toda la cadena combinacional ya propagó en VCS.
//
//    Patrón de cada TC:
//      idle_inputs;           // limpia entradas
//      <aplica estímulos>;    // señales del caso
//      @(posedge clk); #1;   // avanza al flanco y espera 1ns
//      chk(...);              // verifica aquí — señales estables
//
//  Clocking : flanco positivo (clk), período = 10 ns
//  Reset    : síncrono activo en bajo (rst_n)
// ============================================================

`timescale 1ns/1ps

module tb_scheduler_sva;

    // ── Parámetros del sistema ─────────────────────────────
    localparam ADDR_W          = 32;
    localparam AXI_DATA_WIDTH  = 32;
    localparam N_BANKS         = 4;
    localparam BANK_SIZE_BYTES = 1024;
    localparam ERR_CNT_W       = 16;
    localparam CLK_HALF        = 5;

    // ── Parámetros derivados ───────────────────────────────
    localparam BANK_BITS       = 2;
    localparam BANK_ADDR_WIDTH = 8;
    localparam WR_BUS_MSB      = ADDR_W + AXI_DATA_WIDTH + AXI_DATA_WIDTH/8;

    // ── Reloj y reset ──────────────────────────────────────
    reg clk;
    reg rst_n;

    // ── Entradas al DUT ────────────────────────────────────
    reg [BANK_BITS-1:0]       wr_bank_id;
    reg [BANK_BITS-1:0]       rd_bank_id;
    reg [BANK_ADDR_WIDTH-1:0] wr_bank_word_addr;
    reg [BANK_ADDR_WIDTH-1:0] rd_bank_word_addr;
    reg                       wr_addr_valid;
    reg                       rd_addr_valid;
    reg                       wr_req_pndng;
    reg                       rd_req_pndng;
    reg [WR_BUS_MSB:0]        wr_req_data;
    reg  [N_BANKS-1:0]        bank_busy;        // packed → propagación inmediata
    wire                      bank_busy_arr [0:N_BANKS-1];
    reg                       rob_tag_free;
    reg                       wr_resp_full;

    // ── Wire array para conectar bank_busy packed al DUT ──
    assign bank_busy_arr[0] = bank_busy[0];
    assign bank_busy_arr[1] = bank_busy[1];
    assign bank_busy_arr[2] = bank_busy[2];
    assign bank_busy_arr[3] = bank_busy[3];

    // ── Salidas del DUT ────────────────────────────────────
    wire                       wr_req_pop;
    wire                       rd_req_pop;
    wire                       bank_req_valid [0:N_BANKS-1];
    wire                       bank_req_op    [0:N_BANKS-1];
    wire [BANK_ADDR_WIDTH-1:0] bank_req_addr  [0:N_BANKS-1];
    wire [AXI_DATA_WIDTH-1:0]  bank_req_wdata;
    wire [AXI_DATA_WIDTH/8-1:0]bank_req_wstrb;
    wire [BANK_BITS-1:0]       bank_req_tag;
    wire [BANK_BITS:0]         wr_ptr_ext;
    wire [ERR_CNT_W-1:0]       wr_err_cnt;
    wire [ERR_CNT_W-1:0]       rd_err_cnt;

    // ── Vector packed de bank_req_valid para verificación ─
    wire [N_BANKS-1:0] brv;
    assign brv[0] = bank_req_valid[0];
    assign brv[1] = bank_req_valid[1];
    assign brv[2] = bank_req_valid[2];
    assign brv[3] = bank_req_valid[3];

    // ── Contadores globales ────────────────────────────────
    integer pass_cnt;
    integer fail_cnt;

    // ── Instancia DUT ─────────────────────────────────────
    scheduler #(
        .ADDR_W         (ADDR_W),
        .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
        .N_BANKS        (N_BANKS),
        .BANK_SIZE_BYTES(BANK_SIZE_BYTES),
        .ERR_CNT_W      (ERR_CNT_W)
    ) u_dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .wr_bank_id       (wr_bank_id),
        .rd_bank_id       (rd_bank_id),
        .wr_bank_word_addr(wr_bank_word_addr),
        .rd_bank_word_addr(rd_bank_word_addr),
        .wr_addr_valid    (wr_addr_valid),
        .rd_addr_valid    (rd_addr_valid),
        .wr_req_pndng     (wr_req_pndng),
        .rd_req_pndng     (rd_req_pndng),
        .wr_req_data      (wr_req_data),
        .bank_busy        (bank_busy_arr),
        .rob_tag_free     (rob_tag_free),
        .wr_resp_full     (wr_resp_full),
        .wr_req_pop       (wr_req_pop),
        .rd_req_pop       (rd_req_pop),
        .bank_req_valid   (bank_req_valid),
        .bank_req_op      (bank_req_op),
        .bank_req_addr    (bank_req_addr),
        .bank_req_wdata   (bank_req_wdata),
        .bank_req_wstrb   (bank_req_wstrb),
        .bank_req_tag     (bank_req_tag),
        .wr_ptr_ext       (wr_ptr_ext),
        .wr_err_cnt       (wr_err_cnt),
        .rd_err_cnt       (rd_err_cnt)
    );

    // ── Generador de reloj ─────────────────────────────────
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    // ============================================================
    // TASK: chk
    // ============================================================
    task chk;
        input        cond;
        input [63:0] tc;
        input [255:0] msg;
        begin
            if (cond) begin
                pass_cnt = pass_cnt + 1;
                $display("  [PASS] TC%0d: %s", tc, msg);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("  [FAIL] TC%0d: %s", tc, msg);
            end
        end
    endtask

    // ============================================================
    // TASK: all_banks_idle
    // ============================================================
    task all_banks_idle;
        begin
            bank_busy = {N_BANKS{1'b0}};
        end
    endtask

    // ============================================================
    // TASK: all_banks_busy
    // ============================================================
    task all_banks_busy;
        begin
            bank_busy = {N_BANKS{1'b1}};
        end
    endtask

    // ============================================================
    // TASK: idle_inputs — estado neutro entre casos de prueba
    // ============================================================
    task idle_inputs;
        begin
            wr_req_pndng      = 1'b0;
            rd_req_pndng      = 1'b0;
            wr_addr_valid     = 1'b0;
            rd_addr_valid     = 1'b0;
            wr_bank_id        = 2'b00;
            rd_bank_id        = 2'b00;
            wr_bank_word_addr = {BANK_ADDR_WIDTH{1'b0}};
            rd_bank_word_addr = {BANK_ADDR_WIDTH{1'b0}};
            wr_req_data       = {(WR_BUS_MSB+1){1'b0}};
            rob_tag_free      = 1'b1;
            wr_resp_full      = 1'b0;
            all_banks_idle;
        end
    endtask

    // ============================================================
    // TASK: apply_reset
    // ============================================================
    task apply_reset;
        begin
            rst_n = 1'b0;
            idle_inputs;
            @(posedge clk); #1;
            @(posedge clk); #1;
            rst_n = 1'b1;
            @(posedge clk); #1;
        end
    endtask

    // ============================================================
    // MAIN — Secuencia de 23 casos de prueba
    // ============================================================
    integer       i;
    reg [ERR_CNT_W-1:0] prev_wr_err;
    reg [ERR_CNT_W-1:0] prev_rd_err;
    reg           mono_ok;

    initial begin
        pass_cnt = 0;
        fail_cnt = 0;

        // ── Inicialización explícita — evita X en tiempo 0 ──
        rst_n             = 1'b0;
        wr_req_pndng      = 1'b0;
        rd_req_pndng      = 1'b0;
        wr_addr_valid     = 1'b0;
        rd_addr_valid     = 1'b0;
        wr_bank_id        = 2'b00;
        rd_bank_id        = 2'b00;
        wr_bank_word_addr = 8'h00;
        rd_bank_word_addr = 8'h00;
        wr_req_data       = {(WR_BUS_MSB+1){1'b0}};
        rob_tag_free      = 1'b1;
        wr_resp_full      = 1'b0;
        bank_busy         = {N_BANKS{1'b0}};

        $display("=====================================================");
        $display("  TB_SCHEDULER_SVA — Inicio de simulacion");
        $display("=====================================================");

        apply_reset;

        // ==========================================================
        // TC01 — Estado post-reset
        // ==========================================================
        $display("\n[TC01] Estado post-reset");
        idle_inputs;
        @(posedge clk); #1;
        chk(!wr_req_pop,          1, "wr_req_pop = 0 tras reset");
        chk(!rd_req_pop,          1, "rd_req_pop = 0 tras reset");
        chk(brv == 4'b0000,       1, "ningun banco activo tras reset");
        chk(wr_err_cnt == 16'd0,  1, "wr_err_cnt = 0 tras reset");
        chk(rd_err_cnt == 16'd0,  1, "rd_err_cnt = 0 tras reset");

        // ==========================================================
        // TC02 — WR-only, banco 0
        // SVA: a_grant_wr_valid, a_dispatch_wr_coherent,
        //      a_wr_pop_reason, a_bank_req_valid_max2
        // ==========================================================
        $display("\n[TC02] WR-only dispatch — banco 0");
        idle_inputs;
        wr_req_pndng      = 1'b1;
        wr_addr_valid     = 1'b1;
        wr_bank_id        = 2'd0;
        wr_bank_word_addr = 8'hAA;
        wr_req_data       = {1'b1, 32'h0000_0000, 32'hDEAD_BEEF, 4'hF};
        bank_busy[0]      = 1'b0;
        @(posedge clk); #1;
        chk(wr_req_pop == 1'b1,           2, "wr_req_pop = 1 (WR grant)");
        chk(rd_req_pop == 1'b0,           2, "rd_req_pop = 0 (sin RD)");
        chk(bank_req_valid[0] == 1'b1,    2, "banco 0 activo");
        chk(bank_req_op[0]    == 1'b1,    2, "op = WR en banco 0");
        chk(brv == 4'b0001,               2, "one-hot: solo banco 0");

        // ==========================================================
        // TC03 — RD-only, banco 2
        // SVA: a_grant_rd_valid, a_dispatch_rd_coherent,
        //      a_rd_pop_reason, a_bank_req_valid_max2
        // ==========================================================
        $display("\n[TC03] RD-only dispatch — banco 2");
        idle_inputs;
        rd_req_pndng      = 1'b1;
        rd_addr_valid     = 1'b1;
        rd_bank_id        = 2'd2;
        rd_bank_word_addr = 8'h55;
        bank_busy[2]      = 1'b0;
        rob_tag_free      = 1'b1;
        @(posedge clk); #1;
        chk(rd_req_pop == 1'b1,           3, "rd_req_pop = 1 (RD grant)");
        chk(wr_req_pop == 1'b0,           3, "wr_req_pop = 0 (sin WR)");
        chk(bank_req_valid[2] == 1'b1,    3, "banco 2 activo");
        chk(bank_req_op[2]    == 1'b0,    3, "op = RD en banco 2");
        chk(brv == 4'b0100,               3, "one-hot: solo banco 2");

        // ==========================================================
        // TC04 — WR banco 1 + RD banco 3 (bancos distintos)
        // SVA: both_no_conflict, a_bank_req_valid_2hot_distinct,
        //      a_double_grant_distinct_banks
        // ==========================================================
        $display("\n[TC04] WR banco 1 + RD banco 3 (bancos distintos)");
        idle_inputs;
        wr_req_pndng      = 1'b1;
        wr_addr_valid     = 1'b1;
        wr_bank_id        = 2'd1;
        wr_bank_word_addr = 8'h10;
        wr_req_data       = {1'b1, 32'h4, 32'hCAFE_BABE, 4'hF};
        bank_busy[1]      = 1'b0;
        rd_req_pndng      = 1'b1;
        rd_addr_valid     = 1'b1;
        rd_bank_id        = 2'd3;
        rd_bank_word_addr = 8'h20;
        bank_busy[3]      = 1'b0;
        rob_tag_free      = 1'b1;
        @(posedge clk); #1;
        chk(wr_req_pop == 1'b1,           4, "wr_req_pop = 1 (WR grant dual)");
        chk(rd_req_pop == 1'b1,           4, "rd_req_pop = 1 (RD grant dual)");
        chk(bank_req_valid[1] == 1'b1,    4, "banco 1 activo (WR)");
        chk(bank_req_valid[3] == 1'b1,    4, "banco 3 activo (RD)");
        chk(bank_req_op[1]    == 1'b1,    4, "op = WR en banco 1");
        chk(bank_req_op[3]    == 1'b0,    4, "op = RD en banco 3");
        chk(bank_req_valid[0] == 1'b0,    4, "banco 0 inactivo");
        chk(bank_req_valid[2] == 1'b0,    4, "banco 2 inactivo");
        chk(brv == 4'b1010,               4, "exactamente bancos 1 y 3 activos");

        // ==========================================================
        // TC05 — WRITE-FIRST: conflicto banco 0
        // SVA: a_write_first
        // ==========================================================
        $display("\n[TC05] WRITE-FIRST — conflicto banco 0");
        idle_inputs;
        wr_req_pndng      = 1'b1;
        wr_addr_valid     = 1'b1;
        wr_bank_id        = 2'd0;
        wr_bank_word_addr = 8'h01;
        wr_req_data       = {1'b1, 32'h0, 32'hAAAA_AAAA, 4'hF};
        bank_busy[0]      = 1'b0;
        rd_req_pndng      = 1'b1;
        rd_addr_valid     = 1'b1;
        rd_bank_id        = 2'd0;
        rd_bank_word_addr = 8'h01;
        rob_tag_free      = 1'b1;
        @(posedge clk); #1;
        chk(wr_req_pop == 1'b1,           5, "wr_req_pop = 1 (WR gana conflicto)");
        chk(rd_req_pop == 1'b0,           5, "rd_req_pop = 0 (RD pospuesto)");
        chk(bank_req_valid[0] == 1'b1,    5, "banco 0 activo con WR");
        chk(bank_req_op[0]    == 1'b1,    5, "op = WR en conflicto");
        chk(brv == 4'b0001,               5, "one-hot: solo banco 0 en conflicto");

        // ==========================================================
        // TC06 — WR con dirección inválida → wr_discard
        // SVA: a_wr_ok_discard_mutex, a_wr_pop_reason,
        //      a_wr_err_monotonic
        // Nota: prev_wr_err se lee DESPUÉS del posedge porque
        //       wr_err_cnt es secuencial — su valor post-ciclo
        //       es el que corresponde verificar en el ciclo siguiente.
        // ==========================================================
        $display("\n[TC06] WR addr invalida — discard");
        idle_inputs;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b0;
        wr_bank_id    = 2'd1;
        @(posedge clk); #1;
        prev_wr_err = wr_err_cnt;
        chk(wr_req_pop == 1'b1,           6, "wr_req_pop = 1 (pop por discard)");
        chk(rd_req_pop == 1'b0,           6, "rd_req_pop = 0 (sin RD)");
        chk(brv == 4'b0000,               6, "ningun banco activo en discard WR");
        @(posedge clk); #1;
        chk(wr_err_cnt == prev_wr_err + 1, 6, "wr_err_cnt incremento en 1");

        // ==========================================================
        // TC07 — RD con dirección inválida → rd_discard
        // SVA: a_rd_ok_discard_mutex, a_rd_pop_reason,
        //      a_rd_err_monotonic
        // ==========================================================
        $display("\n[TC07] RD addr invalida — discard");
        idle_inputs;
        rd_req_pndng  = 1'b1;
        rd_addr_valid = 1'b0;
        rd_bank_id    = 2'd2;
        rob_tag_free  = 1'b1;
        @(posedge clk); #1;
        prev_rd_err = rd_err_cnt;
        chk(rd_req_pop == 1'b1,           7, "rd_req_pop = 1 (pop por discard)");
        chk(wr_req_pop == 1'b0,           7, "wr_req_pop = 0 (sin WR)");
        chk(brv == 4'b0000,               7, "ningun banco activo en discard RD");
        @(posedge clk); #1;
        chk(rd_err_cnt == prev_rd_err + 1, 7, "rd_err_cnt incremento en 1");

        // ==========================================================
        // TC08 — WR banco ocupado → sin grant ni pop
        // SVA: a_grant_wr_valid (precondición wr_bank_free)
        // ==========================================================
        $display("\n[TC08] WR banco ocupado — sin despacho");
        idle_inputs;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b1;
        wr_bank_id    = 2'd2;
        bank_busy[2]  = 1'b1;
        @(posedge clk); #1;
        chk(wr_req_pop == 1'b0,           8, "wr_req_pop = 0 (banco busy)");
        chk(brv == 4'b0000,               8, "ningun banco activo con banco busy");

        // ==========================================================
        // TC09 — RD sin rob_tag_free → sin grant ni pop
        // SVA: a_grant_rd_valid (precondición rob_tag_free)
        // ==========================================================
        $display("\n[TC09] RD con ROB lleno — sin despacho");
        idle_inputs;
        rd_req_pndng  = 1'b1;
        rd_addr_valid = 1'b1;
        rd_bank_id    = 2'd1;
        bank_busy[1]  = 1'b0;
        rob_tag_free  = 1'b0;
        @(posedge clk); #1;
        chk(rd_req_pop == 1'b0,           9, "rd_req_pop = 0 (ROB lleno)");
        chk(brv == 4'b0000,               9, "ningun banco activo con ROB lleno");

        // ==========================================================
        // TC10 — WR+RD mismo banco ocupado → ninguno despacha
        // ==========================================================
        $display("\n[TC10] WR+RD mismo banco ocupado — ninguno despacha");
        idle_inputs;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b1;
        wr_bank_id    = 2'd3;
        bank_busy[3]  = 1'b1;
        rd_req_pndng  = 1'b1;
        rd_addr_valid = 1'b1;
        rd_bank_id    = 2'd3;
        rob_tag_free  = 1'b1;
        @(posedge clk); #1;
        chk(wr_req_pop == 1'b0,           10, "wr_req_pop = 0 (ambos busy)");
        chk(rd_req_pop == 1'b0,           10, "rd_req_pop = 0 (ambos busy)");
        chk(brv == 4'b0000,               10, "ningun banco activo");

        // ==========================================================
        // TC11 — Sin requests pendientes → idle total
        // ==========================================================
        $display("\n[TC11] Sin requests pendientes — idle total");
        idle_inputs;
        @(posedge clk); #1;
        chk(wr_req_pop == 1'b0,           11, "wr_req_pop = 0 en idle");
        chk(rd_req_pop == 1'b0,           11, "rd_req_pop = 0 en idle");
        chk(brv == 4'b0000,               11, "ningun banco activo en idle");

        // ==========================================================
        // TC12 — WR discard + RD grant simultáneos
        // SVA: a_wr_pop_reason(discard) + a_rd_pop_reason(grant)
        // ==========================================================
        $display("\n[TC12] WR discard + RD grant simultaneos");
        idle_inputs;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b0;
        wr_bank_id    = 2'd0;
        rd_req_pndng  = 1'b1;
        rd_addr_valid = 1'b1;
        rd_bank_id    = 2'd1;
        bank_busy[1]  = 1'b0;
        rob_tag_free  = 1'b1;
        @(posedge clk); #1;
        prev_wr_err = wr_err_cnt;
        chk(wr_req_pop == 1'b1,           12, "wr_req_pop = 1 (discard)");
        chk(rd_req_pop == 1'b1,           12, "rd_req_pop = 1 (grant RD)");
        chk(bank_req_valid[1] == 1'b1,    12, "banco 1 activo (RD grant)");
        chk(bank_req_op[1]    == 1'b0,    12, "op = RD en banco 1");
        chk(bank_req_valid[0] == 1'b0,    12, "banco 0 inactivo (WR discard)");
        @(posedge clk); #1;
        chk(wr_err_cnt == prev_wr_err + 1, 12, "wr_err_cnt incremento (discard sim.)");

        // ==========================================================
        // TC13 — WR grant + RD discard simultáneos
        // SVA: a_wr_pop_reason(grant) + a_rd_pop_reason(discard)
        // ==========================================================
        $display("\n[TC13] WR grant + RD discard simultaneos");
        idle_inputs;
        wr_req_pndng      = 1'b1;
        wr_addr_valid     = 1'b1;
        wr_bank_id        = 2'd2;
        wr_bank_word_addr = 8'h30;
        wr_req_data       = {1'b1, 32'h8, 32'h1234_5678, 4'hA};
        bank_busy[2]      = 1'b0;
        rd_req_pndng      = 1'b1;
        rd_addr_valid     = 1'b0;
        rd_bank_id        = 2'd3;
        rob_tag_free      = 1'b1;
        @(posedge clk); #1;
        prev_rd_err = rd_err_cnt;
        chk(wr_req_pop == 1'b1,           13, "wr_req_pop = 1 (WR grant)");
        chk(rd_req_pop == 1'b1,           13, "rd_req_pop = 1 (RD discard)");
        chk(bank_req_valid[2] == 1'b1,    13, "banco 2 activo (WR)");
        chk(bank_req_op[2]    == 1'b1,    13, "op = WR en banco 2");
        chk(bank_req_valid[3] == 1'b0,    13, "banco 3 inactivo (RD discard)");
        @(posedge clk); #1;
        chk(rd_err_cnt == prev_rd_err + 1, 13, "rd_err_cnt incremento (discard sim.)");

        // ==========================================================
        // TC14 — Monotonicidad: ráfaga de 8 discards WR+RD
        // SVA: a_wr_err_monotonic + a_rd_err_monotonic
        // ==========================================================
        $display("\n[TC14] Monotonicity — rafaga de 8 discards WR+RD");
        mono_ok = 1'b1;
        for (i = 0; i < 8; i = i + 1) begin
            idle_inputs;
            wr_req_pndng  = 1'b1;
            wr_addr_valid = 1'b0;
            rd_req_pndng  = 1'b1;
            rd_addr_valid = 1'b0;
            @(posedge clk); #1;
            prev_wr_err = wr_err_cnt;
            prev_rd_err = rd_err_cnt;
            @(posedge clk); #1;
            if (wr_err_cnt < prev_wr_err) begin
                mono_ok = 1'b0;
                $display("  [FAIL] TC14 iter %0d: wr_err_cnt decrecio", i);
            end
            if (rd_err_cnt < prev_rd_err) begin
                mono_ok = 1'b0;
                $display("  [FAIL] TC14 iter %0d: rd_err_cnt decrecio", i);
            end
        end
        chk(mono_ok, 14, "wr_err_cnt y rd_err_cnt monotonicos en 8 ciclos");
        $display("         wr_err_cnt acumulado = %0d | rd_err_cnt acumulado = %0d",
                 wr_err_cnt, rd_err_cnt);

        // ==========================================================
        // TC15 — WRITE-FIRST: conflicto banco 3
        // SVA: a_write_first con banco de mayor índice
        // ==========================================================
        $display("\n[TC15] WRITE-FIRST — conflicto banco 3");
        idle_inputs;
        wr_req_pndng      = 1'b1;
        wr_addr_valid     = 1'b1;
        wr_bank_id        = 2'd3;
        wr_bank_word_addr = 8'hFF;
        wr_req_data       = {1'b1, 32'hC, 32'hBEEF_CAFE, 4'hF};
        bank_busy[3]      = 1'b0;
        rd_req_pndng      = 1'b1;
        rd_addr_valid     = 1'b1;
        rd_bank_id        = 2'd3;
        rd_bank_word_addr = 8'hFF;
        rob_tag_free      = 1'b1;
        @(posedge clk); #1;
        chk(wr_req_pop == 1'b1,           15, "wr_req_pop = 1 (WR gana banco 3)");
        chk(rd_req_pop == 1'b0,           15, "rd_req_pop = 0 (RD pospuesto banco 3)");
        chk(bank_req_valid[3] == 1'b1,    15, "banco 3 activo WR");
        chk(bank_req_op[3]    == 1'b1,    15, "op = WR en banco 3");

        // ==========================================================
        // TC16 — WR libre + RD banco ocupado → solo WR
        // ==========================================================
        $display("\n[TC16] WR libre + RD banco ocupado — solo WR");
        idle_inputs;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b1;
        wr_bank_id    = 2'd0;
        bank_busy[0]  = 1'b0;
        rd_req_pndng  = 1'b1;
        rd_addr_valid = 1'b1;
        rd_bank_id    = 2'd2;
        bank_busy[2]  = 1'b1;
        rob_tag_free  = 1'b1;
        @(posedge clk); #1;
        chk(wr_req_pop == 1'b1,           16, "wr_req_pop = 1 (WR libre)");
        chk(rd_req_pop == 1'b0,           16, "rd_req_pop = 0 (RD banco busy)");
        chk(bank_req_valid[0] == 1'b1,    16, "banco 0 activo (WR)");
        chk(bank_req_valid[2] == 1'b0,    16, "banco 2 inactivo (busy)");

        // ==========================================================
        // TC17 — WR banco ocupado + RD libre → solo RD
        // ==========================================================
        $display("\n[TC17] WR banco ocupado + RD libre — solo RD");
        idle_inputs;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b1;
        wr_bank_id    = 2'd1;
        bank_busy[1]  = 1'b1;
        rd_req_pndng  = 1'b1;
        rd_addr_valid = 1'b1;
        rd_bank_id    = 2'd3;
        bank_busy[3]  = 1'b0;
        rob_tag_free  = 1'b1;
        @(posedge clk); #1;
        chk(wr_req_pop == 1'b0,           17, "wr_req_pop = 0 (WR banco busy)");
        chk(rd_req_pop == 1'b1,           17, "rd_req_pop = 1 (RD libre)");
        chk(bank_req_valid[3] == 1'b1,    17, "banco 3 activo (RD)");
        chk(bank_req_op[3]    == 1'b0,    17, "op = RD en banco 3");
        chk(bank_req_valid[1] == 1'b0,    17, "banco 1 inactivo (busy)");

        // ==========================================================
        // TC18 — Todos los bancos ocupados
        // ==========================================================
        $display("\n[TC18] Todos los bancos ocupados — ningun despacho");
        idle_inputs;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b1;
        wr_bank_id    = 2'd0;
        rd_req_pndng  = 1'b1;
        rd_addr_valid = 1'b1;
        rd_bank_id    = 2'd1;
        rob_tag_free  = 1'b1;
        all_banks_busy;
        @(posedge clk); #1;
        chk(wr_req_pop == 1'b0,           18, "wr_req_pop = 0 (todos busy)");
        chk(rd_req_pop == 1'b0,           18, "rd_req_pop = 0 (todos busy)");
        chk(brv == 4'b0000,               18, "ningun banco activo");

        // ==========================================================
        // TC19 — Secuencia WR bancos 0→1→2→3
        // SVA: a_dispatch_wr_coherent para todos los bancos
        // ==========================================================
        $display("\n[TC19] Secuencia WR bancos 0-1-2-3");
        begin
            integer b;
            reg     seq_ok;
            seq_ok = 1'b1;
            for (b = 0; b < N_BANKS; b = b + 1) begin
                idle_inputs;
                wr_req_pndng      = 1'b1;
                wr_addr_valid     = 1'b1;
                wr_bank_id        = b[BANK_BITS-1:0];
                wr_bank_word_addr = 8'(b * 8);
                wr_req_data       = {1'b1, 32'(b), 32'hDEAD_0000 | 32'(b), 4'hF};
                bank_busy[b]      = 1'b0;
                @(posedge clk); #1;
                if (!wr_req_pop || !bank_req_valid[b] || !bank_req_op[b]) begin
                    seq_ok = 1'b0;
                    $display("  [FAIL] TC19 banco %0d: pop=%b valid=%b op=%b",
                             b, wr_req_pop, bank_req_valid[b], bank_req_op[b]);
                end else begin
                    $display("  [PASS] TC19 banco %0d: WR despacho correcto", b);
                end
            end
            chk(seq_ok, 19, "Secuencia WR 0-3 completada sin errores");
        end

        // ==========================================================
        // TC20 — Secuencia RD bancos 0→1→2→3
        // SVA: a_dispatch_rd_coherent para todos los bancos
        // ==========================================================
        $display("\n[TC20] Secuencia RD bancos 0-1-2-3");
        begin
            integer b;
            reg     seq_ok;
            seq_ok = 1'b1;
            for (b = 0; b < N_BANKS; b = b + 1) begin
                idle_inputs;
                rd_req_pndng      = 1'b1;
                rd_addr_valid     = 1'b1;
                rd_bank_id        = b[BANK_BITS-1:0];
                rd_bank_word_addr = 8'(b * 4);
                bank_busy[b]      = 1'b0;
                rob_tag_free      = 1'b1;
                @(posedge clk); #1;
                if (!rd_req_pop || !bank_req_valid[b] || bank_req_op[b]) begin
                    seq_ok = 1'b0;
                    $display("  [FAIL] TC20 banco %0d: pop=%b valid=%b op=%b",
                             b, rd_req_pop, bank_req_valid[b], bank_req_op[b]);
                end else begin
                    $display("  [PASS] TC20 banco %0d: RD despacho correcto", b);
                end
            end
            chk(seq_ok, 20, "Secuencia RD 0-3 completada sin errores");
        end

        // ==========================================================
        // TC21 — wr_resp_full=1 congela WR path (Versión A)
        // SVA: a_no_wr_activity_when_resp_full
        // ==========================================================
        $display("\n[TC21] wr_resp_full=1 — WR path congelado");
        idle_inputs;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b1;
        wr_bank_id    = 2'd0;
        bank_busy[0]  = 1'b0;
        wr_resp_full  = 1'b1;
        @(posedge clk); #1;
        chk(wr_req_pop == 1'b0,        21, "wr_req_pop = 0 (WR congelado por resp_full)");
        chk(brv        == 4'b0000,     21, "ningun banco activo con resp_full=1");

        // ==========================================================
        // TC22 — wr_resp_full=1 + addr invalida: sin discard
        // SVA: a_no_wr_activity_when_resp_full (cubre wr_discard)
        // ==========================================================
        $display("\n[TC22] wr_resp_full=1 + addr invalida — sin discard");
        idle_inputs;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b0;
        wr_bank_id    = 2'd1;
        wr_resp_full  = 1'b1;
        @(posedge clk); #1;
        prev_wr_err = wr_err_cnt;
        chk(wr_req_pop == 1'b0,        22, "wr_req_pop = 0 (discard bloqueado por resp_full)");
        @(posedge clk); #1;
        chk(wr_err_cnt == prev_wr_err, 22, "wr_err_cnt no incrementa con resp_full=1");

        // ==========================================================
        // TC23 — wr_resp_full baja: WR path se desbloquea
        // ==========================================================
        $display("\n[TC23] wr_resp_full baja — WR path se desbloquea");
        idle_inputs;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b1;
        wr_bank_id    = 2'd0;
        bank_busy[0]  = 1'b0;
        wr_resp_full  = 1'b0;
        @(posedge clk); #1;
        chk(wr_req_pop == 1'b1,           23, "wr_req_pop = 1 tras liberar resp_full");
        chk(bank_req_valid[0] == 1'b1,    23, "banco 0 activo tras liberar resp_full");
        chk(bank_req_op[0]    == 1'b1,    23, "op = WR tras liberar resp_full");

        // ============================================================
        // RESUMEN FINAL
        // ============================================================
        @(posedge clk); #1;
        $display("\n=====================================================");
        $display("  RESUMEN FINAL — TB_SCHEDULER_SVA");
        $display("  Checks TB    : PASS = %0d | FAIL = %0d | TOTAL = %0d",
                 pass_cnt, fail_cnt, pass_cnt + fail_cnt);
        $display("  wr_err_cnt acumulado = %0d", wr_err_cnt);
        $display("  rd_err_cnt acumulado = %0d", rd_err_cnt);
        $display("  (Las aserciones SVA del bind se reportan por separado)");
        if (fail_cnt == 0)
            $display("  RESULTADO TB : *** TODOS LOS CHECKS PASARON ***");
        else
            $display("  RESULTADO TB : *** %0d CHECKS FALLARON — VER ARRIBA ***", fail_cnt);
        $display("=====================================================");
        $finish;
    end

    // ── Guardia de timeout ────────────────────────────────
    initial begin
        #100_000;
        $display("[TIMEOUT] Simulacion excedio el limite de tiempo");
        $finish;
    end

endmodule
