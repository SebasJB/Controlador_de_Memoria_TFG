// ============================================================
//  File    : tb_scheduler_sva.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : Testbench para Scheduler + SVA bind
//
//  Propósito:
//    Estimular el módulo scheduler con 20 casos de prueba que
//    ejercen colectivamente las 12 aserciones de scheduler_sva.sv.
//    El bind de scheduler_sva se activa automáticamente al
//    instanciar scheduler; no se necesita instanciarlo aquí.
//
//  Compilar en VCS:
//    vcs -sverilog +lint=all \
//        scheduler.sv scheduler_sva.sv tb_scheduler_sva.sv \
//        -o simv && ./simv
//
//  Parámetros del sistema (deben coincidir con el DUT):
//    N_BANKS=4  ADDR_W=32  AXI_DATA_WIDTH=32  BANK_SIZE_BYTES=1024
//
//  Convenciones de tiempo:
//    – Estímulos se aplican entre flancos positivos del reloj.
//    – #1 tras aplicar entradas permite que la lógica
//      combinacional se propague antes de verificar.
//    – @(posedge clk) + #1 avanza al siguiente ciclo para
//      verificar registros (error counters, etc.).
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
    localparam CLK_HALF        = 5;    // 5 ns → período 10 ns

    // ── Parámetros derivados ───────────────────────────────
    localparam BANK_BITS       = 2;    // $clog2(4)
    localparam BANK_ADDR_WIDTH = 8;    // $clog2(1024/(32/8)) = $clog2(256)
    // Bus WR: {op[0], addr[31:0], data[31:0], strb[3:0]} = 69 bits → [68:0]
    localparam WR_BUS_MSB      = ADDR_W + AXI_DATA_WIDTH + AXI_DATA_WIDTH/8; // 68

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
    reg [N_BANKS-1:0]         bank_busy;
    reg                       rob_tag_free;
    reg                       wr_resp_full;

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

    // ── Vector packed de bank_req_valid (para $onehot0 TB) ─
    wire [N_BANKS-1:0] brv;
    assign brv[0] = bank_req_valid[0];
    assign brv[1] = bank_req_valid[1];
    assign brv[2] = bank_req_valid[2];
    assign brv[3] = bank_req_valid[3];

    // ── Contadores globales ────────────────────────────────
    integer pass_cnt;
    integer fail_cnt;

    // ── Instancia DUT ─────────────────────────────────────
    //    El bind de scheduler_sva se activa automáticamente.
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
        .bank_busy        (bank_busy),
        .rob_tag_free     (rob_tag_free),
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
    // TASK: chk — verifica una condición y actualiza contadores
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
    // TASK: all_banks_idle — pone todos los bank_busy a 0
    // ============================================================
    task all_banks_idle;
        begin
            bank_busy = {N_BANKS{1'b0}};
        end
    endtask

    // ============================================================
    // TASK: all_banks_busy — pone todos los bank_busy a 1
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
            #1;   // permite que bank_busy[] (array unpacked) propague
        end
    endtask

    // ============================================================
    // TASK: apply_reset — reset síncrono de 2 ciclos
    // ============================================================
    task apply_reset;
        integer k;
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
    // MAIN — Secuencia de 20 casos de prueba
    // ============================================================
    integer       i;
    reg [ERR_CNT_W-1:0] prev_wr_err;
    reg [ERR_CNT_W-1:0] prev_rd_err;
    reg           mono_ok;

    initial begin
        pass_cnt = 0;
        fail_cnt = 0;

        $display("=====================================================");
        $display("  TB_SCHEDULER_SVA — Inicio de simulación");
        $display("=====================================================");

        apply_reset;

        // ==========================================================
        // TC01 — Estado post-reset
        //  Verifica que todos los outputs comienzan en 0 tras rst_n.
        //  SVA cubierta: precondición de todas las aserciones.
        // ==========================================================
        $display("\n[TC01] Estado post-reset");
        idle_inputs;
        #1;
        chk(!wr_req_pop,          1, "wr_req_pop = 0 tras reset");
        chk(!rd_req_pop,          1, "rd_req_pop = 0 tras reset");
        chk(brv == 4'b0000,       1, "ningun banco activo tras reset");
        chk(wr_err_cnt == 16'd0,  1, "wr_err_cnt = 0 tras reset");
        chk(rd_err_cnt == 16'd0,  1, "rd_err_cnt = 0 tras reset");
        @(posedge clk); #1;

        // ==========================================================
        // TC02 — WR-only, banco 0
        //  Solo WR pendiente, dirección válida, banco libre.
        //  SVA: a_grant_wr_valid, a_dispatch_wr_coherent,
        //       a_wr_pop_reason, a_bank_req_valid_onehot
        // ==========================================================
        $display("\n[TC02] WR-only dispatch — banco 0");
        idle_inputs;
        wr_req_pndng      = 1'b1;
        wr_addr_valid     = 1'b1;
        wr_bank_id        = 2'd0;
        wr_bank_word_addr = 8'hAA;
        wr_req_data       = {1'b1, 32'h0000_0000, 32'hDEAD_BEEF, 4'hF};
        bank_busy[0]      = 1'b0;
        #1;
        chk(wr_req_pop == 1'b1,           2, "wr_req_pop = 1 (WR grant)");
        chk(rd_req_pop == 1'b0,           2, "rd_req_pop = 0 (sin RD)");
        chk(bank_req_valid[0] == 1'b1,    2, "banco 0 activo");
        chk(bank_req_op[0]    == 1'b1,    2, "op = WR en banco 0");
        chk(brv == 4'b0001,               2, "one-hot: solo banco 0");
        @(posedge clk); #1;

        // ==========================================================
        // TC03 — RD-only, banco 2
        //  Solo RD pendiente, dirección válida, banco libre, ROB free.
        //  SVA: a_grant_rd_valid, a_dispatch_rd_coherent,
        //       a_rd_pop_reason, a_bank_req_valid_onehot
        // ==========================================================
        $display("\n[TC03] RD-only dispatch — banco 2");
        idle_inputs;
        rd_req_pndng      = 1'b1;
        rd_addr_valid     = 1'b1;
        rd_bank_id        = 2'd2;
        rd_bank_word_addr = 8'h55;
        bank_busy[2]      = 1'b0;
        rob_tag_free      = 1'b1;
        #1;
        chk(rd_req_pop == 1'b1,           3, "rd_req_pop = 1 (RD grant)");
        chk(wr_req_pop == 1'b0,           3, "wr_req_pop = 0 (sin WR)");
        chk(bank_req_valid[2] == 1'b1,    3, "banco 2 activo");
        chk(bank_req_op[2]    == 1'b0,    3, "op = RD en banco 2");
        chk(brv == 4'b0100,               3, "one-hot: solo banco 2");
        @(posedge clk); #1;

        // ==========================================================
        // TC04 — WR banco 1 + RD banco 3 (bancos distintos)
        //  Ambos candidatos válidos, bancos distintos → ambos grants.
        //  Resultado esperado: brv = 4'b1010 (bancos 1 y 3 activos).
        //  SVA: both_no_conflict path, a_bank_req_valid_onehot
        //       (la aserción admite $onehot0 → 2-hot sigue siendo ≤2)
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
        #1;
        chk(wr_req_pop == 1'b1,           4, "wr_req_pop = 1 (WR grant dual)");
        chk(rd_req_pop == 1'b1,           4, "rd_req_pop = 1 (RD grant dual)");
        chk(bank_req_valid[1] == 1'b1,    4, "banco 1 activo (WR)");
        chk(bank_req_valid[3] == 1'b1,    4, "banco 3 activo (RD)");
        chk(bank_req_op[1]    == 1'b1,    4, "op = WR en banco 1");
        chk(bank_req_op[3]    == 1'b0,    4, "op = RD en banco 3");
        chk(bank_req_valid[0] == 1'b0,    4, "banco 0 inactivo");
        chk(bank_req_valid[2] == 1'b0,    4, "banco 2 inactivo");
        chk(brv == 4'b1010,               4, "exactamente bancos 1 y 3 activos");
        @(posedge clk); #1;

        // ==========================================================
        // TC05 — WRITE-FIRST: conflicto banco 0
        //  WR y RD apuntan al mismo banco 0, ambos válidos y libres.
        //  Política WRITE-FIRST: solo WR gana, RD se pospone.
        //  SVA: a_write_first (CRÍTICA para política de arbitraje)
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
        rd_bank_id        = 2'd0;   // mismo banco → conflicto
        rd_bank_word_addr = 8'h01;
        rob_tag_free      = 1'b1;
        #1;
        chk(wr_req_pop == 1'b1,           5, "wr_req_pop = 1 (WR gana conflicto)");
        chk(rd_req_pop == 1'b0,           5, "rd_req_pop = 0 (RD pospuesto)");
        chk(bank_req_valid[0] == 1'b1,    5, "banco 0 activo con WR");
        chk(bank_req_op[0]    == 1'b1,    5, "op = WR en conflicto");
        chk(brv == 4'b0001,               5, "one-hot: solo banco 0 en conflicto");
        @(posedge clk); #1;

        // ==========================================================
        // TC06 — WR con dirección inválida → wr_discard
        //  Pendiente WR pero addr_valid=0 → discard sin grant.
        //  El pop ocurre por discard (no por grant).
        //  SVA: a_wr_ok_discard_mutex, a_wr_pop_reason,
        //       a_wr_err_monotonic (registro secuencial)
        // ==========================================================
        $display("\n[TC06] WR addr invalida — discard");
        idle_inputs;
        prev_wr_err   = wr_err_cnt;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b0;   // invalida → discard
        wr_bank_id    = 2'd1;
        #1;
        chk(wr_req_pop == 1'b1,           6, "wr_req_pop = 1 (pop por discard)");
        chk(rd_req_pop == 1'b0,           6, "rd_req_pop = 0 (sin RD)");
        chk(brv == 4'b0000,               6, "ningun banco activo en discard WR");
        @(posedge clk); #1;
        chk(wr_err_cnt == prev_wr_err + 1, 6, "wr_err_cnt incremento en 1");

        // ==========================================================
        // TC07 — RD con dirección inválida → rd_discard
        //  Análogo a TC06 para el canal de lectura.
        //  SVA: a_rd_ok_discard_mutex, a_rd_pop_reason,
        //       a_rd_err_monotonic
        // ==========================================================
        $display("\n[TC07] RD addr invalida — discard");
        idle_inputs;
        prev_rd_err   = rd_err_cnt;
        rd_req_pndng  = 1'b1;
        rd_addr_valid = 1'b0;   // invalida → discard
        rd_bank_id    = 2'd2;
        rob_tag_free  = 1'b1;
        #1;
        chk(rd_req_pop == 1'b1,           7, "rd_req_pop = 1 (pop por discard)");
        chk(wr_req_pop == 1'b0,           7, "wr_req_pop = 0 (sin WR)");
        chk(brv == 4'b0000,               7, "ningun banco activo en discard RD");
        @(posedge clk); #1;
        chk(rd_err_cnt == prev_rd_err + 1, 7, "rd_err_cnt incremento en 1");

        // ==========================================================
        // TC08 — WR banco ocupado → sin grant ni pop
        //  bank_busy bloquea al candidato WR.
        //  SVA: a_grant_wr_valid (precondición wr_bank_free)
        // ==========================================================
        $display("\n[TC08] WR banco ocupado — sin despacho");
        idle_inputs;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b1;
        wr_bank_id    = 2'd2;
        bank_busy[2]  = 1'b1;   // banco ocupado
        #1;
        chk(wr_req_pop == 1'b0,           8, "wr_req_pop = 0 (banco busy)");
        chk(brv == 4'b0000,               8, "ningun banco activo con banco busy");
        @(posedge clk); #1;

        // ==========================================================
        // TC09 — RD sin rob_tag_free → sin grant ni pop
        //  ROB lleno bloquea a rd_candidate.
        //  SVA: a_grant_rd_valid (precondición rob_tag_free)
        // ==========================================================
        $display("\n[TC09] RD con ROB lleno — sin despacho");
        idle_inputs;
        rd_req_pndng  = 1'b1;
        rd_addr_valid = 1'b1;
        rd_bank_id    = 2'd1;
        bank_busy[1]  = 1'b0;
        rob_tag_free  = 1'b0;   // ROB lleno
        #1;
        chk(rd_req_pop == 1'b0,           9, "rd_req_pop = 0 (ROB lleno)");
        chk(brv == 4'b0000,               9, "ningun banco activo con ROB lleno");
        @(posedge clk); #1;

        // ==========================================================
        // TC10 — WR+RD mismo banco, banco ocupado → ninguno
        //  Conflicto + busy: ni wr_candidate ni rd_candidate válidos.
        //  SVA: ambas precondiciones fallan a la vez
        // ==========================================================
        $display("\n[TC10] WR+RD mismo banco ocupado — ninguno despacha");
        idle_inputs;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b1;
        wr_bank_id    = 2'd3;
        bank_busy[3]  = 1'b1;
        rd_req_pndng  = 1'b1;
        rd_addr_valid = 1'b1;
        rd_bank_id    = 2'd3;   // mismo banco
        rob_tag_free  = 1'b1;
        #1;
        chk(wr_req_pop == 1'b0,           10, "wr_req_pop = 0 (ambos busy)");
        chk(rd_req_pop == 1'b0,           10, "rd_req_pop = 0 (ambos busy)");
        chk(brv == 4'b0000,               10, "ningun banco activo");
        @(posedge clk); #1;

        // ==========================================================
        // TC11 — Ningún request pendiente → todo inactivo
        //  Baseline idle: sin requests, sin pops, sin bancos activos.
        // ==========================================================
        $display("\n[TC11] Sin requests pendientes — idle total");
        idle_inputs;
        wr_req_pndng = 1'b0;
        rd_req_pndng = 1'b0;
        #1;
        chk(wr_req_pop == 1'b0,           11, "wr_req_pop = 0 en idle");
        chk(rd_req_pop == 1'b0,           11, "rd_req_pop = 0 en idle");
        chk(brv == 4'b0000,               11, "ningun banco activo en idle");
        @(posedge clk); #1;

        // ==========================================================
        // TC12 — WR discard simultáneo con RD grant
        //  WR addr inválida (discard) al mismo tiempo que RD válida.
        //  Ambos pops ocurren pero por causas distintas.
        //  SVA: a_wr_pop_reason(discard) + a_rd_pop_reason(grant)
        // ==========================================================
        $display("\n[TC12] WR discard + RD grant simultaneos");
        idle_inputs;
        prev_wr_err   = wr_err_cnt;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b0;   // discard
        wr_bank_id    = 2'd0;
        rd_req_pndng  = 1'b1;
        rd_addr_valid = 1'b1;
        rd_bank_id    = 2'd1;
        bank_busy[1]  = 1'b0;
        rob_tag_free  = 1'b1;
        #1;
        chk(wr_req_pop == 1'b1,           12, "wr_req_pop = 1 (discard)");
        chk(rd_req_pop == 1'b1,           12, "rd_req_pop = 1 (grant RD)");
        chk(bank_req_valid[1] == 1'b1,    12, "banco 1 activo (RD grant)");
        chk(bank_req_op[1]    == 1'b0,    12, "op = RD en banco 1");
        chk(bank_req_valid[0] == 1'b0,    12, "banco 0 inactivo (WR discard)");
        @(posedge clk); #1;
        chk(wr_err_cnt == prev_wr_err + 1, 12, "wr_err_cnt incremento (discard sim.)");

        // ==========================================================
        // TC13 — WR grant simultáneo con RD discard
        //  WR válida (banco libre) al mismo tiempo que RD addr inválida.
        //  SVA: a_wr_pop_reason(grant) + a_rd_pop_reason(discard)
        // ==========================================================
        $display("\n[TC13] WR grant + RD discard simultaneos");
        idle_inputs;
        prev_rd_err   = rd_err_cnt;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b1;
        wr_bank_id    = 2'd2;
        wr_bank_word_addr = 8'h30;
        wr_req_data   = {1'b1, 32'h8, 32'h1234_5678, 4'hA};
        bank_busy[2]  = 1'b0;
        rd_req_pndng  = 1'b1;
        rd_addr_valid = 1'b0;   // discard
        rd_bank_id    = 2'd3;
        rob_tag_free  = 1'b1;
        #1;
        chk(wr_req_pop == 1'b1,           13, "wr_req_pop = 1 (WR grant)");
        chk(rd_req_pop == 1'b1,           13, "rd_req_pop = 1 (RD discard)");
        chk(bank_req_valid[2] == 1'b1,    13, "banco 2 activo (WR)");
        chk(bank_req_op[2]    == 1'b1,    13, "op = WR en banco 2");
        chk(bank_req_valid[3] == 1'b0,    13, "banco 3 inactivo (RD discard)");
        @(posedge clk); #1;
        chk(rd_err_cnt == prev_rd_err + 1, 13, "rd_err_cnt incremento (discard sim.)");

        // ==========================================================
        // TC14 — Monotonicidad: ráfaga de 8 discards WR+RD
        //  Ambos contadores deben crecer monotónicamente cada ciclo.
        //  SVA: a_wr_err_monotonic + a_rd_err_monotonic en continuo
        // ==========================================================
        $display("\n[TC14] Monotonicity — rafaga de 8 discards WR+RD");
        mono_ok = 1'b1;
        for (i = 0; i < 8; i = i + 1) begin
            idle_inputs;
            prev_wr_err   = wr_err_cnt;
            prev_rd_err   = rd_err_cnt;
            wr_req_pndng  = 1'b1;
            wr_addr_valid = 1'b0;
            rd_req_pndng  = 1'b1;
            rd_addr_valid = 1'b0;
            @(posedge clk); #1;
            if (wr_err_cnt < prev_wr_err) begin
                mono_ok = 1'b0;
                $display("  [FAIL] TC14 iter %0d: wr_err_cnt decreció %0d→%0d", i, prev_wr_err, wr_err_cnt);
            end
            if (rd_err_cnt < prev_rd_err) begin
                mono_ok = 1'b0;
                $display("  [FAIL] TC14 iter %0d: rd_err_cnt decreció %0d→%0d", i, prev_rd_err, rd_err_cnt);
            end
        end
        chk(mono_ok, 14, "wr_err_cnt y rd_err_cnt monotonicos en 8 ciclos");
        $display("         wr_err_cnt acumulado = %0d | rd_err_cnt acumulado = %0d",
                 wr_err_cnt, rd_err_cnt);

        // ==========================================================
        // TC15 — WRITE-FIRST: conflicto banco 3 (variante banco != 0)
        //  Mismo escenario que TC05 pero en banco 3 para cobertura.
        //  SVA: a_write_first con banco de mayor índice
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
        rd_bank_id        = 2'd3;   // mismo banco → conflicto
        rd_bank_word_addr = 8'hFF;
        rob_tag_free      = 1'b1;
        #1;
        chk(wr_req_pop == 1'b1,           15, "wr_req_pop = 1 (WR gana banco 3)");
        chk(rd_req_pop == 1'b0,           15, "rd_req_pop = 0 (RD pospuesto banco 3)");
        chk(bank_req_valid[3] == 1'b1,    15, "banco 3 activo WR");
        chk(bank_req_op[3]    == 1'b1,    15, "op = WR en banco 3");
        @(posedge clk); #1;

        // ==========================================================
        // TC16 — WR libre + RD banco ocupado → solo WR despacha
        //  RD candidate falla por bank_busy; WR despacha solo.
        //  SVA: a_grant_rd_valid desde perspectiva negativa
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
        bank_busy[2]  = 1'b1;   // banco RD ocupado
        rob_tag_free  = 1'b1;
        #1;
        chk(wr_req_pop == 1'b1,           16, "wr_req_pop = 1 (WR libre)");
        chk(rd_req_pop == 1'b0,           16, "rd_req_pop = 0 (RD banco busy)");
        chk(bank_req_valid[0] == 1'b1,    16, "banco 0 activo (WR)");
        chk(bank_req_valid[2] == 1'b0,    16, "banco 2 inactivo (busy)");
        @(posedge clk); #1;

        // ==========================================================
        // TC17 — WR banco ocupado + RD libre → solo RD despacha
        //  WR candidate falla por bank_busy; RD despacha solo.
        //  SVA: a_grant_wr_valid desde perspectiva negativa
        // ==========================================================
        $display("\n[TC17] WR banco ocupado + RD libre — solo RD");
        idle_inputs;
        wr_req_pndng  = 1'b1;
        wr_addr_valid = 1'b1;
        wr_bank_id    = 2'd1;
        bank_busy[1]  = 1'b1;   // banco WR ocupado
        rd_req_pndng  = 1'b1;
        rd_addr_valid = 1'b1;
        rd_bank_id    = 2'd3;
        bank_busy[3]  = 1'b0;
        rob_tag_free  = 1'b1;
        #1;
        chk(wr_req_pop == 1'b0,           17, "wr_req_pop = 0 (WR banco busy)");
        chk(rd_req_pop == 1'b1,           17, "rd_req_pop = 1 (RD libre)");
        chk(bank_req_valid[3] == 1'b1,    17, "banco 3 activo (RD)");
        chk(bank_req_op[3]    == 1'b0,    17, "op = RD en banco 3");
        chk(bank_req_valid[1] == 1'b0,    17, "banco 1 inactivo (busy)");
        @(posedge clk); #1;

        // ==========================================================
        // TC18 — Todos los bancos ocupados
        //  Sin grants posibles aunque ambos requests sean válidos.
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
        #1;
        chk(wr_req_pop == 1'b0,           18, "wr_req_pop = 0 (todos busy)");
        chk(rd_req_pop == 1'b0,           18, "rd_req_pop = 0 (todos busy)");
        chk(brv == 4'b0000,               18, "ningun banco activo");
        @(posedge clk); #1;

        // ==========================================================
        // TC19 — Secuencia de 4 WR, uno por banco (0→1→2→3)
        //  Verifica que el dispatch llega correctamente a cada banco.
        //  SVA: a_dispatch_wr_coherent para todos los bancos
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
                #1;
                if (!wr_req_pop || !bank_req_valid[b] || !bank_req_op[b]) begin
                    seq_ok = 1'b0;
                    $display("  [FAIL] TC19 banco %0d: pop=%b valid=%b op=%b",
                             b, wr_req_pop, bank_req_valid[b], bank_req_op[b]);
                end else begin
                    $display("  [PASS] TC19 banco %0d: WR despacho correcto", b);
                end
                @(posedge clk); #1;
            end
            chk(seq_ok, 19, "Secuencia WR 0-3 completada sin errores");
        end

        // ==========================================================
        // TC20 — Secuencia de 4 RD, uno por banco (0→1→2→3)
        //  Verifica dispatch correcto de lecturas a cada banco.
        //  SVA: a_dispatch_rd_coherent para todos los bancos
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
                #1;
                if (!rd_req_pop || !bank_req_valid[b] || bank_req_op[b]) begin
                    seq_ok = 1'b0;
                    $display("  [FAIL] TC20 banco %0d: pop=%b valid=%b op=%b",
                             b, rd_req_pop, bank_req_valid[b], bank_req_op[b]);
                end else begin
                    $display("  [PASS] TC20 banco %0d: RD despacho correcto", b);
                end
                @(posedge clk); #1;
            end
            chk(seq_ok, 20, "Secuencia RD 0-3 completada sin errores");
        end

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

    // ── Guardia de timeout — evita simulacion infinita ────
    initial begin
        #100_000;
        $display("[TIMEOUT] Simulacion excedio el limite de tiempo seguro");
        $finish;
    end

endmodule
