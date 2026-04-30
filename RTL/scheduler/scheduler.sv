// ============================================================
//  File    : scheduler.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : Scheduler (segundo nivel)
//
//  Módulos:
//    1.  wr_req_valid_check     — comb
//    2.  rd_req_valid_check     — comb
//    3.  bank_conflict_check    — comb
//    4.  bank_availability_check— comb
//    5.  priority_resolver      — comb
//    6.  fifo_pop_control       — comb
//    7.  error_counter          — reg
//    8.  unified_dispatch_logic — comb (per bank)
//    9.  read_tag_generator     — reg
//    10. wdata_wstrb_broadcast  — comb
//    11. scheduler              — top
//
//  Clocking : flanco positivo (clk)
//  Reset    : síncrono activo en bajo (rst_n)
// ============================================================

`timescale 1ns/1ps


// ============================================================
// 1. WR_REQ_VALID_CHECK
//    ok      = pndng && valid
//    discard = pndng && !valid  → pop && err_cnt++
// ============================================================
module wr_req_valid_check (
    input  wire wr_req_pndng,
    input  wire wr_addr_valid,
    input  wire wr_resp_full,
    output wire wr_ok,
    output wire wr_discard
);
    assign wr_ok      = wr_req_pndng & wr_addr_valid & !wr_resp_full;
    assign wr_discard = wr_req_pndng & !wr_addr_valid & !wr_resp_full;
endmodule


// ============================================================
// 2. RD_REQ_VALID_CHECK
//    ok      = pndng && valid
//    discard = pndng && !valid  → pop && err_cnt++
// ============================================================
module rd_req_valid_check (
    input  wire rd_req_pndng,
    input  wire rd_addr_valid,
    output wire rd_ok,
    output wire rd_discard
);
    assign rd_ok      = rd_req_pndng & rd_addr_valid;
    assign rd_discard = rd_req_pndng & !rd_addr_valid;
endmodule


// ============================================================
// 3. BANK_CONFLICT_CHECK
//    same_bank = (wr_bank_id == rd_bank_id)
// ============================================================
module bank_conflict_check #(
    parameter N_BANKS = 4
)(
    input  wire [$clog2(N_BANKS)-1:0] wr_bank_id,
    input  wire [$clog2(N_BANKS)-1:0] rd_bank_id,
    output wire                       same_bank
);
    assign same_bank = (wr_bank_id == rd_bank_id);
endmodule


// ============================================================
// 4. BANK_AVAILABILITY_CHECK
//    wr_bank_free = !bank_busy[wr_bank_id]
//    rd_bank_free = !bank_busy[rd_bank_id]
// ============================================================
module bank_availability_check #(
    parameter N_BANKS = 4
)(
    input  wire bank_busy [0:N_BANKS-1],
    input  wire [$clog2(N_BANKS)-1:0] wr_bank_id,
    input  wire [$clog2(N_BANKS)-1:0] rd_bank_id,
    output wire                       wr_bank_free,
    output wire                       rd_bank_free
);
    assign wr_bank_free = !bank_busy[wr_bank_id];
    assign rd_bank_free = !bank_busy[rd_bank_id];
endmodule


// ============================================================
// 5. PRIORITY_RESOLVER
//    Completamente combinacional. Política WRITE-FIRST.
//
//    wr_candidate = wr_ok && wr_bank_free
//    rd_candidate = rd_ok && rd_bank_free && rob_tag_free
//
//    Cuatro casos:
//      wr_only         = wr_candidate && !rd_candidate
//      rd_only         = !wr_candidate && rd_candidate
//      both_conflict   = wr_candidate && rd_candidate && same_bank
//      both_no_conflict= wr_candidate && rd_candidate && !same_bank
//
//    grant_wr = wr_only || both_no_conflict || both_conflict
//    grant_rd = rd_only || both_no_conflict
// ============================================================
module priority_resolver (
    input  wire wr_ok,
    input  wire rd_ok,
    input  wire wr_bank_free,
    input  wire rd_bank_free,
    input  wire rob_tag_free,
    input  wire same_bank,
    output wire grant_wr,
    output wire grant_rd
);
    wire wr_candidate;
    wire rd_candidate;
    assign wr_candidate = wr_ok & wr_bank_free;
    assign rd_candidate = rd_ok & rd_bank_free & rob_tag_free;

    wire wr_only;
    wire rd_only;
    wire both_conflict;
    wire both_no_conflict;
    assign wr_only          = wr_candidate & !rd_candidate;
    assign rd_only          = !wr_candidate & rd_candidate;
    assign both_conflict    = wr_candidate & rd_candidate & same_bank;
    assign both_no_conflict = wr_candidate & rd_candidate & !same_bank;

    assign grant_wr = wr_only | both_no_conflict | both_conflict;
    assign grant_rd = rd_only | both_no_conflict;
endmodule


// ============================================================
// 6. FIFO_POP_CONTROL
//    wr_pop = grant_wr || wr_discard
//    rd_pop = grant_rd || rd_discard
// ============================================================
module fifo_pop_control (
    input  wire grant_wr,
    input  wire grant_rd,
    input  wire wr_discard,
    input  wire rd_discard,
    output wire wr_req_pop,
    output wire rd_req_pop
);
    assign wr_req_pop = grant_wr | wr_discard;
    assign rd_req_pop = grant_rd | rd_discard;
endmodule


// ============================================================
// 7. ERROR_COUNTER
//    Cuenta eventos de discard (dirección inválida).
//    Registro de ERR_CNT_W bits. Métrica M5 del sistema.
// ============================================================
module error_counter #(
    parameter ERR_CNT_W = 16
)(
    input  wire clk,
    input  wire rst_n,
    input  wire wr_discard,
    input  wire rd_discard,
    output reg  [ERR_CNT_W-1:0] wr_err_cnt,
    output reg  [ERR_CNT_W-1:0] rd_err_cnt
);
    always @(posedge clk) begin
        if (!rst_n) begin
            wr_err_cnt <= {ERR_CNT_W{1'b0}};
            rd_err_cnt <= {ERR_CNT_W{1'b0}};
        end else begin
            if (wr_discard)
                wr_err_cnt <= wr_err_cnt + {{(ERR_CNT_W-1){1'b0}}, 1'b1};
            if (rd_discard)
                rd_err_cnt <= rd_err_cnt + {{(ERR_CNT_W-1){1'b0}}, 1'b1};
        end
    end
endmodule


// ============================================================
// 8. UNIFIED_DISPATCH_LOGIC
//    Completamente combinacional. Para cada banco i:
//
//    Valid Generator:
//      wr_hit[i] = grant_wr && (wr_bank_id == i)
//      rd_hit[i] = grant_rd && (rd_bank_id == i)
//      bank_req_valid[i] = wr_hit[i] || rd_hit[i]
//
//    Op MUX:
//      bank_req_op[i] = wr_hit[i] ? WR(1) : RD(0)
//
//    Addr MUX:
//      bank_req_addr[i] = wr_hit[i] ?
//          wr_bank_word_addr : rd_bank_word_addr
// ============================================================
module unified_dispatch_logic #(
    parameter N_BANKS         = 4,
    parameter BANK_ADDR_WIDTH = 8
)(
    input  wire                        grant_wr,
    input  wire                        grant_rd,
    input  wire [$clog2(N_BANKS)-1:0]  wr_bank_id,
    input  wire [$clog2(N_BANKS)-1:0]  rd_bank_id,
    input  wire [BANK_ADDR_WIDTH-1:0]  wr_bank_word_addr,
    input  wire [BANK_ADDR_WIDTH-1:0]  rd_bank_word_addr,

    output reg                      bank_req_valid [0:N_BANKS-1],
    output reg                      bank_req_op    [0:N_BANKS-1],
    output reg [BANK_ADDR_WIDTH-1:0] bank_req_addr [0:N_BANKS-1]
);
    localparam BANK_BITS = $clog2(N_BANKS);

    integer i;
    reg wr_hit [0:N_BANKS-1];
    reg rd_hit [0:N_BANKS-1];

    always_comb begin
        for (i = 0; i < N_BANKS; i = i + 1) begin
            bank_req_valid[i] = 1'b0;
            bank_req_op[i]    = 1'b0;
            bank_req_addr[i]  = {BANK_ADDR_WIDTH{1'b0}};
        end

        for (i = 0; i < N_BANKS; i = i + 1) begin
            // Valid Generator
            wr_hit[i] = grant_wr & (wr_bank_id == BANK_BITS'(i));
            rd_hit[i] = grant_rd & (rd_bank_id == BANK_BITS'(i));
            bank_req_valid[i] = wr_hit[i] | rd_hit[i];

            // Op MUX: WR = 1'b1, RD = 1'b0
            bank_req_op[i] = wr_hit[i] ? 1'b1 : 1'b0;

            // Addr MUX
            bank_req_addr[i] = wr_hit[i] ? wr_bank_word_addr : rd_bank_word_addr;
        end
    end
endmodule


// ============================================================
// 9. READ_TAG_GENERATOR
//    Contador circular de 0 a N_BANKS-1 con wrap bit.
//    Cuando cnt_addr == N_BANKS-1 y grant_rd:
//      cnt_addr se reinicia a 0
//      cnt_wrap se invierte (toggle)
//    En cualquier otro grant_rd: cnt <= cnt + 1.
//
//    Salidas:
//      bank_req_tag = cnt[BANK_BITS-1:0]   (tag slicer)
//      wr_ptr_ext   = cnt[BANK_BITS:0]     (wrap pointer → ROB)
// ============================================================
module read_tag_generator #(
    parameter N_BANKS = 4
)(
    input  wire clk,
    input  wire rst_n,
    input  wire grant_rd,

    output wire [$clog2(N_BANKS)-1:0] bank_req_tag,
    output wire [$clog2(N_BANKS):0]   wr_ptr_ext
);
    localparam BANK_BITS = $clog2(N_BANKS);
    localparam PTR_W     = BANK_BITS + 1;
    localparam [BANK_BITS-1:0] ADDR_MAX = N_BANKS - 1;

    reg [BANK_BITS-1:0] cnt_addr;
    reg cnt_wrap;

    always @(posedge clk) begin
        if (!rst_n) begin
            cnt_addr <= {BANK_BITS{1'b0}};
            cnt_wrap <= 1'b0;
        end else begin
            if (grant_rd) begin
                if (cnt_addr == ADDR_MAX) begin
                    // Reiniciar dirección a 0, invertir wrap bit
                    cnt_addr <= {BANK_BITS{1'b0}};
                    cnt_wrap <= !cnt_wrap;
                end else begin
                    cnt_addr <= cnt_addr + {{(BANK_BITS-1){1'b0}}, 1'b1};
                end
            end
        end
    end

    // Tag slicer
    assign bank_req_tag = cnt_addr;

    // Wrap pointer → ROB
    assign wr_ptr_ext = {cnt_wrap, cnt_addr};

endmodule


// ============================================================
// 10. WDATA_WSTRB_BROADCAST
//     Extrae wdata y wstrb del bus packed wr_req_data.
//     Bus: {op(1), addr(ADDR_W), data(DATA_W), strb(STRB_W)}
//     Broadcast a todos los bancos (el valid del dispatch
//     logic selecciona quién lo usa).
// ============================================================
module wdata_wstrb_broadcast #(
    parameter ADDR_W         = 32,
    parameter AXI_DATA_WIDTH = 32
)(
    input  wire [ADDR_W + AXI_DATA_WIDTH + AXI_DATA_WIDTH/8 : 0] wr_req_data,
    output wire [AXI_DATA_WIDTH-1:0]                              bank_req_wdata,
    output wire [AXI_DATA_WIDTH/8-1:0]                            bank_req_wstrb
);
    localparam STRB_W = AXI_DATA_WIDTH / 8;

    assign bank_req_wstrb = wr_req_data[STRB_W-1 : 0];
    assign bank_req_wdata = wr_req_data[AXI_DATA_WIDTH + STRB_W - 1 : STRB_W];
endmodule


// ============================================================
// 11. SCHEDULER — TOP
//     Instancia y conecta todos los submódulos.
//     Solo instanciación, sin lógica inline.
// ============================================================
module scheduler #(
    parameter ADDR_W          = 32,
    parameter AXI_DATA_WIDTH  = 32,
    parameter N_BANKS         = 4,
    parameter BANK_SIZE_BYTES = 1024,
    parameter ERR_CNT_W       = 16
)(
    input  wire clk,
    input  wire rst_n,

    // ── Desde Addr Decoders ──────────────────────────
    input  wire [$clog2(N_BANKS)-1:0]                             wr_bank_id,
    input  wire [$clog2(N_BANKS)-1:0]                             rd_bank_id,
    input  wire [$clog2(BANK_SIZE_BYTES/(AXI_DATA_WIDTH/8))-1:0]  wr_bank_word_addr,
    input  wire [$clog2(BANK_SIZE_BYTES/(AXI_DATA_WIDTH/8))-1:0]  rd_bank_word_addr,
    input  wire                                                   wr_addr_valid,
    input  wire                                                   rd_addr_valid,

    // ── Desde FIFOs (pending) ────────────────────────
    input  wire wr_req_pndng,
    input  wire rd_req_pndng,

    // ── Desde WR REQ FIFO (bus packed para broadcast) ─
    input  wire [ADDR_W + AXI_DATA_WIDTH + AXI_DATA_WIDTH/8 : 0] wr_req_data,

    // ── Desde Bank Controllers ───────────────────────
    input  wire bank_busy [0:N_BANKS-1],

    // ── Desde ROB ────────────────────────────────────
    input  wire rob_tag_free,

    // ── Desde WR Response Path (backpressure canal B) ──
    input  wire wr_resp_full,

    // ── Hacia WR/RD REQ FIFOs (pop) ─────────────────
    output wire wr_req_pop,
    output wire rd_req_pop,

    // ── Hacia Bank Controllers (dispatch) ────────────
    output wire                                                                  bank_req_valid [0:N_BANKS-1],
    output wire                                                                  bank_req_op    [0:N_BANKS-1],
    output wire [$clog2(BANK_SIZE_BYTES/(AXI_DATA_WIDTH/8))-1:0]                 bank_req_addr  [0:N_BANKS-1],
    output wire [AXI_DATA_WIDTH-1:0]                                             bank_req_wdata,
    output wire [AXI_DATA_WIDTH/8-1:0]                                           bank_req_wstrb,

    // ── Hacia ROB (tag + pointer) ────────────────────
    output wire [$clog2(N_BANKS)-1:0] bank_req_tag,
    output wire [$clog2(N_BANKS):0]   wr_ptr_ext,

    // ── Métricas M5 ─────────────────────────────────
    output wire [ERR_CNT_W-1:0] wr_err_cnt,
    output wire [ERR_CNT_W-1:0] rd_err_cnt
);

    localparam BANK_BITS       = $clog2(N_BANKS);
    localparam WORD_BYTES      = AXI_DATA_WIDTH / 8;
    localparam BANK_WORDS      = BANK_SIZE_BYTES / WORD_BYTES;
    localparam BANK_ADDR_WIDTH = $clog2(BANK_WORDS);

    // ── Señales internas ─────────────────────────────
    wire wr_ok, wr_discard;
    wire rd_ok, rd_discard;
    wire same_bank;
    wire wr_bank_free, rd_bank_free;
    wire grant_wr, grant_rd;

    // ── 1. WR Req Valid Check ────────────────────────
    wr_req_valid_check u_wr_req_valid_check (
        .wr_req_pndng (wr_req_pndng),
        .wr_addr_valid(wr_addr_valid),
        .wr_resp_full (wr_resp_full),
        .wr_ok        (wr_ok),
        .wr_discard   (wr_discard)
    );

    // ── 2. RD Req Valid Check ────────────────────────
    rd_req_valid_check u_rd_req_valid_check (
        .rd_req_pndng (rd_req_pndng),
        .rd_addr_valid(rd_addr_valid),
        .rd_ok        (rd_ok),
        .rd_discard   (rd_discard)
    );

    // ── 3. Bank Conflict Check ───────────────────────
    bank_conflict_check #(
        .N_BANKS(N_BANKS)
    ) u_bank_conflict_check (
        .wr_bank_id(wr_bank_id),
        .rd_bank_id(rd_bank_id),
        .same_bank (same_bank)
    );

    // ── 4. Bank Availability Check ───────────────────
    bank_availability_check #(
        .N_BANKS(N_BANKS)
    ) u_bank_availability_check (
        .bank_busy    (bank_busy),
        .wr_bank_id   (wr_bank_id),
        .rd_bank_id   (rd_bank_id),
        .wr_bank_free (wr_bank_free),
        .rd_bank_free (rd_bank_free)
    );

    // ── 5. Priority Resolver ─────────────────────────
    priority_resolver u_priority_resolver (
        .wr_ok        (wr_ok),
        .rd_ok        (rd_ok),
        .wr_bank_free (wr_bank_free),
        .rd_bank_free (rd_bank_free),
        .rob_tag_free (rob_tag_free),
        .same_bank    (same_bank),
        .grant_wr     (grant_wr),
        .grant_rd     (grant_rd)
    );

    // ── 6. FIFO Pop Control ──────────────────────────
    fifo_pop_control u_fifo_pop_control (
        .grant_wr   (grant_wr),
        .grant_rd   (grant_rd),
        .wr_discard (wr_discard),
        .rd_discard (rd_discard),
        .wr_req_pop (wr_req_pop),
        .rd_req_pop (rd_req_pop)
    );

    // ── 7. Error Counter ─────────────────────────────
    error_counter #(
        .ERR_CNT_W(ERR_CNT_W)
    ) u_error_counter (
        .clk       (clk),
        .rst_n     (rst_n),
        .wr_discard(wr_discard),
        .rd_discard(rd_discard),
        .wr_err_cnt(wr_err_cnt),
        .rd_err_cnt(rd_err_cnt)
    );

    // ── 8. Unified Dispatch Logic ────────────────────
    unified_dispatch_logic #(
        .N_BANKS        (N_BANKS),
        .BANK_ADDR_WIDTH(BANK_ADDR_WIDTH)
    ) u_unified_dispatch_logic (
        .grant_wr         (grant_wr),
        .grant_rd         (grant_rd),
        .wr_bank_id       (wr_bank_id),
        .rd_bank_id       (rd_bank_id),
        .wr_bank_word_addr(wr_bank_word_addr),
        .rd_bank_word_addr(rd_bank_word_addr),
        .bank_req_valid   (bank_req_valid),
        .bank_req_op      (bank_req_op),
        .bank_req_addr    (bank_req_addr)
    );

    // ── 9. Read Tag Generator ────────────────────────
    read_tag_generator #(
        .N_BANKS(N_BANKS)
    ) u_read_tag_generator (
        .clk         (clk),
        .rst_n       (rst_n),
        .grant_rd    (grant_rd),
        .bank_req_tag(bank_req_tag),
        .wr_ptr_ext  (wr_ptr_ext)
    );

    // ── 10. Wdata/Wstrb Broadcast ────────────────────
    wdata_wstrb_broadcast #(
        .ADDR_W        (ADDR_W),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) u_wdata_wstrb_broadcast (
        .wr_req_data   (wr_req_data),
        .bank_req_wdata(bank_req_wdata),
        .bank_req_wstrb(bank_req_wstrb)
    );

endmodule
