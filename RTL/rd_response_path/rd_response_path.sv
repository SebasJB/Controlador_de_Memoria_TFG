// ============================================================
//  File    : rd_response_path.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : Read Response Path — Canal R (completo)
//
//  Módulos:
//    1. rd_resp_mux         — OR-reduce + priority mux (comb)
//    2. reorder_buffer      — ROB punteros circulares (reg)
//    3. r_channel_logic     — AXI4-Lite canal R (comb)
//    4. rd_response_path    — top que instancia 1, 2, FIFO, 3
//
//  Nota: read_tag_generator vive en scheduler.sv.
//
//  Clocking : flanco positivo (clk)
//  Reset    : síncrono activo en bajo (rst_n)
// ============================================================

`timescale 1ns/1ps


// ============================================================
// 1. RD_RESP_MUX
//    Completamente combinacional — sin registros.
//    OR-reduce de rd_resp_valid[i] → mux_valid.
//    Priority mux: selecciona data y tag del primer banco
//    con valid=1. El Scheduler garantiza one-hot (máximo
//    1 banco completa por ciclo), el priority encoder es
//    solo por safety.
// ============================================================
module rd_resp_mux #(
    parameter N_BANKS        = 4,
    parameter AXI_DATA_WIDTH = 32
)(
    input  wire                      rd_resp_valid [0:N_BANKS-1],
    input  wire [AXI_DATA_WIDTH-1:0] rd_resp_data  [0:N_BANKS-1],
    input  wire [$clog2(N_BANKS)-1:0] rd_resp_tag  [0:N_BANKS-1],

    output wire                                    mux_valid,
    output reg  [AXI_DATA_WIDTH-1:0]               mux_data,
    output reg  [$clog2(N_BANKS)-1:0]              mux_tag
);

    localparam BANK_BITS = $clog2(N_BANKS);

    integer i;
    reg mux_valid_r;
    always_comb begin
        mux_valid_r = 1'b0;
        mux_data    = {AXI_DATA_WIDTH{1'b0}};
        mux_tag     = {BANK_BITS{1'b0}};
        for (i = 0; i < N_BANKS; i = i + 1) begin
            if (rd_resp_valid[i]) begin
                mux_valid_r = 1'b1;
                mux_data    = rd_resp_data[i];
                mux_tag     = rd_resp_tag[i];
            end
        end
    end
    assign mux_valid = mux_valid_r;

endmodule


// ============================================================
// 3. REORDER_BUFFER
//    Buffer circular de N_BANKS entradas — Opción B
//    (punteros con wrap bit, sin contador up/down).
//
//    Submódulos lógicos del diagrama de tercer nivel:
//
//      Response capture : mux_tag → addr de escritura
//                         valid <= 1, data[tag] = mux_data
//
//      Buffer array     : N_BANKS slots de {valid, data}
//
//      In-order drain   : rd_ptr selecciona slot,
//          (mux)          saca data si valid=1
//
//      Head pointer     : rd_ptr de BANK_BITS+1 bits,
//          (rd_ptr)       avanza con commit (overflow natural)
//
//      Commit ctrl      : rob_push = valid[rd_ptr] && !rd_resp_fifo_full
//                         clear_valid = rob_push
//                         inc rd_ptr = rob_push
//
//      Full comparator  : full = (wr_msb != rd_msb) &&
//                                (wr_addr == rd_addr)
//                         rob_tag_free = !full
// ============================================================
module reorder_buffer #(
    parameter N_BANKS        = 4,
    parameter AXI_DATA_WIDTH = 32
)(
    input  wire clk,
    input  wire rst_n,

    // ── Response capture (desde rd_resp_mux) ─────────
    input  wire                          mux_valid,
    input  wire [AXI_DATA_WIDTH-1:0]     mux_data,
    input  wire [$clog2(N_BANKS)-1:0]    mux_tag,

    // ── Puntero de escritura extendido (desde Tag Gen) ─
    input  wire [$clog2(N_BANKS):0]      wr_ptr_ext,

    // ── Backpressure desde RD RESP FIFO ──────────────
    input  wire                          rd_resp_fifo_full,

    // ── Hacia RD RESP FIFO ───────────────────────────
    output wire                          rob_push,
    output wire [AXI_DATA_WIDTH-1:0]     rob_data_out,

    // ── Hacia Priority Resolver (Scheduler) ──────────
    output wire                          rob_tag_free
);

    localparam BANK_BITS = $clog2(N_BANKS);
    localparam PTR_W     = BANK_BITS + 1;
    localparam [BANK_BITS-1:0] ADDR_MAX = N_BANKS - 1;

    // ── Buffer array: {valid, data} por slot ─────────
    reg                      rob_valid [0:N_BANKS-1];
    reg [AXI_DATA_WIDTH-1:0] rob_data  [0:N_BANKS-1];

    // ── Head pointer (rd_ptr) con wrap bit ───────────
    //    Cuenta explícita 0..N_BANKS-1 con wrap manual
    //    (necesario para N_BANKS no potencia de 2, e.g. 12)
    reg [BANK_BITS-1:0] rd_ptr_addr;
    reg rd_ptr_wrap;

    wire [PTR_W-1:0] rd_ptr_ext;
    assign rd_ptr_ext = {rd_ptr_wrap, rd_ptr_addr};

    // ── In-order drain: rd_ptr selecciona slot ───────
    assign rob_data_out = rob_data[rd_ptr_addr];

    // ── Commit ctrl ──────────────────────────────────
    //    rob_push = valid[rd_ptr] && !rd_resp_fifo_full
    //    clear_valid = rob_push
    //    inc rd_ptr  = rob_push
    assign rob_push = rob_valid[rd_ptr_addr] & !rd_resp_fifo_full;

    // ── Full comparator ──────────────────────────────
    //    full = (wr_msb != rd_msb) && (wr_addr == rd_addr)
    wire full;
    assign full = (wr_ptr_ext[BANK_BITS] != rd_ptr_wrap) & (wr_ptr_ext[BANK_BITS-1:0] == rd_ptr_addr);

    assign rob_tag_free = !full;

    // ── Sequential 1: Buffer Array ───────────────────────
    //    Escrito por Response Capture (mux_valid) y
    //    limpiado por Commit clear_valid (rob_push)
    integer k;
    always @(posedge clk) begin
        if (!rst_n) begin
            for (k = 0; k < N_BANKS; k = k + 1) begin
                rob_valid[k] <= 1'b0;
                rob_data[k]  <= {AXI_DATA_WIDTH{1'b0}};
            end
        end else begin
            if (mux_valid) begin
                rob_valid[mux_tag] <= 1'b1;
                rob_data[mux_tag]  <= mux_data;
            end
            if (rob_push)
                rob_valid[rd_ptr_addr] <= 1'b0;
        end
    end

    // ── Sequential 2: Head Pointer (rd_ptr) ──────────────
    //    Avanza solo con rob_push; wrap manual para N_BANKS
    //    no potencia de 2
    always @(posedge clk) begin
        if (!rst_n) begin
            rd_ptr_addr <= {BANK_BITS{1'b0}};
            rd_ptr_wrap <= 1'b0;
        end else if (rob_push) begin
            if (rd_ptr_addr == ADDR_MAX) begin
                rd_ptr_addr <= {BANK_BITS{1'b0}};
                rd_ptr_wrap <= !rd_ptr_wrap;
            end else begin
                rd_ptr_addr <= rd_ptr_addr + {{(BANK_BITS-1){1'b0}}, 1'b1};
            end
        end
    end

endmodule


// ============================================================
// 4. R_CHANNEL_LOGIC
//    Completamente combinacional — sin registros.
//      rvalid   = pndng (FIFO tiene dato)
//      data     = Dout
//      rresp    = 2'b00 (OKAY, constante cableada)
//      fifo_pop = rvalid && rready (handshake fire)
// ============================================================
module r_channel_logic #(
    parameter AXI_DATA_WIDTH = 32
)(
    // ── Desde RD RESP FIFO ───────────────────────────
    input  wire                        fifo_pndng,
    input  wire [AXI_DATA_WIDTH-1:0]   fifo_dout,

    // ── Hacia RD RESP FIFO ───────────────────────────
    output wire                        fifo_pop,

    // ── AXI4-Lite R Channel ──────────────────────────
    output wire                        s_axi_rvalid,
    output wire [AXI_DATA_WIDTH-1:0]   s_axi_rdata,
    output wire [1:0]                  s_axi_rresp,
    input  wire                        s_axi_rready
);

    assign s_axi_rvalid = fifo_pndng;
    assign s_axi_rdata  = fifo_dout;
    assign s_axi_rresp  = 2'b00;

    assign fifo_pop = s_axi_rvalid & s_axi_rready;

endmodule


// ============================================================
// 5. RD_RESPONSE_PATH — TOP
//    Instancia y conecta:
//      2. rd_resp_mux       — combi
//      3. reorder_buffer    — ROB
//         FIFO externa      — IP verificada (RD RESP FIFO)
//      4. r_channel_logic   — combi
//
//    El read_tag_generator (1) vive en el Scheduler;
//    este módulo recibe wr_ptr_ext como entrada.
// ============================================================
module rd_response_path #(
    parameter N_BANKS        = 4,
    parameter AXI_DATA_WIDTH = 32
)(
    input  wire clk,
    input  wire rst_n,

    // ── Desde Bank Controllers ───────────────────────
    input  wire                       rd_resp_valid [0:N_BANKS-1],
    input  wire [AXI_DATA_WIDTH-1:0]  rd_resp_data  [0:N_BANKS-1],
    input  wire [$clog2(N_BANKS)-1:0] rd_resp_tag   [0:N_BANKS-1],

    // ── Desde Scheduler (Tag Generator) ──────────────
    input  wire [$clog2(N_BANKS):0]                    wr_ptr_ext,

    // ── Hacia Scheduler (Priority Resolver) ──────────
    output wire                                        rob_tag_free,

    // ── AXI4-Lite R Channel ──────────────────────────
    output wire                                        s_axi_rvalid,
    output wire [AXI_DATA_WIDTH-1:0]                   s_axi_rdata,
    output wire [1:0]                                  s_axi_rresp,
    input  wire                                        s_axi_rready
);

    localparam BANK_BITS = $clog2(N_BANKS);

    // ── Señales internas ─────────────────────────────
    wire                      mux_valid;
    wire [AXI_DATA_WIDTH-1:0] mux_data;
    wire [BANK_BITS-1:0]      mux_tag;

    wire                      rob_push;
    wire [AXI_DATA_WIDTH-1:0] rob_data_out;

    wire                      rd_resp_fifo_full;
    wire                      fifo_pndng;
    wire [AXI_DATA_WIDTH-1:0] fifo_dout;
    wire                      fifo_pop;

    // ── 2. RD Resp Mux ──────────────────────────────
    rd_resp_mux #(
        .N_BANKS       (N_BANKS),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) u_rd_resp_mux (
        .rd_resp_valid (rd_resp_valid),
        .rd_resp_data  (rd_resp_data),
        .rd_resp_tag   (rd_resp_tag),
        .mux_valid     (mux_valid),
        .mux_data      (mux_data),
        .mux_tag       (mux_tag)
    );

    // ── 3. Reorder Buffer ───────────────────────────
    reorder_buffer #(
        .N_BANKS       (N_BANKS),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) u_reorder_buffer (
        .clk              (clk),
        .rst_n            (rst_n),
        .mux_valid        (mux_valid),
        .mux_data         (mux_data),
        .mux_tag          (mux_tag),
        .wr_ptr_ext       (wr_ptr_ext),
        .rd_resp_fifo_full(rd_resp_fifo_full),
        .rob_push         (rob_push),
        .rob_data_out     (rob_data_out),
        .rob_tag_free     (rob_tag_free)
    );

    // ── RD RESP FIFO (IP externa) ───────────────────
    //    Interfaz fija: Din, Dout, push, pop, clk,
    //    full, pndng, rst (activo en ALTO)
    //    bits = AXI_DATA_WIDTH (solo data)
    fifo_flops #(
        .bits(AXI_DATA_WIDTH)
    ) u_rd_resp_fifo (
        .Din  (rob_data_out),
        .Dout (fifo_dout),
        .push (rob_push),
        .pop  (fifo_pop),
        .clk  (clk),
        .full (rd_resp_fifo_full),
        .pndng(fifo_pndng),
        .rst  (~rst_n)
    );

    // ── 4. R Channel Logic ──────────────────────────
    r_channel_logic #(
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) u_r_channel_logic (
        .fifo_pndng   (fifo_pndng),
        .fifo_dout    (fifo_dout),
        .fifo_pop     (fifo_pop),
        .s_axi_rvalid (s_axi_rvalid),
        .s_axi_rdata  (s_axi_rdata),
        .s_axi_rresp  (s_axi_rresp),
        .s_axi_rready (s_axi_rready)
    );

endmodule
