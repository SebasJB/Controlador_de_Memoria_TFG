// ============================================================
//  File    : rd_response_path.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : Read Response Path — Canal R (completo)
//
//  Módulos:
//    1. reorder_buffer      — ROB con captura paralela por tag (reg)
//    2. r_channel_logic     — AXI4-Lite canal R (comb)
//    3. rd_response_path    — top que instancia 1, FIFO, 2
//
//  Nota: read_tag_generator vive en scheduler.sv.
//
//  Clocking : flanco positivo (clk)
//  Reset    : síncrono activo en bajo (rst_n)
// ============================================================

`timescale 1ns/1ps


// ============================================================
// 1. REORDER_BUFFER
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

    // ── Response capture DIRECTA desde N bancos ──────
    //    Escritura paralela: cada banco escribe a su tag.
    //    Tags son únicos por construcción del Tag Generator,
    //    por lo que NO hay conflictos de escritura.
    input  wire                          rd_resp_valid [0:N_BANKS-1],
    input  wire [AXI_DATA_WIDTH-1:0]     rd_resp_data  [0:N_BANKS-1],
    input  wire [$clog2(N_BANKS)-1:0]    rd_resp_tag   [0:N_BANKS-1],

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

    // ── Sequential 1: Buffer Array (writes paralelos) ───
    //    Cada banco escribe a SU tag (associative array).
    //    El Tag Generator garantiza tags únicos por ciclo,
    //    por lo que distintos bancos escriben a slots distintos.
    //    Limpieza por Commit (rob_push) del head pointer.
    integer k, b;
    always @(posedge clk) begin
        if (!rst_n) begin
            for (k = 0; k < N_BANKS; k = k + 1) begin
                rob_valid[k] <= 1'b0;
                rob_data[k]  <= {AXI_DATA_WIDTH{1'b0}};
            end
        end else begin
            // Captura paralela: cada banco escribe a su tag
            for (b = 0; b < N_BANKS; b = b + 1) begin
                if (rd_resp_valid[b]) begin
                    rob_data [rd_resp_tag[b]] <= rd_resp_data[b];
                    rob_valid[rd_resp_tag[b]] <= 1'b1;
                end
            end
            // Commit: limpia el slot del head pointer
            // NOTA: si hay escritura paralela al mismo slot que se limpia,
            // la escritura GANA (orden no-bloqueante de SystemVerilog:
            // last assignment wins en el mismo always_ff).
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
// 2. R_CHANNEL_LOGIC
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
// 3. RD_RESPONSE_PATH — TOP
//    Instancia y conecta:
//      1. rd_resp_mux       — combi
//      2. reorder_buffer    — ROB
//         FIFO externa      — IP verificada (RD RESP FIFO)
//      3. r_channel_logic   — combi
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
    wire                      rob_push;
    wire [AXI_DATA_WIDTH-1:0] rob_data_out;

    wire                      rd_resp_fifo_full;
    wire                      fifo_pndng;
    wire [AXI_DATA_WIDTH-1:0] fifo_dout;
    wire                      fifo_pop;

    // ── Reorder Buffer con captura paralela ─────────
    //    Recibe directamente de los N bancos.
    //    Cada banco escribe a su tag asignado por el Tag Generator.
    reorder_buffer #(
        .N_BANKS       (N_BANKS),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) u_reorder_buffer (
        .clk              (clk),
        .rst_n            (rst_n),
        .rd_resp_valid    (rd_resp_valid),
        .rd_resp_data     (rd_resp_data),
        .rd_resp_tag      (rd_resp_tag),
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
    fifo_generic #(
        .DataWidth(AXI_DATA_WIDTH)
    ) u_rd_resp_fifo (
        .writeData(rob_data_out),
        .readData (fifo_dout),
        .writeEn  (rob_push),
        .readEn   (fifo_pop),
        .clk      (clk),
        .full     (rd_resp_fifo_full),
        .pndng    (fifo_pndng),
        .rst      (~rst_n)
    );

    // ── 2. R Channel Logic ──────────────────────────
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
