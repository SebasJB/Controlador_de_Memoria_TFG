// ============================================================
//  File    : mem_handler_top.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : Top-level del sistema completo
//
//  Contiene:
//    1. sram_sync          — modelo behavioral de SRAM síncrona
//    2. mem_handler_top    — módulo top (solo instancias)
//
//  Bloques instanciados:
//    - axi4_lite_front_end  — slave AXI4-Lite (AW/W/AR)
//    - fifo (×2)            — WR REQ FIFO y RD REQ FIFO (IP externa)
//    - addr_decoder (×2)    — decodificación WR y RD, validación de rango
//    - scheduler            — árbitro WRITE-FIRST, dispatch, métricas M5
//    - sram_bank_controller — controlador por banco (×N_BANKS, genvar)
//    - sram_sync            — modelo behavioral de SRAM (×N_BANKS, genvar)
//    - wr_response_path     — canal B AXI4-Lite (pending counter)
//    - rd_response_path     — canal R AXI4-Lite (ROB + FIFO de salida)
//
//  Decisiones arquitectónicas clave:
//    - No lógica inline en el top: solo declaraciones de señales
//      e instanciaciones de módulos
//    - Backpressure completo:
//        wr_resp_full (de wr_response_path) → scheduler
//        rob_tag_free (de rd_response_path) → scheduler
//    - wr_req_data_out se rutea al scheduler (broadcast wdata/wstrb)
//      Y al addr_decoder WR (slicing de addr con assign)
//    - rd_req_data_out solo al addr_decoder RD (lecturas sin data)
//    - WR REQ bus packed: {op(1), addr(ADDR_W), data(DATA_W), strb(STRB_W)}
//    - RD REQ bus packed: {op(1), addr(ADDR_W)}
//
//  Clocking : flanco positivo (clk)
//  Reset    : síncrono activo en bajo (rst_n)
//             La FIFO IP usa rst activo en alto → se conecta con ~rst_n
// ============================================================

`timescale 1ns/1ps

`include "./Controlador_de_Memoria_TFG/fifo.sv"
`include "./Controlador_de_Memoria_TFG/addr_decoder/addr_decoder.sv"
`include "./Controlador_de_Memoria_TFG/axi4_lite_front_end/axi4_lite_front_end.sv"
`include "./Controlador_de_Memoria_TFG/scheduler/scheduler.sv"
`include "./Controlador_de_Memoria_TFG/sram_bank_controller/sram_bank_controller.sv"
`include "./Controlador_de_Memoria_TFG/wr_response_path/wr_response_path.sv"
`include "./Controlador_de_Memoria_TFG/rd_response_path/rd_response_path.sv"


// ============================================================
// SRAM_SYNC — Modelo behavioral de SRAM síncrona parametrizable
//
// Behavioral SRAM model. For ASIC synthesis, replace with
// foundry SRAM macro.
//
// Comportamiento:
//   - Escritura byte-enabled: en cada posedge clk, si en && we,
//     solo los bytes con wstrb[i]=1 se escriben en mem[addr].
//   - Lectura con pipeline: si en && !we, mem[addr] entra al
//     shift register de READ_LATENCY etapas. dout es la salida
//     del último registro del pipeline.
//   - Para READ_LATENCY=1: pipeline de 1 registro → latencia 1 ciclo.
//   - Inicialización a 0 en initial block para simulación determinista;
//     este bloque es ignorado en síntesis ASIC.
// ============================================================
module sram_sync #(
    parameter AXI_DATA_WIDTH  = 32,
    parameter BANK_ADDR_WIDTH = 8,
    parameter READ_LATENCY    = 1
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        en,
    input  wire                        we,
    input  wire [BANK_ADDR_WIDTH-1:0]  addr,
    input  wire [AXI_DATA_WIDTH-1:0]   din,
    input  wire [AXI_DATA_WIDTH/8-1:0] wstrb,
    output wire [AXI_DATA_WIDTH-1:0]   dout
);
    localparam MEM_DEPTH = 1 << BANK_ADDR_WIDTH;
    localparam STRB_W    = AXI_DATA_WIDTH / 8;

    // ── Array de memoria ─────────────────────────────────────
    reg [AXI_DATA_WIDTH-1:0] mem [0:MEM_DEPTH-1];

    // ── Pipeline de lectura (READ_LATENCY etapas) ────────────
    // Para READ_LATENCY=1: un solo registro (rd_pipe[0]).
    reg [AXI_DATA_WIDTH-1:0] rd_pipe [0:READ_LATENCY-1];

    integer j;

    // Para simulación determinista: inicializar todo a 0.
    // En síntesis ASIC este bloque se ignora; la macro de foundry
    // no garantiza estado inicial de la memoria.
    integer k;
    initial begin
        for (k = 0; k < MEM_DEPTH; k = k + 1)
            mem[k] = {AXI_DATA_WIDTH{1'b0}};
    end

    always @(posedge clk) begin
        // ── Escritura byte-enabled ────────────────────────
        if (en && we) begin
            for (j = 0; j < STRB_W; j = j + 1) begin
                if (wstrb[j])
                    mem[addr][j*8 +: 8] <= din[j*8 +: 8];
            end
        end

        // ── Etapa 0: captura dato leído de la memoria ─────
        if (en && !we)
            rd_pipe[0] <= mem[addr];

        // ── Etapas 1..READ_LATENCY-1: desplazamiento ──────
        // Si READ_LATENCY==1, este loop no ejecuta (j parte en 1,
        // condición j < 1 es falsa). Correcto por diseño.
        for (j = 1; j < READ_LATENCY; j = j + 1)
            rd_pipe[j] <= rd_pipe[j-1];
    end

    // Salida: último elemento del pipeline de lectura
    assign dout = rd_pipe[READ_LATENCY-1];

endmodule


// ============================================================
// MEM_HANDLER_TOP — Top-level del controlador de memoria
// ============================================================
module mem_handler_top #(
    parameter ADDR_W            = 32,
    parameter AXI_DATA_WIDTH    = 32,
    parameter N_BANKS           = 4,
    parameter BANK_SIZE_BYTES   = 1024,
    parameter READ_LATENCY      = 1,
    parameter LAT_CNT_W         = 8,
    parameter WR_REQ_FIFO_DEPTH = 8,
    parameter RD_REQ_FIFO_DEPTH = 8,
    parameter WR_RESP_DEPTH     = 8,
    parameter ERR_CNT_W         = 16
)(
    input  wire clk,
    input  wire rst_n,

    // ── AXI4-Lite slave AW ────────────────────────────────
    input  wire              s_axi_awvalid,
    output wire              s_axi_awready,
    input  wire [ADDR_W-1:0] s_axi_awaddr,

    // ── AXI4-Lite slave W ─────────────────────────────────
    input  wire                        s_axi_wvalid,
    output wire                        s_axi_wready,
    input  wire [AXI_DATA_WIDTH-1:0]   s_axi_wdata,
    input  wire [AXI_DATA_WIDTH/8-1:0] s_axi_wstrb,

    // ── AXI4-Lite slave B ─────────────────────────────────
    output wire       s_axi_bvalid,
    output wire [1:0] s_axi_bresp,
    input  wire       s_axi_bready,

    // ── AXI4-Lite slave AR ────────────────────────────────
    input  wire              s_axi_arvalid,
    output wire              s_axi_arready,
    input  wire [ADDR_W-1:0] s_axi_araddr,

    // ── AXI4-Lite slave R ─────────────────────────────────
    output wire                       s_axi_rvalid,
    output wire [AXI_DATA_WIDTH-1:0]  s_axi_rdata,
    output wire [1:0]                 s_axi_rresp,
    input  wire                       s_axi_rready,

    // ── Métricas M5 (visibles al exterior) ────────────────
    output wire [ERR_CNT_W-1:0] wr_err_cnt,
    output wire [ERR_CNT_W-1:0] rd_err_cnt
);

    // ── Localparams derivados ─────────────────────────────
    localparam BANK_BITS       = $clog2(N_BANKS);
    localparam WORD_BYTES      = AXI_DATA_WIDTH / 8;
    localparam BANK_WORDS      = BANK_SIZE_BYTES / WORD_BYTES;
    localparam BANK_ADDR_WIDTH = $clog2(BANK_WORDS);
    localparam STRB_W          = AXI_DATA_WIDTH / 8;
    // Anchos de los buses packed de las FIFOs REQ
    localparam WR_REQ_BUS_W    = 1 + ADDR_W + AXI_DATA_WIDTH + STRB_W;
    localparam RD_REQ_BUS_W    = 1 + ADDR_W;

    // =========================================================
    // Declaración de señales internas — agrupadas por bloque
    // =========================================================

    // ── Buses packed FIFOs REQ ───────────────────────────────
    // WR REQ: {op(1), addr(ADDR_W), data(DATA_W), strb(STRB_W)}
    wire [WR_REQ_BUS_W-1:0] wr_req_data;       // front-end → FIFO Din
    wire [WR_REQ_BUS_W-1:0] wr_req_data_out;   // FIFO Dout → sched + slicer
    // RD REQ: {op(1), addr(ADDR_W)}
    wire [RD_REQ_BUS_W-1:0] rd_req_data;       // front-end → FIFO Din
    wire [RD_REQ_BUS_W-1:0] rd_req_data_out;   // FIFO Dout → slicer

    // ── Control WR REQ FIFO ───────────────────────────────────
    wire wr_req_push;    // front-end → FIFO push
    wire wr_req_pop;     // scheduler → FIFO pop
    wire wr_req_full;    // FIFO full → front-end backpressure
    wire wr_req_pndng;   // FIFO pndng → scheduler

    // ── Control RD REQ FIFO ───────────────────────────────────
    wire rd_req_push;    // front-end → FIFO push
    wire rd_req_pop;     // scheduler → FIFO pop
    wire rd_req_full;    // FIFO full → front-end backpressure
    wire rd_req_pndng;   // FIFO pndng → scheduler

    // ── Addr decoder WR ───────────────────────────────────────
    wire [ADDR_W-1:0]          wr_addr;          // sliced de wr_req_data_out
    wire [BANK_BITS-1:0]       wr_bank_id;
    wire [BANK_ADDR_WIDTH-1:0] wr_bank_word_addr;
    wire                       wr_addr_valid;

    // ── Addr decoder RD ───────────────────────────────────────
    wire [ADDR_W-1:0]          rd_addr;          // sliced de rd_req_data_out
    wire [BANK_BITS-1:0]       rd_bank_id;
    wire [BANK_ADDR_WIDTH-1:0] rd_bank_word_addr;
    wire                       rd_addr_valid;

    // ── Scheduler dispatch ────────────────────────────────────
    wire                       bank_req_valid [0:N_BANKS-1];
    wire                       bank_req_op    [0:N_BANKS-1];
    wire [BANK_ADDR_WIDTH-1:0] bank_req_addr  [0:N_BANKS-1];
    wire [AXI_DATA_WIDTH-1:0]  bank_req_wdata;           // broadcast
    wire [AXI_DATA_WIDTH/8-1:0]bank_req_wstrb;           // broadcast
    wire [BANK_BITS-1:0]       bank_req_tag;              // broadcast
    wire [BANK_BITS:0]         wr_ptr_ext;                // → ROB

    // ── Interfaces SRAM (arrays indexados por banco) ──────────
    wire                        sram_en_arr    [0:N_BANKS-1];
    wire                        sram_we_arr    [0:N_BANKS-1];
    wire [BANK_ADDR_WIDTH-1:0]  sram_addr_arr  [0:N_BANKS-1];
    wire [AXI_DATA_WIDTH-1:0]   sram_din_arr   [0:N_BANKS-1];
    wire [AXI_DATA_WIDTH/8-1:0] sram_wstrb_arr [0:N_BANKS-1];
    wire [AXI_DATA_WIDTH-1:0]   sram_dout_arr  [0:N_BANKS-1];

    // ── Estado de bancos y respuestas ─────────────────────────
    wire                       bank_busy     [0:N_BANKS-1];
    wire                       wr_resp_valid [0:N_BANKS-1];

    // ── Respuestas de lectura por banco → rd_response_path ────
    wire [AXI_DATA_WIDTH-1:0]  rd_resp_data  [0:N_BANKS-1];
    wire [BANK_BITS-1:0]       rd_resp_tag   [0:N_BANKS-1];
    wire                       rd_resp_valid [0:N_BANKS-1];

    // ── Backpressure entre bloques de respuesta y scheduler ───
    wire rob_tag_free;   // rd_response_path → scheduler
    wire wr_resp_full;   // wr_response_path → scheduler

    // =========================================================
    // Slicing de direcciones desde la salida de las FIFOs REQ
    // (assigns combinacionales — no lógica inline en el top)
    //
    // WR REQ bus: {op[WR_REQ_BUS_W-1], addr[...], data[...], strb[STRB_W-1:0]}
    //   wr_addr ocupa los bits [ADDR_W+DATA_W+STRB_W-1 : DATA_W+STRB_W]
    //   Con los valores por defecto: bits [67:36] de wr_req_data_out
    //
    // RD REQ bus: {op[RD_REQ_BUS_W-1], addr[ADDR_W-1:0]}
    //   rd_addr ocupa los bits [ADDR_W-1:0] de rd_req_data_out
    // =========================================================
    assign wr_addr = wr_req_data_out[ADDR_W + AXI_DATA_WIDTH + STRB_W - 1 :
                                     AXI_DATA_WIDTH + STRB_W];

    assign rd_addr = rd_req_data_out[ADDR_W-1:0];


    // =========================================================
    // Instancias — en orden de flujo de datos
    // =========================================================

    // ── 1. AXI4-Lite Front End ───────────────────────────────
    axi4_lite_front_end #(
        .ADDR_W        (ADDR_W),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) u_axi_front_end (
        .clk           (clk),
        .rst_n         (rst_n),
        // Canal AW
        .s_axi_awvalid (s_axi_awvalid),
        .s_axi_awready (s_axi_awready),
        .s_axi_awaddr  (s_axi_awaddr),
        // Canal W
        .s_axi_wvalid  (s_axi_wvalid),
        .s_axi_wready  (s_axi_wready),
        .s_axi_wdata   (s_axi_wdata),
        .s_axi_wstrb   (s_axi_wstrb),
        // Canal AR
        .s_axi_arvalid (s_axi_arvalid),
        .s_axi_arready (s_axi_arready),
        .s_axi_araddr  (s_axi_araddr),
        // Hacia WR REQ FIFO
        .wr_req_push   (wr_req_push),
        .wr_req_data   (wr_req_data),
        .wr_req_full   (wr_req_full),
        // Hacia RD REQ FIFO
        .rd_req_push   (rd_req_push),
        .rd_req_data   (rd_req_data),
        .rd_req_full   (rd_req_full)
    );


    // ── 2. WR REQ FIFO ───────────────────────────────────────
    // Bus packed: {op(1), addr(ADDR_W), data(DATA_W), strb(STRB_W)}
    // rst = ~rst_n (la IP FIFO usa reset activo en alto)
    fifo_flops #(
        .bits (WR_REQ_BUS_W),
        .depth(WR_REQ_FIFO_DEPTH)
    ) u_wr_req_fifo (
        .Din  (wr_req_data),
        .Dout (wr_req_data_out),
        .push (wr_req_push),
        .pop  (wr_req_pop),
        .clk  (clk),
        .full (wr_req_full),
        .pndng(wr_req_pndng),
        .rst  (~rst_n)
    );


    // ── 3. RD REQ FIFO ───────────────────────────────────────
    // Bus packed: {op(1), addr(ADDR_W)}
    // rst = ~rst_n (la IP FIFO usa reset activo en alto)
    fifo_flops #(
        .bits (RD_REQ_BUS_W),
        .depth(RD_REQ_FIFO_DEPTH)
    ) u_rd_req_fifo (
        .Din  (rd_req_data),
        .Dout (rd_req_data_out),
        .push (rd_req_push),
        .pop  (rd_req_pop),
        .clk  (clk),
        .full (rd_req_full),
        .pndng(rd_req_pndng),
        .rst  (~rst_n)
    );


    // ── 4a. Addr Decoder — WR ────────────────────────────────
    // La dirección se extrae del bus packed de la WR REQ FIFO
    // mediante el assign wr_addr declarado antes de las instancias.
    addr_decoder #(
        .ADDR_W         (ADDR_W),
        .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
        .N_BANKS        (N_BANKS),
        .BANK_SIZE_BYTES(BANK_SIZE_BYTES)
    ) u_wr_addr_decoder (
        .req_addr      (wr_addr),
        .bank_id       (wr_bank_id),
        .bank_word_addr(wr_bank_word_addr),
        .addr_valid    (wr_addr_valid)
    );


    // ── 4b. Addr Decoder — RD ────────────────────────────────
    // La dirección se extrae del bus packed de la RD REQ FIFO
    // mediante el assign rd_addr declarado antes de las instancias.
    addr_decoder #(
        .ADDR_W         (ADDR_W),
        .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
        .N_BANKS        (N_BANKS),
        .BANK_SIZE_BYTES(BANK_SIZE_BYTES)
    ) u_rd_addr_decoder (
        .req_addr      (rd_addr),
        .bank_id       (rd_bank_id),
        .bank_word_addr(rd_bank_word_addr),
        .addr_valid    (rd_addr_valid)
    );


    // ── 5. Scheduler ─────────────────────────────────────────
    // Recibe wr_req_data_out (no wr_req_data) — el scheduler
    // extrae wdata/wstrb del dato ya dequeued de la FIFO.
    // wr_resp_full y rob_tag_free son los dos caminos de
    // backpressure hacia el árbitro.
    scheduler #(
        .ADDR_W         (ADDR_W),
        .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
        .N_BANKS        (N_BANKS),
        .BANK_SIZE_BYTES(BANK_SIZE_BYTES),
        .ERR_CNT_W      (ERR_CNT_W)
    ) u_scheduler (
        .clk              (clk),
        .rst_n            (rst_n),
        // Desde Addr Decoders
        .wr_bank_id       (wr_bank_id),
        .rd_bank_id       (rd_bank_id),
        .wr_bank_word_addr(wr_bank_word_addr),
        .rd_bank_word_addr(rd_bank_word_addr),
        .wr_addr_valid    (wr_addr_valid),
        .rd_addr_valid    (rd_addr_valid),
        // Desde FIFOs REQ (pending flags)
        .wr_req_pndng     (wr_req_pndng),
        .rd_req_pndng     (rd_req_pndng),
        // Bus packed WR para broadcast wdata/wstrb
        .wr_req_data      (wr_req_data_out),
        // Desde Bank Controllers
        .bank_busy        (bank_busy),
        // Backpressure del ROB
        .rob_tag_free     (rob_tag_free),
        // Backpressure del canal B
        .wr_resp_full     (wr_resp_full),
        // Hacia FIFOs REQ (pop)
        .wr_req_pop       (wr_req_pop),
        .rd_req_pop       (rd_req_pop),
        // Dispatch hacia bank controllers
        .bank_req_valid   (bank_req_valid),
        .bank_req_op      (bank_req_op),
        .bank_req_addr    (bank_req_addr),
        .bank_req_wdata   (bank_req_wdata),
        .bank_req_wstrb   (bank_req_wstrb),
        // Tag y puntero de escritura del ROB
        .bank_req_tag     (bank_req_tag),
        .wr_ptr_ext       (wr_ptr_ext),
        // Métricas M5
        .wr_err_cnt       (wr_err_cnt),
        .rd_err_cnt       (rd_err_cnt)
    );


    // ── 6. SRAM Bank Controllers (×N_BANKS) ──────────────────
    // bank_req_wdata, bank_req_wstrb, bank_req_tag son señales
    // broadcast del scheduler — la misma señal va a todos los
    // bancos; bank_req_valid[i] selecciona cuál captura.
    genvar i;
    generate
        for (i = 0; i < N_BANKS; i = i + 1) begin : gen_bank
            sram_bank_controller #(
                .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
                .BANK_ADDR_WIDTH(BANK_ADDR_WIDTH),
                .N_BANKS        (N_BANKS),
                .READ_LATENCY   (READ_LATENCY),
                .LAT_CNT_W      (LAT_CNT_W)
            ) u_bank (
                .clk           (clk),
                .rst_n         (rst_n),
                // Del Scheduler (dispatch)
                .bank_req_valid(bank_req_valid[i]),
                .bank_req_op   (bank_req_op[i]),
                .bank_req_addr (bank_req_addr[i]),
                .bank_req_wdata(bank_req_wdata),     // broadcast
                .bank_req_wstrb(bank_req_wstrb),     // broadcast
                .bank_req_tag  (bank_req_tag),       // broadcast
                // Hacia SRAM IP
                .sram_en       (sram_en_arr[i]),
                .sram_we       (sram_we_arr[i]),
                .sram_addr     (sram_addr_arr[i]),
                .sram_din      (sram_din_arr[i]),
                .sram_wstrb    (sram_wstrb_arr[i]),
                .sram_dout     (sram_dout_arr[i]),
                // Hacia Scheduler (estado)
                .bank_busy     (bank_busy[i]),
                // Hacia WR Response Path
                .wr_resp_valid (wr_resp_valid[i]),
                // Hacia RD Response Path
                .rd_resp_data  (rd_resp_data[i]),
                .rd_resp_tag   (rd_resp_tag[i]),
                .rd_resp_valid (rd_resp_valid[i])
            );
        end
    endgenerate


    // ── 7. SRAM Behavioral Models (×N_BANKS) ─────────────────
    // Para síntesis ASIC: reemplazar sram_sync por macro de foundry
    // con la misma interfaz (en, we, addr, din, wstrb, dout).
    generate
        for (i = 0; i < N_BANKS; i = i + 1) begin : gen_sram
            sram_sync #(
                .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
                .BANK_ADDR_WIDTH(BANK_ADDR_WIDTH),
                .READ_LATENCY   (READ_LATENCY)
            ) u_sram (
                .clk  (clk),
                .rst_n(rst_n),
                .en   (sram_en_arr[i]),
                .we   (sram_we_arr[i]),
                .addr (sram_addr_arr[i]),
                .din  (sram_din_arr[i]),
                .wstrb(sram_wstrb_arr[i]),
                .dout (sram_dout_arr[i])
            );
        end
    endgenerate


    // ── 8. WR Response Path (canal B AXI4-Lite) ──────────────
    // wr_resp_full sale de aquí hacia el scheduler como
    // backpressure: el scheduler no despacha writes cuando
    // el pending counter del canal B está lleno.
    wr_response_path #(
        .N_BANKS      (N_BANKS),
        .WR_FIFO_DEPTH(WR_RESP_DEPTH)
    ) u_wr_response_path (
        .clk          (clk),
        .rst_n        (rst_n),
        .wr_resp_valid(wr_resp_valid),
        .s_axi_bvalid (s_axi_bvalid),
        .s_axi_bresp  (s_axi_bresp),
        .s_axi_bready (s_axi_bready),
        .wr_resp_full (wr_resp_full)
    );


    // ── 9. RD Response Path (canal R AXI4-Lite) ──────────────
    // wr_ptr_ext viene del scheduler (Read Tag Generator).
    // rob_tag_free sale hacia el scheduler para evitar aliasing
    // de tags cuando el contador de lecturas da vuelta.
    rd_response_path #(
        .N_BANKS       (N_BANKS),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) u_rd_response_path (
        .clk          (clk),
        .rst_n        (rst_n),
        // Desde Bank Controllers
        .rd_resp_valid(rd_resp_valid),
        .rd_resp_data (rd_resp_data),
        .rd_resp_tag  (rd_resp_tag),
        // Desde Scheduler (Tag Generator)
        .wr_ptr_ext   (wr_ptr_ext),
        // Hacia Scheduler (backpressure ROB)
        .rob_tag_free (rob_tag_free),
        // Canal R AXI4-Lite
        .s_axi_rvalid (s_axi_rvalid),
        .s_axi_rdata  (s_axi_rdata),
        .s_axi_rresp  (s_axi_rresp),
        .s_axi_rready (s_axi_rready)
    );

endmodule
