// ============================================================
//  Module  : wr_response_path
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : Write Response Path (B Channel AXI4-Lite)
//
//  Incluye:
//    1. wr_resp_collector  — OR-reduce de wr_resp_valid[i]
//    2. pending_counter    — reemplaza FIFO (up/down counter)
//    3. b_channel_logic    — interfaz AXI4-Lite canal B
//    4. wr_response_path   — top que instancia los 3 bloques
//
//  Optimización: la WR RESP FIFO se reemplaza por un contador
//  up/down porque BRESP=OKAY es constante — no hay dato
//  variable que almacenar. Solo se necesita saber cuántas
//  respuestas están pendientes de ser consumidas por el master.
//
//  Casos del contador:
//    push && !pop  → cnt++   (nueva resp, master no acepta)
//    pop  && !push → cnt--   (master acepta, no hay nueva resp)
//    push &&  pop  → cnt     (sin cambio — se cancelan)
//    !push && !pop → cnt     (sin cambio)
//
//  Backpressure: wr_resp_full → Scheduler deja de despachar
//  writes cuando el contador llega a WR_FIFO_DEPTH.
//
//  Dominio de reloj : clk (único dominio síncrono por ahora)
//  Reset            : síncrono activo en bajo (rst_n)
// ============================================================

`timescale 1ns/1ps

// ============================================================
// 1. WR RESP COLLECTOR
//    OR-reduce combinacional de todas las señales
//    wr_resp_valid[i] provenientes de los bank controllers.
//    El Scheduler garantiza despacho one-hot → máximo 1
//    banco completa escritura por ciclo → máximo 1 push/ciclo.
// ============================================================
module wr_resp_collector #(
    parameter N_BANKS = 4
)(
    input  wire wr_resp_valid [0:N_BANKS-1],
    output wire               wr_resp_push
);
    wire [N_BANKS-1:0] valid_flat;
    genvar i;
    generate
        for (i = 0; i < N_BANKS; i = i + 1) begin : gen_flat
            assign valid_flat[i] = wr_resp_valid[i];
        end
    endgenerate

    assign wr_resp_push = |valid_flat;

endmodule


// ============================================================
// 2. PENDING COUNTER
//    Contador up/down que reemplaza la WR RESP FIFO.
//    Rango: 0 .. WR_FIFO_DEPTH
//    Ancho: $clog2(WR_FIFO_DEPTH+1) bits
//
//    push && pop  simultáneos → cnt no cambia (se cancelan)
//    Esto evita el caso en que una respuesta entra y sale
//    en el mismo ciclo sin pasar por el contador, lo cual
//    es comportamiento correcto — bvalid ya estaba en 1.
// ============================================================
module pending_counter #(
    parameter WR_FIFO_DEPTH = 8
)(
    input  wire clk,
    input  wire rst_n,

    // Control
    input  wire push,
    input  wire pop,

    // Estado
    output wire full,
    output wire empty
);
    localparam CNT_WIDTH = $clog2(WR_FIFO_DEPTH + 1);

    reg [CNT_WIDTH-1:0] cnt;

    wire push_ok;
    wire pop_ok;
    assign push_ok = push & !full;
    assign pop_ok  = pop  & !empty;

    always @(posedge clk) begin
        if (!rst_n) begin
            cnt <= {CNT_WIDTH{1'b0}};
        end else begin
            case ({push_ok, pop_ok})
                2'b10 : cnt <= cnt + {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                2'b01 : cnt <= cnt - {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                default: cnt <= cnt;
            endcase
        end
    end

    assign full  = (cnt == WR_FIFO_DEPTH);
    assign empty = (cnt == {CNT_WIDTH{1'b0}});

endmodule


// ============================================================
// 3. B CHANNEL LOGIC (AXI4-Lite)
//    Interfaz hacia el master AXI4-Lite canal B.
//
//    bvalid   : combinacional desde !empty (cnt > 0)
//    bresp    : 2'b00 (OKAY) — constante cableada, no viene
//               del contador ni de ningún registro
//    fire     : bvalid && bready → decrementa contador (pop)
//
//    Nota: bresp podría registrarse para mejorar timing si
//    el path combinacional !empty → bvalid → master no cierra.
//    En esa situación se agrega un FF de salida aquí sin
//    cambiar la interfaz externa.
// ============================================================
module b_channel_logic (
    input  wire empty,
    output wire fire,

    output wire       s_axi_bvalid,
    output wire [1:0] s_axi_bresp,
    input  wire       s_axi_bready
);
    assign s_axi_bvalid = !empty;
    assign s_axi_bresp  = 2'b00;
    assign fire         = s_axi_bvalid & s_axi_bready;

endmodule


// ============================================================
// 4. WR_RESPONSE_PATH — TOP de este bloque
//    Instancia y conecta los 3 submódulos anteriores.
//
//    Interfaz externa:
//      - wr_resp_valid[0:N_BANKS-1] ← bank controllers
//      - wr_resp_full               → Scheduler (backpressure)
//      - s_axi_b*                   ↔ AXI4-Lite master
// ============================================================
module wr_response_path #(
    parameter N_BANKS       = 4,
    parameter WR_FIFO_DEPTH = 8
)(
    input  wire clk,
    input  wire rst_n,

    // Desde bank controllers
    input  wire wr_resp_valid [0:N_BANKS-1],

    // AXI4-Lite B Channel
    output wire       s_axi_bvalid,
    output wire [1:0] s_axi_bresp,
    input  wire       s_axi_bready,

    // Backpressure hacia Scheduler
    output wire       wr_resp_full
);

    // ── Señales internas ──────────────────────────────────
    wire wr_resp_push;
    wire cnt_empty;
    wire fire;

    // ── Instancias ────────────────────────────────────────

    wr_resp_collector #(
        .N_BANKS(N_BANKS)
    ) u_collector (
        .wr_resp_valid (wr_resp_valid),
        .wr_resp_push  (wr_resp_push)
    );

    pending_counter #(
        .WR_FIFO_DEPTH(WR_FIFO_DEPTH)
    ) u_counter (
        .clk    (clk),
        .rst_n  (rst_n),
        .push   (wr_resp_push),
        .pop    (fire),
        .full   (wr_resp_full),
        .empty  (cnt_empty)
    );

    b_channel_logic u_b_chan (
        .empty        (cnt_empty),
        .fire         (fire),
        .s_axi_bvalid (s_axi_bvalid),
        .s_axi_bresp  (s_axi_bresp),
        .s_axi_bready (s_axi_bready)
    );

endmodule
