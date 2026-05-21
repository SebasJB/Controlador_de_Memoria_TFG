// ============================================================
//  File    : wr_response_path_sva.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : Write Response Path (B Channel AXI4-Lite)
//  Type    : SVA bind module — aserciones formales y de simulación
//
//  Descripción:
//    Módulo de aserciones para wr_response_path. Se adjunta vía
//    `bind` sin modificar el RTL original. Referencia señales
//    internas de los submódulos u_collector, u_counter y u_b_chan
//    usando jerarquía relativa.
//
//    Las aserciones están organizadas en 7 categorías, cada una
//    anotada con nivel de criticidad (CRÍTICA / ALTA / MEDIA) y
//    descripción del invariante que protege.
//
//  Compatibilidad: Synopsys VCS, SystemVerilog-2012.
//  Reset: síncrono activo en bajo (rst_n). Las aserciones se
//         deshabilitan durante reset con `disable iff (!rst_n)`.
// ============================================================

`timescale 1ns/1ps

module wr_response_path_sva #(
    parameter N_BANKS       = 4,
    parameter WR_FIFO_DEPTH = 8
)(
    // ── Reloj y reset ─────────────────────────────────────
    input wire clk,
    input wire rst_n,

    // ── Entradas del DUT (banco) ──────────────────────────
    input wire wr_resp_valid [0:N_BANKS-1],

    // ── Salidas AXI4-Lite Canal B ─────────────────────────
    input wire       s_axi_bvalid,
    input wire [1:0] s_axi_bresp,
    input wire       s_axi_bready,

    // ── Backpressure ──────────────────────────────────────
    input wire wr_resp_full,

    // ── Señales internas u_collector ─────────────────────
    input wire wr_resp_push,          // u_collector.wr_resp_push

    // ── Señales internas u_counter ────────────────────────
    // CNT_WIDTH = $clog2(WR_FIFO_DEPTH+1) — se parametriza aquí
    input wire [$clog2(WR_FIFO_DEPTH+1)-1:0] cnt,   // u_counter.cnt
    input wire push_ok,               // u_counter.push_ok
    input wire pop_ok,                // u_counter.pop_ok
    input wire cnt_full,              // u_counter.full  (alias local)
    input wire cnt_empty              // u_counter.empty (alias local)
);

    // ================================================================
    // Construcción de packed vector desde wr_resp_valid[] unpacked.
    // $onehot0 no acepta arrays unpacked en todos los simuladores.
    // Se construye un vector packed con un generate para garantizar
    // compatibilidad VCS / JasperGold.
    // ================================================================
    localparam CNT_WIDTH = $clog2(WR_FIFO_DEPTH + 1);

    genvar gi;
    wire [N_BANKS-1:0] wr_resp_valid_packed;
    generate
        for (gi = 0; gi < N_BANKS; gi = gi + 1) begin : gen_pack
            assign wr_resp_valid_packed[gi] = wr_resp_valid[gi];
        end
    endgenerate


    // ================================================================
    // CATEGORÍA 1 — Collector one-hot (contract del Scheduler)
    // ================================================================

    // [CRÍTICA] a_wr_resp_onehot
    // Invariante: el Scheduler garantiza despacho one-hot → máximo
    // 1 banco puede completar una escritura por ciclo. Si más de uno
    // aparece simultáneamente, el contador recibe múltiples pushes
    // en un mismo ciclo y su semántica de "1 push máximo/ciclo" se
    // viola, pudiendo causar pérdida de respuestas (silent drop).
    a_wr_resp_onehot: assert property (
        @(posedge clk) disable iff (!rst_n)
        $onehot0(wr_resp_valid_packed)
    ) else $error("[CRITICA] a_wr_resp_onehot: mas de un banco con wr_resp_valid=1 simultaneamente");

    // [ALTA] a_push_reflects_any
    // Invariante: el OR-reduce del collector debe coincidir exactamente
    // con el OR lógico de todas las entradas. Detecta errores de cableado
    // o síntesis incorrecta en el bloque combinacional u_collector.
    a_push_reflects_any: assert property (
        @(posedge clk) disable iff (!rst_n)
        (wr_resp_push == |wr_resp_valid_packed)
    ) else $error("[ALTA] a_push_reflects_any: wr_resp_push no coincide con OR de wr_resp_valid");


    // ================================================================
    // CATEGORÍA 2 — Counter integrity (no overflow, no underflow)
    // ================================================================

    // [CRÍTICA] a_no_pop_empty
    // Invariante: jamás se decrementa el contador cuando está vacío.
    // Un underflow corrompería cnt a su valor máximo (wrap-around),
    // causando que wr_resp_full se activara incorrectamente y
    // bloqueara el Scheduler sin razón.
    a_no_pop_empty: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(pop_ok && cnt_empty)
    ) else $error("[CRITICA] a_no_pop_empty: pop_ok=1 con cnt_empty=1 — underflow");

    // [CRÍTICA] a_no_push_full_effective
    // Invariante: push_ok nunca puede ser 1 simultáneamente con full=1
    // (es invariante por construcción de push_ok = push & !full, pero
    // se aserta explícitamente para detectar errores de síntesis o
    // modificaciones incorrectas al RTL).
    a_no_push_full_effective: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(push_ok && cnt_full)
    ) else $error("[CRITICA] a_no_push_full_effective: push_ok=1 con cnt_full=1 — imposible por diseño");

    // [CRÍTICA] a_cnt_range
    // Invariante: el contador nunca supera WR_FIFO_DEPTH. Un valor
    // fuera de rango indicaría overflow aritmético o error en los
    // guards de push_ok/pop_ok.
    a_cnt_range: assert property (
        @(posedge clk) disable iff (!rst_n)
        (cnt <= CNT_WIDTH'(WR_FIFO_DEPTH))
    ) else $error("[CRITICA] a_cnt_range: cnt=%0d supera WR_FIFO_DEPTH=%0d", cnt, WR_FIFO_DEPTH);


    // ================================================================
    // CATEGORÍA 3 — No-silent-drop (principio rector del diseño)
    // ================================================================

    // [CRÍTICA] a_no_silent_drop_on_push_only
    // Invariante: cada push efectivo sin pop simultáneo incrementa el
    // contador exactamente en 1. Si el contador no sube, la respuesta
    // se pierde silenciosamente — violación directa del principio rector.
    a_no_silent_drop_on_push_only: assert property (
        @(posedge clk) disable iff (!rst_n)
        (push_ok && !pop_ok)
            |=> (cnt == $past(cnt) + CNT_WIDTH'(1))
    ) else $error("[CRITICA] a_no_silent_drop_on_push_only: push sin pop no incremento el contador");

    // [CRÍTICA] a_push_full_but_valid
    // Invariante: el Scheduler NUNCA debe generar wr_resp_push cuando
    // wr_resp_full=1. Si esto ocurre, la respuesta se descarta
    // silenciosamente porque push_ok=0 (full bloquea). Esta aserción
    // actúa como trampa de integración: falla siempre que se dispara.
    //
    // TODO: depende de integración Scheduler-wr_resp_full.
    // Activar solo cuando el top-level conecte wr_resp_full al
    // Scheduler como backpressure real. En un TB de bloque aislado
    // este test puede dispararse intencionalmente (ver T4 en el TB).
    a_push_full_but_valid: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(wr_resp_push && cnt_full)
    ) else $warning("[CRITICA-INTEGRACION] a_push_full_but_valid: wr_resp_push=1 con cnt_full=1 — Scheduler ignorando backpressure");


    // ================================================================
    // CATEGORÍA 4 — Decremento consistente
    // ================================================================

    // [ALTA] a_decrement_on_pop_only
    // Invariante: pop efectivo sin push simultáneo decrementa el
    // contador exactamente en 1. Detecta errores de lógica en los
    // guards del always @(posedge clk) del pending_counter.
    a_decrement_on_pop_only: assert property (
        @(posedge clk) disable iff (!rst_n)
        (!push_ok && pop_ok)
            |=> (cnt == $past(cnt) - CNT_WIDTH'(1))
    ) else $error("[ALTA] a_decrement_on_pop_only: pop sin push no decremento el contador");

    // [ALTA] a_no_change_on_cancel
    // Invariante: push y pop simultáneos se cancelan — el contador no
    // debe cambiar. Esto es correcto porque la respuesta "entra y sale"
    // en el mismo ciclo; bvalid ya estaba alto y el master recibe su
    // handshake sin pasar por el estado intermedio del contador.
    a_no_change_on_cancel: assert property (
        @(posedge clk) disable iff (!rst_n)
        (push_ok && pop_ok)
            |=> (cnt == $past(cnt))
    ) else $error("[ALTA] a_no_change_on_cancel: push+pop simultaneos cambiaron el contador");

    // [MEDIA] a_no_change_on_idle
    // Invariante: sin actividad (ni push ni pop efectivos), el contador
    // permanece constante. Detecta glitches en la lógica secuencial.
    a_no_change_on_idle: assert property (
        @(posedge clk) disable iff (!rst_n)
        (!push_ok && !pop_ok)
            |=> (cnt == $past(cnt))
    ) else $error("[MEDIA] a_no_change_on_idle: cnt cambio sin push ni pop");


    // ================================================================
    // CATEGORÍA 5 — B Channel output consistency
    // ================================================================

    // [ALTA] a_bvalid_reflects_empty
    // Invariante: bvalid es exactamente !empty. Si bvalid pudiera
    // estar bajo con respuestas pendientes, el master nunca las vería
    // (silent stall). Si bvalid estuviera alto con contador vacío,
    // el master recibiría handshakes fantasma.
    a_bvalid_reflects_empty: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s_axi_bvalid == !cnt_empty)
    ) else $error("[ALTA] a_bvalid_reflects_empty: s_axi_bvalid no coincide con !cnt_empty");

    // [MEDIA] a_bresp_okay
    // Invariante: bresp es siempre 2'b00 (OKAY). En AXI4-Lite un
    // controlador de SRAM sin protección de acceso no genera SLVERR
    // ni DECERR. Detecta corrupciones accidentales del hardwire.
    a_bresp_okay: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s_axi_bresp == 2'b00)
    ) else $error("[MEDIA] a_bresp_okay: s_axi_bresp != 2'b00 (OKAY)");

    // [ALTA] a_full_reflects
    // Invariante: wr_resp_full debe ser exactamente (cnt == WR_FIFO_DEPTH).
    // El Scheduler usa esta señal para su backpressure; cualquier
    // discrepancia puede causar despacho de escrituras con contador lleno.
    a_full_reflects: assert property (
        @(posedge clk) disable iff (!rst_n)
        (wr_resp_full == cnt_full)
    ) else $error("[ALTA] a_full_reflects: wr_resp_full no coincide con (cnt==WR_FIFO_DEPTH)");


    // ================================================================
    // CATEGORÍA 6 — AXI4-Lite B Channel compliance
    // ================================================================

    // [CRÍTICA] a_bvalid_stable_until_bready
    // Invariante AXI4-Lite §A3.2.1: una vez bvalid=1, no puede bajar
    // hasta que bready=1 complete el handshake. Si bvalid bajara antes,
    // el master podría perder la respuesta o entrar en deadlock.
    a_bvalid_stable_until_bready: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s_axi_bvalid && !s_axi_bready) |=> s_axi_bvalid
    ) else $error("[CRITICA] a_bvalid_stable_until_bready: bvalid bajo antes del handshake bready");

    // [MEDIA] a_bresp_stable_when_bvalid
    // Invariante AXI4-Lite §A3.2.2: los campos de respuesta (bresp) deben
    // mantenerse estables mientras bvalid=1 hasta el handshake. Aunque
    // bresp es constante aquí, esta aserción protege contra modificaciones
    // futuras que rompan la estabilidad.
    a_bresp_stable_when_bvalid: assert property (
        @(posedge clk) disable iff (!rst_n)
        s_axi_bvalid |-> (s_axi_bresp == 2'b00)
    ) else $error("[MEDIA] a_bresp_stable_when_bvalid: bresp != OKAY con bvalid activo");


    // ================================================================
    // CATEGORÍA 7 — Fairness / liveness
    // ================================================================

    // [ALTA] a_eventual_drain
    // Invariante de liveness acotada: si hay un handshake exitoso
    // (bvalid=1 && bready=1) sin push simultáneo, el contador debe
    // decrementar en el siguiente ciclo. Detecta situaciones donde el
    // pop no se propaga correctamente al contador.
    a_eventual_drain: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s_axi_bvalid && s_axi_bready && !push_ok)
            |=> (cnt == $past(cnt) - CNT_WIDTH'(1))
    ) else $error("[ALTA] a_eventual_drain: handshake B exitoso no decremento el contador");

endmodule


// ================================================================
// BIND — adjunta el módulo SVA al DUT sin modificar el RTL.
// Se conectan tanto las señales del puerto externo como las
// señales internas de los submódulos vía jerarquía relativa.
// ================================================================
bind wr_response_path wr_response_path_sva #(
    .N_BANKS       (N_BANKS),
    .WR_FIFO_DEPTH (WR_FIFO_DEPTH)
) u_wr_response_path_sva (
    // Reloj y reset
    .clk            (clk),
    .rst_n          (rst_n),

    // Entradas del DUT
    .wr_resp_valid  (wr_resp_valid),

    // Salidas AXI4-Lite Canal B
    .s_axi_bvalid   (s_axi_bvalid),
    .s_axi_bresp    (s_axi_bresp),
    .s_axi_bready   (s_axi_bready),

    // Backpressure
    .wr_resp_full   (wr_resp_full),

    // Señales internas u_collector
    .wr_resp_push   (u_collector.wr_resp_push),

    // Señales internas u_counter
    .cnt            (u_counter.cnt),
    .push_ok        (u_counter.push_ok),
    .pop_ok         (u_counter.pop_ok),
    .cnt_full       (u_counter.full),
    .cnt_empty      (u_counter.empty)
);
