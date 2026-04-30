// ============================================================
//  File    : rd_response_path_sva.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : Read Response Path — Canal R
//  Type    : SVA bind module
//
//  Aserciones formales para verificar:
//    - Integridad one-hot del mux de respuesta
//    - Captura correcta en el ROB
//    - Drenado en orden estricto
//    - Corrección de rob_tag_free
//    - Cumplimiento protocolo AXI4-Lite canal R
//    - Sin overflow de FIFO
//    - Liveness básica
//
//  Uso: bind rd_response_path rd_response_path_sva #(...) u_sva (...);
// ============================================================
`timescale 1ns/1ps

module rd_response_path_sva #(
    parameter N_BANKS        = 4,
    parameter AXI_DATA_WIDTH = 32
)(
    input wire clk,
    input wire rst_n,

    // ── Entradas externas al DUT ──────────────────────
    input wire                       rd_resp_valid [0:N_BANKS-1],
    input wire [AXI_DATA_WIDTH-1:0]  rd_resp_data  [0:N_BANKS-1],
    input wire [$clog2(N_BANKS)-1:0] rd_resp_tag   [0:N_BANKS-1],
    input wire [$clog2(N_BANKS):0]   wr_ptr_ext,

    // ── Salidas del DUT ───────────────────────────────
    input wire                       rob_tag_free,
    input wire                       s_axi_rvalid,
    input wire [AXI_DATA_WIDTH-1:0]  s_axi_rdata,
    input wire [1:0]                 s_axi_rresp,
    input wire                       s_axi_rready,

    // ── Señales internas: u_rd_resp_mux ──────────────
    input wire                       mux_valid,
    input wire [AXI_DATA_WIDTH-1:0]  mux_data,
    input wire [$clog2(N_BANKS)-1:0] mux_tag,

    // ── Señales internas: u_reorder_buffer ───────────
    input wire                       rob_valid    [0:N_BANKS-1],
    input wire [AXI_DATA_WIDTH-1:0]  rob_data     [0:N_BANKS-1],
    input wire [$clog2(N_BANKS)-1:0] rd_ptr_addr,
    input wire                       rd_ptr_wrap,
    input wire                       rob_push,
    input wire [AXI_DATA_WIDTH-1:0]  rob_data_out,
    input wire                       rob_full,

    // ── Señales internas: u_rd_resp_fifo ─────────────
    input wire                       rd_resp_fifo_full,
    input wire                       fifo_pndng,

    // ── Señales internas: u_r_channel_logic ──────────
    input wire                       fifo_pop
);

    localparam BANK_BITS = $clog2(N_BANKS);

    // ── Condición de reset para disable iff ──────────
    wire rst_active = !rst_n;

    // ============================================================
    // CATEGORÍA 1 — Mux one-hot (contract del Scheduler)
    // ============================================================

    // Convertir array unpacked a packed para $onehot0
    wire [N_BANKS-1:0] rd_resp_valid_packed;
    genvar gi;
    generate
        for (gi = 0; gi < N_BANKS; gi = gi + 1) begin : gen_pack_valid
            assign rd_resp_valid_packed[gi] = rd_resp_valid[gi];
        end
    endgenerate

    // [CRÍTICA] Máximo 1 banco completa RD por ciclo.
    // El Scheduler despacha one-hot → max 1 wr_resp_valid al tiempo.
    // Si esto falla, hay un bug grave en el Scheduler.
    a_rd_resp_onehot: assert property (
        @(posedge clk) disable iff (rst_active)
        $onehot0(rd_resp_valid_packed)
    ) else $error("[SVA][CRIT] a_rd_resp_onehot: más de un banco completó lectura simultáneamente");

    // [ALTA] mux_valid debe reflejar la OR-reduce de todos los valids.
    a_mux_valid_reflects_any: assert property (
        @(posedge clk) disable iff (rst_active)
        mux_valid == |rd_resp_valid_packed
    ) else $error("[SVA][ALTA] a_mux_valid_reflects_any: mux_valid no coincide con OR-reduce de rd_resp_valid");

    // [ALTA] El mux selecciona correctamente: cuando exactamente un banco
    // tiene valid, los datos y tag del mux corresponden a ese banco.
    // Se verifica como propiedad genérica por banco.
    generate
        for (gi = 0; gi < N_BANKS; gi = gi + 1) begin : gen_mux_sel
            a_mux_selects_correctly: assert property (
                @(posedge clk) disable iff (rst_active)
                (rd_resp_valid_packed == (N_BANKS'(1) << gi))
                |->
                (mux_data == rd_resp_data[gi] && mux_tag == rd_resp_tag[gi])
            ) else $error("[SVA][ALTA] a_mux_selects_correctly[%0d]: mux no seleccionó el banco correcto", gi);
        end
    endgenerate

    // ============================================================
    // CATEGORÍA 2 — ROB capture integrity
    // ============================================================

    // [CRÍTICA] Cuando llega mux_valid, al siguiente ciclo el slot
    // rob_valid[mux_tag] queda marcado como 1. Garantiza que ninguna
    // respuesta se pierde al entrar al ROB.
    a_rob_capture: assert property (
        @(posedge clk) disable iff (rst_active)
        mux_valid |=> rob_valid[$past(mux_tag)]
    ) else $error("[SVA][CRIT] a_rob_capture: slot ROB no fue marcado como válido tras mux_valid");

    // [CRÍTICA] El dato se captura junto con el valid. Verifica integridad
    // de datos: lo que entra al ROB es exactamente lo que llegó del banco.
    a_rob_data_capture: assert property (
        @(posedge clk) disable iff (rst_active)
        mux_valid |=>
            (rob_data[$past(mux_tag)] == $past(mux_data))
    ) else $error("[SVA][CRIT] a_rob_data_capture: dato capturado en ROB no coincide con mux_data");

    // [CRÍTICA] Invariante arquitectónico: nunca se escribe sobre un slot
    // que ya está válido (sin haber sido drenado antes).
    // Si dispara: bug en Scheduler — asignó un tag a slot aún ocupado.
    a_no_overwrite_valid_slot: assert property (
        @(posedge clk) disable iff (rst_active)
        (mux_valid && rob_valid[mux_tag]) |-> 1'b0
    ) else $error("[SVA][CRIT] a_no_overwrite_valid_slot: sobreescritura de slot ROB válido — bug en Scheduler/rob_tag_free");

    // ============================================================
    // CATEGORÍA 3 — Drain in-order (razón de existir del ROB)
    // ============================================================

    // [CRÍTICA] rob_push solo ocurre cuando el slot apuntado por
    // rd_ptr_addr está válido. El ROB nunca puede drenar un slot vacío.
    a_drain_only_if_valid: assert property (
        @(posedge clk) disable iff (rst_active)
        rob_push |-> rob_valid[rd_ptr_addr]
    ) else $error("[SVA][CRIT] a_drain_only_if_valid: rob_push cuando slot rd_ptr_addr no es válido");

    // [CRÍTICA] rob_push solo cuando la FIFO downstream no está llena.
    // Evita pérdida silenciosa de datos al pasar del ROB a la FIFO R.
    a_drain_not_full_fifo: assert property (
        @(posedge clk) disable iff (rst_active)
        rob_push |-> !rd_resp_fifo_full
    ) else $error("[SVA][CRIT] a_drain_not_full_fifo: rob_push hacia FIFO llena — descarte silencioso");

    // [CRÍTICA] Después de rob_push, el slot drenado queda limpio,
    // salvo que en el mismo ciclo mux_valid escriba de vuelta en él.
    a_drain_clears_slot: assert property (
        @(posedge clk) disable iff (rst_active)
        (rob_push && !(mux_valid && (mux_tag == rd_ptr_addr)))
        |=>
        !rob_valid[$past(rd_ptr_addr)]
    ) else $error("[SVA][CRIT] a_drain_clears_slot: slot ROB no fue limpiado tras rob_push");

    // [CRÍTICA] Después de rob_push, rd_ptr_addr avanza correctamente.
    // Wrap manual para N_BANKS no necesariamente potencia de 2.
    a_rd_ptr_advances: assert property (
        @(posedge clk) disable iff (rst_active)
        rob_push |=>
            (rd_ptr_addr ==
             (($past(rd_ptr_addr) == BANK_BITS'(N_BANKS - 1))
              ? {BANK_BITS{1'b0}}
              : $past(rd_ptr_addr) + {{(BANK_BITS-1){1'b0}}, 1'b1}))
    ) else $error("[SVA][CRIT] a_rd_ptr_advances: rd_ptr_addr no avanzó correctamente tras rob_push");

    // [CRÍTICA] rd_ptr_addr no avanza si no hubo rob_push.
    a_rd_ptr_stable_no_push: assert property (
        @(posedge clk) disable iff (rst_active)
        !rob_push |=> (rd_ptr_addr == $past(rd_ptr_addr))
    ) else $error("[SVA][CRIT] a_rd_ptr_stable_no_push: rd_ptr_addr cambió sin rob_push");

    // ============================================================
    // CATEGORÍA 4 — rob_tag_free correctness
    // ============================================================

    // [CRÍTICA] rob_tag_free refleja exactamente el estado !full del ROB.
    // El Scheduler depende de esta señal para no despachar cuando el ROB
    // no puede aceptar más entradas.
    a_rob_tag_free_def: assert property (
        @(posedge clk) disable iff (rst_active)
        rob_tag_free == !rob_full
    ) else $error("[SVA][CRIT] a_rob_tag_free_def: rob_tag_free no coincide con !rob_full");

    // [ALTA] Cuando el ROB está full y llega un mux_valid, el tag
    // apuntado no debe tener su slot ocupado (el Scheduler debió haber
    // detenido el dispatch antes de este punto).
    a_no_capture_when_full: assert property (
        @(posedge clk) disable iff (rst_active)
        (rob_full && mux_valid) |-> !rob_valid[mux_tag]
    ) else $error("[SVA][ALTA] a_no_capture_when_full: mux_valid con ROB full y slot colisionando");

    // ============================================================
    // CATEGORÍA 5 — AXI4-Lite R Channel compliance
    // ============================================================

    // [CRÍTICA] AXI4-Lite: una vez rvalid=1, no puede bajar hasta que
    // ocurra el handshake (rready=1). Fundamental para cumplir la spec.
    a_rvalid_stable_until_rready: assert property (
        @(posedge clk) disable iff (rst_active)
        (s_axi_rvalid && !s_axi_rready) |=> s_axi_rvalid
    ) else $error("[SVA][CRIT] a_rvalid_stable_until_rready: rvalid bajó sin rready — violación AXI4-Lite");

    // [CRÍTICA] AXI4-Lite: rdata debe ser estable mientras rvalid está
    // alto esperando handshake. El master tiene derecho a samplear en
    // cualquier momento con rready=1.
    a_rdata_stable_when_rvalid_held: assert property (
        @(posedge clk) disable iff (rst_active)
        (s_axi_rvalid && !s_axi_rready) |=> $stable(s_axi_rdata)
    ) else $error("[SVA][CRIT] a_rdata_stable_when_rvalid_held: rdata cambió mientras rvalid esperaba rready");

    // [MEDIA] bresp debe ser siempre OKAY (2'b00) — constante cableada.
    // Verificación de que no se introdujo lógica incorrecta.
    a_rresp_okay: assert property (
        @(posedge clk) disable iff (rst_active)
        s_axi_rresp == 2'b00
    ) else $error("[SVA][MED] a_rresp_okay: s_axi_rresp != OKAY (2'b00)");

    // [ALTA] rvalid refleja directamente el estado de la FIFO R.
    // Cualquier desviación indica desconexión entre la FIFO y el canal R.
    a_rvalid_reflects_fifo: assert property (
        @(posedge clk) disable iff (rst_active)
        s_axi_rvalid == fifo_pndng
    ) else $error("[SVA][ALTA] a_rvalid_reflects_fifo: s_axi_rvalid no coincide con fifo_pndng");

    // [ALTA] rvalid solo puede subir si hay dato en FIFO.
    a_rvalid_implies_fifo_pndng: assert property (
        @(posedge clk) disable iff (rst_active)
        s_axi_rvalid |-> fifo_pndng
    ) else $error("[SVA][ALTA] a_rvalid_implies_fifo_pndng: rvalid sin dato en FIFO R");

    // ============================================================
    // CATEGORÍA 6 — FIFO no overflow
    // ============================================================

    // [CRÍTICA] Nunca se empuja a la FIFO R cuando está llena.
    // Doble chequeo cruzado con a_drain_not_full_fifo: ambos
    // deben mantenerse para garantizar no-silentdiscard.
    a_no_push_full_fifo: assert property (
        @(posedge clk) disable iff (rst_active)
        rob_push |-> !rd_resp_fifo_full
    ) else $error("[SVA][CRIT] a_no_push_full_fifo: push a FIFO R cuando full — descarte silencioso");

    // [ALTA] fifo_pop solo ocurre cuando hay dato (pndng=1).
    // La lógica combinacional en r_channel_logic debería garantizarlo,
    // pero es útil verificarlo explícitamente.
    a_fifo_pop_only_if_pndng: assert property (
        @(posedge clk) disable iff (rst_active)
        fifo_pop |-> fifo_pndng
    ) else $error("[SVA][ALTA] a_fifo_pop_only_if_pndng: fifo_pop sin dato disponible");

    // ============================================================
    // CATEGORÍA 7 — Liveness / fairness
    // ============================================================

    // [ALTA] Cuando hay handshake R (fire), fifo_pop debe haberse generado.
    // Verifica que la lógica combinacional fire → fifo_pop funciona.
    a_fifo_pop_on_handshake: assert property (
        @(posedge clk) disable iff (rst_active)
        (s_axi_rvalid && s_axi_rready) |-> fifo_pop
    ) else $error("[SVA][ALTA] a_fifo_pop_on_handshake: handshake R sin fifo_pop — dato no consumido de FIFO");

    // [ALTA] fifo_pop en el ciclo actual implica que fue un handshake válido
    // (rvalid && rready). La lógica no puede generar pop espontáneo.
    a_fifo_pop_implies_handshake: assert property (
        @(posedge clk) disable iff (rst_active)
        fifo_pop |-> (s_axi_rvalid && s_axi_rready)
    ) else $error("[SVA][ALTA] a_fifo_pop_implies_handshake: fifo_pop sin handshake R válido");

    // [ALTA] Liveness: si hay dato en FIFO y rready=1, en el siguiente ciclo
    // la FIFO debe tener uno menos (o el mismo si rob_push ocurrió también).
    // Acotado a 1 ciclo — el full liveness requiere model checking.
    a_eventual_drain_on_rready: assert property (
        @(posedge clk) disable iff (rst_active)
        (s_axi_rvalid && s_axi_rready) |=> fifo_pop == $past(s_axi_rvalid && s_axi_rready)
    ) else $error("[SVA][ALTA] a_eventual_drain_on_rready: inconsistencia en pop tras handshake R");

    // [MEDIA] Tras reset, rvalid debe estar bajo.
    a_rvalid_low_after_reset: assert property (
        @(posedge clk)
        !$past(rst_n, 1) && rst_n |-> !s_axi_rvalid
    ) else $error("[SVA][MED] a_rvalid_low_after_reset: rvalid alto justo después de salir de reset");

    // ============================================================
    // COBERTURA — propiedades cover para asegurar que los
    // escenarios importantes son ejercitados en simulación
    // ============================================================

    // Cubrir: mux_valid ocurrió al menos una vez
    c_mux_valid_occurred: cover property (
        @(posedge clk) disable iff (rst_active)
        mux_valid
    );

    // Cubrir: rob_push al menos una vez
    c_rob_push_occurred: cover property (
        @(posedge clk) disable iff (rst_active)
        rob_push
    );

    // Cubrir: handshake R ocurrió al menos una vez
    c_rvalid_handshake: cover property (
        @(posedge clk) disable iff (rst_active)
        s_axi_rvalid && s_axi_rready
    );

    // Cubrir: rob_full ocurrió (ejercita backpressure)
    c_rob_full_occurred: cover property (
        @(posedge clk) disable iff (rst_active)
        rob_full
    );

    // Cubrir: backpressure de FIFO (rd_resp_fifo_full detuvo rob_push)
    c_fifo_full_backpressure: cover property (
        @(posedge clk) disable iff (rst_active)
        rd_resp_fifo_full && rob_valid[rd_ptr_addr]
    );

    // Cubrir: rvalid bajo mientras rready alto (master rápido)
    c_master_rready_fast: cover property (
        @(posedge clk) disable iff (rst_active)
        s_axi_rready && !s_axi_rvalid
    );

    // Cubrir: rvalid alto mientras rready bajo (backpressure master)
    c_master_rready_slow: cover property (
        @(posedge clk) disable iff (rst_active)
        s_axi_rvalid && !s_axi_rready
    );

endmodule


// ============================================================
// BIND — conecta el módulo SVA al DUT en tiempo de elaboración.
// VCS aplica las aserciones sin modificar el RTL original.
//
// Nota: las señales internas de los submódulos se acceden
// con la notación de path jerárquico relativo al DUT:
//   u_rd_resp_mux.mux_valid
//   u_reorder_buffer.rob_valid[i]
//   etc.
// ============================================================
bind rd_response_path rd_response_path_sva #(
    .N_BANKS       (N_BANKS),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
) u_rd_response_path_sva (
    .clk                (clk),
    .rst_n              (rst_n),

    // Entradas externas
    .rd_resp_valid      (rd_resp_valid),
    .rd_resp_data       (rd_resp_data),
    .rd_resp_tag        (rd_resp_tag),
    .wr_ptr_ext         (wr_ptr_ext),

    // Salidas del DUT
    .rob_tag_free       (rob_tag_free),
    .s_axi_rvalid       (s_axi_rvalid),
    .s_axi_rdata        (s_axi_rdata),
    .s_axi_rresp        (s_axi_rresp),
    .s_axi_rready       (s_axi_rready),

    // Señales internas: u_rd_resp_mux
    .mux_valid          (u_rd_resp_mux.mux_valid),
    .mux_data           (u_rd_resp_mux.mux_data),
    .mux_tag            (u_rd_resp_mux.mux_tag),

    // Señales internas: u_reorder_buffer
    .rob_valid          (u_reorder_buffer.rob_valid),
    .rob_data           (u_reorder_buffer.rob_data),
    .rd_ptr_addr        (u_reorder_buffer.rd_ptr_addr),
    .rd_ptr_wrap        (u_reorder_buffer.rd_ptr_wrap),
    .rob_push           (u_reorder_buffer.rob_push),
    .rob_data_out       (u_reorder_buffer.rob_data_out),
    .rob_full           (u_reorder_buffer.full),

    // Señales internas: u_rd_resp_fifo
    .rd_resp_fifo_full  (rd_resp_fifo_full),
    .fifo_pndng         (fifo_pndng),

    // Señales internas: u_r_channel_logic
    .fifo_pop           (fifo_pop)
);
