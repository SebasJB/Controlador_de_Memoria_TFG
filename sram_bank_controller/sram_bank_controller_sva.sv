// ============================================================
//  File    : sram_bank_controller_sva.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : SRAM Bank Controller
//  Type    : Formal/Simulation Assertions (SVA)
//
//  Descripción:
//    Aserciones concurrentes SVA para verificar el módulo
//    sram_bank_controller. Se vinculan vía `bind` sin tocar
//    el RTL original. Cubren FSM, salidas Moore, mutex de
//    respuestas, captura controlada, gating de SRAM y
//    preservación de tags.
//
//  Uso:
//    El bloque `bind` al final de este archivo conecta
//    automáticamente el módulo de aserciones al DUT durante
//    elaboración. No se requiere modificar el RTL.
//
//  Criticidad: CRÍTICA > ALTA > MEDIA
//  Clocking  : @(posedge clk)
//  Reset     : disable iff (!rst_n) — síncrono activo en bajo
// ============================================================

module sram_bank_controller_sva #(
    parameter N_BANKS         = 4,
    parameter READ_LATENCY    = 1,
    parameter AXI_DATA_WIDTH  = 32,
    parameter BANK_ADDR_WIDTH = 10,
    parameter LAT_CNT_W       = 8
)(
    // ── Reloj y reset ─────────────────────────────────────
    input wire clk,
    input wire rst_n,

    // ── Entradas del Scheduler ────────────────────────────
    input wire                        bank_req_valid,
    input wire                        bank_req_op,
    input wire [BANK_ADDR_WIDTH-1:0]  bank_req_addr,
    input wire [AXI_DATA_WIDTH-1:0]   bank_req_wdata,
    input wire [AXI_DATA_WIDTH/8-1:0] bank_req_wstrb,
    input wire [$clog2(N_BANKS)-1:0]  bank_req_tag,

    // ── Salidas hacia SRAM IP ─────────────────────────────
    input wire                        sram_en,
    input wire                        sram_we,
    input wire [BANK_ADDR_WIDTH-1:0]  sram_addr,
    input wire [AXI_DATA_WIDTH-1:0]   sram_din,
    input wire [AXI_DATA_WIDTH/8-1:0] sram_wstrb,

    // ── Salidas hacia Scheduler ───────────────────────────
    input wire bank_busy,

    // ── Salidas hacia WR Resp Collector ───────────────────
    input wire wr_resp_valid,

    // ── Salidas hacia RD Resp Mux ─────────────────────────
    input wire [AXI_DATA_WIDTH-1:0]  rd_resp_data,
    input wire [$clog2(N_BANKS)-1:0] rd_resp_tag,
    input wire                       rd_resp_valid,

    // ── Señales internas del DUT (via bind) ───────────────
    // Desde u_fsm (bank_fsm)
    input wire [1:0] u_fsm_state,
    input wire       u_fsm_fsm_idle,
    input wire       u_fsm_issue_en,
    input wire       u_fsm_rd_capture_en,

    // Desde u_capture (request_capture_reg)
    input wire       u_capture_is_wr,
    input wire       u_capture_is_rd,
    input wire       u_capture_capture_en,
    input wire [BANK_ADDR_WIDTH-1:0] u_capture_lat_addr,
    input wire [$clog2(N_BANKS)-1:0] u_capture_lat_tag
);

    // ── Codificación de estados (debe coincidir con bank_fsm) ─
    localparam [1:0]
        IDLE     = 2'b00,
        ISSUE    = 2'b01,
        WAIT     = 2'b10,
        COMPLETE = 2'b11;

    // ==========================================================
    // CATEGORÍA 1 — FSM bien formado
    // ==========================================================

    // [ALTA] a_state_valid
    // El estado del FSM solo puede tomar los 4 valores legales.
    // Un estado inválido indica corrupción de flip-flops o
    // problema de síntesis.
    a_state_valid: assert property (
        @(posedge clk) disable iff (!rst_n)
        u_fsm_state inside {IDLE, ISSUE, WAIT, COMPLETE}
    ) else $error("[SVA ALTA] a_state_valid: estado ilegal = %b", u_fsm_state);

    // ==========================================================
    // CATEGORÍA 2 — Salidas Moore del FSM
    // ==========================================================

    // [CRÍTICA] a_fsm_idle_equiv
    // fsm_idle debe ser exactamente HIGH cuando y solo cuando
    // el FSM está en IDLE. Cualquier divergencia indica
    // decodificación de estado incorrecta.
    a_fsm_idle_equiv: assert property (
        @(posedge clk) disable iff (!rst_n)
        u_fsm_fsm_idle == (u_fsm_state == IDLE)
    ) else $error("[SVA CRÍTICA] a_fsm_idle_equiv: fsm_idle=%b pero state=%b", u_fsm_fsm_idle, u_fsm_state);

    // [CRÍTICA] a_bank_busy_conservative
    // bank_busy es HIGH en ISSUE y WAIT, pero NO en COMPLETE.
    // Esto es la "liberación conservadora": el banco se libera
    // solo cuando sram_dout ya está estable (COMPLETE→IDLE),
    // evitando hazards WAR/WAW. Si bank_busy subiera en
    // COMPLETE, el Scheduler podría despachar otro request
    // antes de que el dato esté disponible para el ROB.
    a_bank_busy_conservative: assert property (
        @(posedge clk) disable iff (!rst_n)
        bank_busy == ((u_fsm_state == ISSUE) || (u_fsm_state == WAIT))
    ) else $error("[SVA CRÍTICA] a_bank_busy_conservative: bank_busy=%b pero state=%b", bank_busy, u_fsm_state);

    // [CRÍTICA] a_wr_resp_valid_when
    // wr_resp_valid debe pulsar en COMPLETE si y solo si la
    // operación latched era escritura. Un pulso espurio podría
    // generar respuestas fantasmas en el canal B.
    a_wr_resp_valid_when: assert property (
        @(posedge clk) disable iff (!rst_n)
        wr_resp_valid == ((u_fsm_state == COMPLETE) && u_capture_is_wr)
    ) else $error("[SVA CRÍTICA] a_wr_resp_valid_when: wr_resp_valid=%b state=%b is_wr=%b",
                  wr_resp_valid, u_fsm_state, u_capture_is_wr);

    // [CRÍTICA] a_rd_resp_valid_when
    // rd_resp_valid debe pulsar en COMPLETE si y solo si la
    // operación latched era lectura. Garantiza que el RD Resp
    // Mux recibe exactamente un pulso por lectura completada.
    a_rd_resp_valid_when: assert property (
        @(posedge clk) disable iff (!rst_n)
        rd_resp_valid == ((u_fsm_state == COMPLETE) && u_capture_is_rd)
    ) else $error("[SVA CRÍTICA] a_rd_resp_valid_when: rd_resp_valid=%b state=%b is_rd=%b",
                  rd_resp_valid, u_fsm_state, u_capture_is_rd);

    // ==========================================================
    // CATEGORÍA 3 — Mutex de respuestas
    // ==========================================================

    // [CRÍTICA] a_resp_mutex
    // Es imposible que un banco complete simultáneamente una
    // escritura y una lectura — solo puede tener una operación
    // latched a la vez. Si ambos valids suben juntos hay un
    // error fundamental en el FSM o en la lógica Moore.
    a_resp_mutex: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(wr_resp_valid && rd_resp_valid)
    ) else $error("[SVA CRÍTICA] a_resp_mutex: wr_resp_valid y rd_resp_valid simultáneos");

    // ==========================================================
    // CATEGORÍA 4 — Captura controlada
    // ==========================================================

    // [ALTA] a_capture_only_idle
    // El capture reg solo debe latchar en IDLE. Si capture_en
    // sube mientras el FSM está en otro estado, podría
    // sobrescribir una operación en curso.
    a_capture_only_idle: assert property (
        @(posedge clk) disable iff (!rst_n)
        u_capture_capture_en |-> u_fsm_fsm_idle
    ) else $error("[SVA ALTA] a_capture_only_idle: capture_en=1 pero fsm_idle=0 (state=%b)", u_fsm_state);

    // [ALTA] a_capture_implies_req
    // capture_en solo puede subir si hay un request válido.
    // Es la condición de habilitación: capture_en = valid & idle.
    a_capture_implies_req: assert property (
        @(posedge clk) disable iff (!rst_n)
        u_capture_capture_en |-> bank_req_valid
    ) else $error("[SVA ALTA] a_capture_implies_req: capture_en=1 pero bank_req_valid=0");

    // ==========================================================
    // CATEGORÍA 5 — Transiciones legales del FSM
    // ==========================================================

    // [ALTA] a_complete_to_idle
    // COMPLETE siempre dura exactamente 1 ciclo: en el
    // siguiente ciclo el FSM debe estar en IDLE. Este
    // comportamiento es fundamental para que bank_busy baje
    // de forma predecible y el Scheduler pueda reutilizar
    // el banco.
    a_complete_to_idle: assert property (
        @(posedge clk) disable iff (!rst_n)
        (u_fsm_state == COMPLETE) |=> (u_fsm_state == IDLE)
    ) else $error("[SVA ALTA] a_complete_to_idle: COMPLETE no fue seguido por IDLE");

    // [ALTA] a_issue_entry
    // El FSM solo puede entrar a ISSUE desde IDLE con un
    // request válido. Verifica que no hay saltos de estado
    // ilegales.
    a_issue_entry: assert property (
        @(posedge clk) disable iff (!rst_n)
        $rose(u_fsm_state == ISSUE) |->
            $past(u_fsm_state == IDLE) && $past(bank_req_valid)
    ) else $error("[SVA ALTA] a_issue_entry: ISSUE iniciado desde estado ilegal o sin req_valid");

    // [ALTA] a_wait_only_if_rd
    // El estado WAIT solo es alcanzable en lecturas con
    // READ_LATENCY > 1. Un write nunca debe pasar por WAIT.
    a_wait_only_if_rd: assert property (
        @(posedge clk) disable iff (!rst_n)
        (u_fsm_state == WAIT) |-> u_capture_is_rd
    ) else $error("[SVA ALTA] a_wait_only_if_rd: FSM en WAIT pero is_rd=0");

    // [ALTA] a_no_wait_lat1
    // Para READ_LATENCY==1 la transición es ISSUE→COMPLETE
    // directo; WAIT nunca debe aparecer.
    generate
        if (READ_LATENCY == 1) begin : gen_no_wait_lat1
            a_no_wait_lat1: assert property (
                @(posedge clk) disable iff (!rst_n)
                u_fsm_state != WAIT
            ) else $error("[SVA ALTA] a_no_wait_lat1: READ_LATENCY=1 pero FSM entró a WAIT");
        end
    endgenerate

    // [ALTA] a_idle_stable_no_req
    // En IDLE, si no hay request válido, el FSM debe
    // permanecer en IDLE. Evita transiciones espontáneas.
    a_idle_stable_no_req: assert property (
        @(posedge clk) disable iff (!rst_n)
        ((u_fsm_state == IDLE) && !bank_req_valid) |=> (u_fsm_state == IDLE)
    ) else $error("[SVA ALTA] a_idle_stable_no_req: FSM salió de IDLE sin bank_req_valid");

    // ==========================================================
    // CATEGORÍA 6 — SRAM signals gating
    // ==========================================================

    // [ALTA] a_sram_en_in_issue
    // sram_en solo puede estar activo en estado ISSUE.
    // Cualquier habilitación fuera de ISSUE es un acceso
    // indeseado a la memoria que podría corromper datos.
    a_sram_en_in_issue: assert property (
        @(posedge clk) disable iff (!rst_n)
        sram_en |-> (u_fsm_state == ISSUE)
    ) else $error("[SVA ALTA] a_sram_en_in_issue: sram_en=1 fuera de ISSUE (state=%b)", u_fsm_state);

    // [ALTA] a_sram_we_matches_op
    // Cuando la SRAM está habilitada, sram_we debe reflejar
    // exactamente el tipo de operación latched. Un mismatch
    // podría causar escrituras accidentales en lecturas o
    // no-writes en escrituras.
    a_sram_we_matches_op: assert property (
        @(posedge clk) disable iff (!rst_n)
        sram_en |-> (sram_we == u_capture_is_wr)
    ) else $error("[SVA ALTA] a_sram_we_matches_op: sram_we=%b != is_wr=%b con sram_en=1",
                  sram_we, u_capture_is_wr);

    // [MEDIA] a_sram_addr_stable_in_issue
    // La dirección enviada a la SRAM en ISSUE debe coincidir
    // con la dirección latched. Detecta problemas de muxeo
    // o paths combinacionales erróneos.
    a_sram_addr_stable_in_issue: assert property (
        @(posedge clk) disable iff (!rst_n)
        (u_fsm_state == ISSUE) |-> (sram_addr == u_capture_lat_addr)
    ) else $error("[SVA MEDIA] a_sram_addr_stable_in_issue: sram_addr=%h != lat_addr=%h",
                  sram_addr, u_capture_lat_addr);

    // ==========================================================
    // CATEGORÍA 7 — RD resp integrity
    // ==========================================================

    // [CRÍTICA] a_rd_resp_tag_matches
    // El tag que sale hacia el ROB debe ser el mismo que se
    // latcheó al inicio de la transacción. Un tag incorrecto
    // causaría reordenamiento erróneo y datos incorrectos
    // en el canal R del master AXI.
    a_rd_resp_tag_matches: assert property (
        @(posedge clk) disable iff (!rst_n)
        rd_resp_valid |-> (rd_resp_tag == u_capture_lat_tag)
    ) else $error("[SVA CRÍTICA] a_rd_resp_tag_matches: rd_resp_tag=%0d != lat_tag=%0d",
                  rd_resp_tag, u_capture_lat_tag);

    // ==========================================================
    // CATEGORÍA 8 — No spurious responses
    // ==========================================================

    // [CRÍTICA] a_no_resp_in_idle
    // En IDLE no puede haber ninguna respuesta activa.
    // Pulsos espurios en IDLE generarían transacciones
    // fantasmas en los canales B y R.
    a_no_resp_in_idle: assert property (
        @(posedge clk) disable iff (!rst_n)
        (u_fsm_state == IDLE) |-> (!wr_resp_valid && !rd_resp_valid)
    ) else $error("[SVA CRÍTICA] a_no_resp_in_idle: respuesta activa en IDLE (wr=%b rd=%b)",
                  wr_resp_valid, rd_resp_valid);

    // [ALTA] a_no_sram_en_in_idle
    // La SRAM no debe estar habilitada cuando el banco está
    // libre. Complementa a_sram_en_in_issue con un check
    // explícito en IDLE para mayor cobertura.
    a_no_sram_en_in_idle: assert property (
        @(posedge clk) disable iff (!rst_n)
        (u_fsm_state == IDLE) |-> !sram_en
    ) else $error("[SVA ALTA] a_no_sram_en_in_idle: sram_en=1 en IDLE");

    // [ALTA] a_no_sram_en_in_complete
    // Tampoco debe haber acceso a SRAM en COMPLETE —
    // ese ciclo es solo para liberar bank_busy y propagar
    // la respuesta.
    a_no_sram_en_in_complete: assert property (
        @(posedge clk) disable iff (!rst_n)
        (u_fsm_state == COMPLETE) |-> !sram_en
    ) else $error("[SVA ALTA] a_no_sram_en_in_complete: sram_en=1 en COMPLETE");

    // [ALTA] a_no_resp_in_issue
    // En ISSUE el controlador está enviando el comando a la
    // SRAM; la respuesta no debe estar activa todavía.
    a_no_resp_in_issue: assert property (
        @(posedge clk) disable iff (!rst_n)
        (u_fsm_state == ISSUE) |-> (!wr_resp_valid && !rd_resp_valid)
    ) else $error("[SVA ALTA] a_no_resp_in_issue: respuesta activa en ISSUE (wr=%b rd=%b)",
                  wr_resp_valid, rd_resp_valid);

    // [ALTA] a_no_resp_in_wait
    // En WAIT la SRAM está procesando; aún no hay dato estable.
    a_no_resp_in_wait: assert property (
        @(posedge clk) disable iff (!rst_n)
        (u_fsm_state == WAIT) |-> (!wr_resp_valid && !rd_resp_valid)
    ) else $error("[SVA ALTA] a_no_resp_in_wait: respuesta activa en WAIT (wr=%b rd=%b)",
                  wr_resp_valid, rd_resp_valid);

    // ==========================================================
    // CATEGORÍA 9 — Propiedades de liveness (coverpoints)
    // ==========================================================
    // Coverpoints para verificar que los caminos críticos
    // son alcanzables durante simulación.

    cp_write_completes: cover property (
        @(posedge clk) disable iff (!rst_n)
        wr_resp_valid
    );

    cp_read_completes: cover property (
        @(posedge clk) disable iff (!rst_n)
        rd_resp_valid
    );

    cp_wait_state_reached: cover property (
        @(posedge clk) disable iff (!rst_n)
        u_fsm_state == WAIT
    );

    cp_back_to_back: cover property (
        @(posedge clk) disable iff (!rst_n)
        (u_fsm_state == COMPLETE) ##1 (u_fsm_state == IDLE) ##1 (u_fsm_state == ISSUE)
    );

endmodule


// ============================================================
// BIND — conecta el módulo SVA al DUT sin modificar el RTL
//
// Las señales internas se acceden usando la jerarquía de
// instancias definida en sram_bank_controller:
//   u_fsm    → bank_fsm
//   u_capture→ request_capture_reg
// ============================================================
bind sram_bank_controller sram_bank_controller_sva #(
    .N_BANKS        (N_BANKS),
    .READ_LATENCY   (READ_LATENCY),
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .BANK_ADDR_WIDTH(BANK_ADDR_WIDTH),
    .LAT_CNT_W      (LAT_CNT_W)
) u_sram_bank_controller_sva (
    // Reloj y reset
    .clk                   (clk),
    .rst_n                 (rst_n),

    // Entradas desde Scheduler
    .bank_req_valid        (bank_req_valid),
    .bank_req_op           (bank_req_op),
    .bank_req_addr         (bank_req_addr),
    .bank_req_wdata        (bank_req_wdata),
    .bank_req_wstrb        (bank_req_wstrb),
    .bank_req_tag          (bank_req_tag),

    // Salidas hacia SRAM IP
    .sram_en               (sram_en),
    .sram_we               (sram_we),
    .sram_addr             (sram_addr),
    .sram_din              (sram_din),
    .sram_wstrb            (sram_wstrb),

    // Salidas hacia Scheduler
    .bank_busy             (bank_busy),

    // Salidas hacia WR Resp Collector
    .wr_resp_valid         (wr_resp_valid),

    // Salidas hacia RD Resp Mux
    .rd_resp_data          (rd_resp_data),
    .rd_resp_tag           (rd_resp_tag),
    .rd_resp_valid         (rd_resp_valid),

    // Señales internas — u_fsm (bank_fsm)
    .u_fsm_state           (u_fsm.state),
    .u_fsm_fsm_idle        (u_fsm.fsm_idle),
    .u_fsm_issue_en        (u_fsm.issue_en),
    .u_fsm_rd_capture_en   (u_fsm.rd_capture_en),

    // Señales internas — u_capture (request_capture_reg)
    .u_capture_is_wr       (u_capture.is_wr),
    .u_capture_is_rd       (u_capture.is_rd),
    .u_capture_capture_en  (u_capture.capture_en),
    .u_capture_lat_addr    (u_capture.lat_addr),
    .u_capture_lat_tag     (u_capture.lat_tag)
);
