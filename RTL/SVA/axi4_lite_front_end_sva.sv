// ============================================================
//  File    : axi4_lite_front_end_sva.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : AXI4-Lite Front End
//  Type    : SVA bind module — aserciones formales/dinámicas
//
//  Descripción:
//    Módulo de aserciones SVA para verificar el comportamiento
//    del axi4_lite_front_end. Se conecta vía `bind` sin
//    modificar el RTL original. Cubre:
//      - Cumplimiento de protocolo AXI4-Lite (valid stability,
//        payload stability, reset behavior)
//      - Backpressure correcto hacia el master
//      - No descarte silencioso (no push a FIFO llena)
//      - Captura y limpieza correcta de hold registers
//      - Formatos correctos de buses packed WR/RD REQ
//
//  Compilación: vcs -sverilog axi4_lite_front_end.sv
//                               axi4_lite_front_end_sva.sv
//  ============================================================

`timescale 1ns/1ps

module axi4_lite_front_end_sva #(
    parameter ADDR_W         = 32,
    parameter AXI_DATA_WIDTH = 32
)(
    // ── Clock y reset ────────────────────────────────────────
    input wire clk,
    input wire rst_n,

    // ── AXI4-Lite AW Channel ─────────────────────────────────
    input wire              s_axi_awvalid,
    input wire              s_axi_awready,
    input wire [ADDR_W-1:0] s_axi_awaddr,

    // ── AXI4-Lite W Channel ──────────────────────────────────
    input wire                        s_axi_wvalid,
    input wire                        s_axi_wready,
    input wire [AXI_DATA_WIDTH-1:0]   s_axi_wdata,
    input wire [AXI_DATA_WIDTH/8-1:0] s_axi_wstrb,

    // ── AXI4-Lite AR Channel ─────────────────────────────────
    input wire              s_axi_arvalid,
    input wire              s_axi_arready,
    input wire [ADDR_W-1:0] s_axi_araddr,

    // ── Hacia WR REQ FIFO ────────────────────────────────────
    input wire                                                   wr_req_push,
    input wire [ADDR_W + AXI_DATA_WIDTH + AXI_DATA_WIDTH/8 : 0] wr_req_data,
    input wire                                                   wr_req_full,

    // ── Hacia RD REQ FIFO ────────────────────────────────────
    input wire              rd_req_push,
    input wire [ADDR_W:0]   rd_req_data,
    input wire              rd_req_full,

    // ── Señales internas de u_aw_capture_reg ─────────────────
    input wire [ADDR_W-1:0] aw_hold_addr,
    input wire              aw_hold_valid,

    // ── Señales internas de u_w_capture_reg ──────────────────
    input wire [AXI_DATA_WIDTH-1:0]   w_hold_data,
    input wire [AXI_DATA_WIDTH/8-1:0] w_hold_strb,
    input wire                        w_hold_valid
);

    // ============================================================
    // Señales auxiliares locales
    // ============================================================
    wire aw_fire;
    wire  w_fire;
    assign aw_fire = s_axi_awvalid & s_axi_awready;
    assign  w_fire = s_axi_wvalid  & s_axi_wready;

    // ============================================================
    // CATEGORÍA 1 — AXI4-Lite protocol compliance: valid stability
    //
    // El estándar AXI4-Lite exige que valid, una vez asertado,
    // se mantenga hasta que ocurra el handshake (valid & ready).
    // Bajar valid prematuramente es una violación de protocolo
    // que puede provocar pérdida silenciosa de transacciones.
    // ============================================================

    // [CRÍTICA] awvalid no puede bajar sin que haya ocurrido el handshake.
    // Protege: integridad del canal AW — el master no puede retirar
    // una solicitud de dirección de escritura a mitad de camino.
    a_awvalid_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s_axi_awvalid && !s_axi_awready) |=> s_axi_awvalid
    ) else $error("[SVA FAIL] a_awvalid_stable: awvalid bajó sin handshake");

    // [CRÍTICA] wvalid no puede bajar sin que haya ocurrido el handshake.
    // Protege: integridad del canal W — el master no puede retirar
    // un dato de escritura a mitad de camino.
    a_wvalid_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s_axi_wvalid && !s_axi_wready) |=> s_axi_wvalid
    ) else $error("[SVA FAIL] a_wvalid_stable: wvalid bajó sin handshake");

    // [CRÍTICA] arvalid no puede bajar sin que haya ocurrido el handshake.
    // Protege: integridad del canal AR — el master no puede retirar
    // una solicitud de dirección de lectura a mitad de camino.
    a_arvalid_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s_axi_arvalid && !s_axi_arready) |=> s_axi_arvalid
    ) else $error("[SVA FAIL] a_arvalid_stable: arvalid bajó sin handshake");

    // ============================================================
    // CATEGORÍA 2 — AXI4-Lite protocol compliance: payload stability
    //
    // Los campos de payload (dirección y datos) deben mantenerse
    // estables mientras el valid está asertado y no ha ocurrido
    // el handshake. Cambios en el payload antes del handshake
    // producen capturas erróneas en los hold registers.
    // ============================================================

    // [CRÍTICA] awaddr estable mientras awvalid espera handshake.
    // Protege: el hold register captura exactamente la dirección
    // que el master presentó — no una versión intermedia modificada.
    a_awaddr_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s_axi_awvalid && !s_axi_awready) |=> $stable(s_axi_awaddr)
    ) else $error("[SVA FAIL] a_awaddr_stable: awaddr cambió sin handshake");

    // [CRÍTICA] wdata y wstrb estables mientras wvalid espera handshake.
    // Protege: integridad del payload de escritura — dato y máscara
    // deben ser coherentes en el momento de la captura.
    a_wdata_wstrb_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s_axi_wvalid && !s_axi_wready)
            |=> ($stable(s_axi_wdata) && $stable(s_axi_wstrb))
    ) else $error("[SVA FAIL] a_wdata_wstrb_stable: wdata/wstrb cambiaron sin handshake");

    // [CRÍTICA] araddr estable mientras arvalid espera handshake.
    // Protege: la dirección de lectura enviada a la RD REQ FIFO
    // corresponde exactamente a lo que el master solicitó.
    a_araddr_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s_axi_arvalid && !s_axi_arready) |=> $stable(s_axi_araddr)
    ) else $error("[SVA FAIL] a_araddr_stable: araddr cambió sin handshake");

    // ============================================================
    // CATEGORÍA 3 — AXI4-Lite protocol compliance: reset behavior
    //
    // Durante el reset el DUT no debe presentar ready alto, ya que
    // un master podría interpretar ready=1 como invitación a
    // completar un handshake cuando el sistema aún no está listo.
    // Esta aserción usa implicación simple (no |=>) porque aplica
    // en el ciclo activo de reset, no en el siguiente.
    // Nota: NO se usa disable iff para que la aserción sea activa
    // precisamente cuando rst_n=0.
    // ============================================================

    // [ALTA] Durante reset activo ningún ready debe estar alto.
    // Protege: el master no puede completar handshakes mientras el
    // DUT está en reset — evita que capture regs queden en estado
    // indeterminado en la primera transacción post-reset.
    a_no_ready_in_reset: assert property (
        @(posedge clk)
        !rst_n |-> (!s_axi_awready && !s_axi_wready && !s_axi_arready)
    ) else $error("[SVA FAIL] a_no_ready_in_reset: ready alto durante reset");

    // ============================================================
    // CATEGORÍA 4 — Backpressure correcto hacia el master
    //
    // El DUT debe bloquear el handshake cuando no puede procesar
    // más transacciones. Hay dos razones para negar ready:
    // (a) la FIFO downstream está llena, (b) el hold register
    // ya tiene un dato pendiente (no se puede capturar otro).
    // Violar estas invariantes genera sobrescritura de datos.
    // ============================================================

    // [CRÍTICA] awready no puede estar alto si wr_req_full=1.
    // Protege: no se acepta nueva dirección AW si no hay espacio
    // downstream — evita que aw_hold_valid se seteé sin poder
    // hacer el push correspondiente, bloqueando el pipeline.
    a_awready_respects_full: assert property (
        @(posedge clk) disable iff (!rst_n)
        wr_req_full |-> !s_axi_awready
    ) else $error("[SVA FAIL] a_awready_respects_full: awready alto con wr_req_full");

    // [CRÍTICA] wready no puede estar alto si wr_req_full=1.
    // Protege: análogo a lo anterior para el canal W — coherencia
    // entre la gestión de backpressure en AW y W.
    a_wready_respects_full: assert property (
        @(posedge clk) disable iff (!rst_n)
        wr_req_full |-> !s_axi_wready
    ) else $error("[SVA FAIL] a_wready_respects_full: wready alto con wr_req_full");

    // [CRÍTICA] arready no puede estar alto si rd_req_full=1.
    // Protege: no se acepta nueva solicitud de lectura si la RD
    // REQ FIFO está llena — principio de no descarte silencioso.
    a_arready_respects_full: assert property (
        @(posedge clk) disable iff (!rst_n)
        rd_req_full |-> !s_axi_arready
    ) else $error("[SVA FAIL] a_arready_respects_full: arready alto con rd_req_full");

    // [ALTA] awready no puede estar alto si aw_hold_valid=1.
    // Protege: el hold register AW tiene capacidad para una sola
    // dirección — aceptar una nueva antes de pushear la actual
    // sobreescribiría la dirección pendiente (descarte silencioso).
    a_awready_respects_hold: assert property (
        @(posedge clk) disable iff (!rst_n)
        aw_hold_valid |-> !s_axi_awready
    ) else $error("[SVA FAIL] a_awready_respects_hold: awready alto con aw_hold_valid");

    // [ALTA] wready no puede estar alto si w_hold_valid=1.
    // Protege: análogo al anterior para el hold register W.
    a_wready_respects_hold: assert property (
        @(posedge clk) disable iff (!rst_n)
        w_hold_valid |-> !s_axi_wready
    ) else $error("[SVA FAIL] a_wready_respects_hold: wready alto con w_hold_valid");

    // ============================================================
    // CATEGORÍA 5 — No descarte silencioso: no push a FIFO llena
    //
    // Invariante fundamental del diseño. Aunque las lógicas de
    // ready y check deberían prevenirlo, estas aserciones actúan
    // como segunda línea de defensa: detectan si por algún bug
    // de implementación se genera un push cuando la FIFO ya está
    // llena, lo que causaría pérdida de datos sin indicación.
    // ============================================================

    // [CRÍTICA] wr_req_push jamás debe ocurrir con wr_req_full=1.
    // Protege: principio de no descarte silencioso — un push a FIFO
    // llena en esta implementación silenciosamente pierde la entrada.
    a_no_wr_push_full: assert property (
        @(posedge clk) disable iff (!rst_n)
        wr_req_push |-> !wr_req_full
    ) else $error("[SVA FAIL] a_no_wr_push_full: push WR con FIFO llena");

    // [CRÍTICA] rd_req_push jamás debe ocurrir con rd_req_full=1.
    // Protege: análogo al anterior para el path de lectura.
    a_no_rd_push_full: assert property (
        @(posedge clk) disable iff (!rst_n)
        rd_req_push |-> !rd_req_full
    ) else $error("[SVA FAIL] a_no_rd_push_full: push RD con FIFO llena");

    // ============================================================
    // CATEGORÍA 6 — Captura correcta en hold registers
    //
    // Después de un handshake (fire), el hold register debe
    // contener exactamente el payload presentado en ese ciclo.
    // Estas aserciones verifican tanto el flag valid como el
    // valor almacenado, usando $past para referenciar el dato
    // del ciclo en que ocurrió el fire.
    // ============================================================

    // [CRÍTICA] Después de aw_fire, aw_hold_valid=1 y el addr capturado
    // es el que el master presentó en ese ciclo.
    // Protege: la dirección AW que llega al Scheduler es exactamente
    // la que el master envió — sin corrupción en la captura.
    a_aw_hold_captured: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s_axi_awvalid && s_axi_awready)
            |=> (aw_hold_valid &&
                 aw_hold_addr == $past(s_axi_awaddr))
    ) else $error("[SVA FAIL] a_aw_hold_captured: captura incorrecta en aw_hold");

    // [CRÍTICA] Después de w_fire, w_hold_valid=1 y data/strb capturados
    // son los que el master presentó en ese ciclo.
    // Protege: el payload de escritura que llega al banco SRAM es
    // exactamente el que el master envió — sin corrupción.
    a_w_hold_captured: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s_axi_wvalid && s_axi_wready)
            |=> (w_hold_valid &&
                 w_hold_data == $past(s_axi_wdata) &&
                 w_hold_strb == $past(s_axi_wstrb))
    ) else $error("[SVA FAIL] a_w_hold_captured: captura incorrecta en w_hold");

    // ============================================================
    // CATEGORÍA 7 — Hold clear correcto
    //
    // Los hold registers deben limpiarse después del push, pero
    // no si en el mismo ciclo llegó un nuevo fire (en ese caso
    // el nuevo dato gana — prioridad fire sobre clear). Estas
    // aserciones verifican ambas condiciones de la FSM implícita
    // de los capture registers.
    // ============================================================

    // [ALTA] aw_hold_valid se limpia después de wr_req_push, salvo que
    // simultáneamente haya ocurrido un nuevo aw_fire (fire tiene prioridad).
    // Protege: el hold register no retiene datos obsoletos después del
    // push, evitando pushes duplicados en ciclos posteriores.
    a_aw_hold_clear: assert property (
        @(posedge clk) disable iff (!rst_n)
        (wr_req_push && !aw_fire)
            |=> !aw_hold_valid
    ) else $error("[SVA FAIL] a_aw_hold_clear: aw_hold_valid no se limpió tras push");

    // [ALTA] w_hold_valid se limpia después de wr_req_push, salvo que
    // simultáneamente haya ocurrido un nuevo w_fire (fire tiene prioridad).
    // Protege: análogo al anterior para el hold register W.
    a_w_hold_clear: assert property (
        @(posedge clk) disable iff (!rst_n)
        (wr_req_push && !w_fire)
            |=> !w_hold_valid
    ) else $error("[SVA FAIL] a_w_hold_clear: w_hold_valid no se limpió tras push");

    // [ALTA] Si aw_fire y wr_req_push ocurren simultáneamente, aw_hold_valid
    // debe mantenerse en 1 (la nueva dirección se captura en el mismo ciclo).
    // Protege: la prioridad fire-sobre-clear evita perder la dirección
    // que llega simultáneamente con el push de la anterior.
    a_aw_fire_priority: assert property (
        @(posedge clk) disable iff (!rst_n)
        (aw_fire && wr_req_push)
            |=> aw_hold_valid
    ) else $error("[SVA FAIL] a_aw_fire_priority: aw_hold_valid bajó pese a aw_fire simultáneo");

    // [ALTA] Si w_fire y wr_req_push ocurren simultáneamente, w_hold_valid
    // debe mantenerse en 1 (el nuevo payload se captura en el mismo ciclo).
    // Protege: análogo al anterior para el hold register W.
    a_w_fire_priority: assert property (
        @(posedge clk) disable iff (!rst_n)
        (w_fire && wr_req_push)
            |=> w_hold_valid
    ) else $error("[SVA FAIL] a_w_fire_priority: w_hold_valid bajó pese a w_fire simultáneo");

    // ============================================================
    // CATEGORÍA 8 — Push completo cuando ambos hold están listos
    //
    // Una vez que ambos hold registers tienen datos válidos y la
    // FIFO no está llena, el push debe ocurrir en ese mismo ciclo.
    // Esta es la garantía de liveness del assembler: si las
    // condiciones están dadas, la acción ocurre sin demora.
    // ============================================================

    // [CRÍTICA] Si aw_hold_valid && w_hold_valid && !wr_req_full,
    // entonces wr_req_push debe estar activo en ese mismo ciclo.
    // Protege: el assembler no introduce latencia adicional cuando
    // ambos componentes del request están listos — garantía de
    // progreso y throughput máximo del canal de escritura.
    a_wr_push_when_both_ready: assert property (
        @(posedge clk) disable iff (!rst_n)
        (aw_hold_valid && w_hold_valid && !wr_req_full)
            |-> wr_req_push
    ) else $error("[SVA FAIL] a_wr_push_when_both_ready: no hubo push con ambos hold válidos");

    // ============================================================
    // CATEGORÍA 9 — Bus packed correctness
    //
    // El formato del bus packed es un contrato entre el front-end
    // y el Scheduler/decoders. El bit MSB es el opcode (1=WR, 0=RD)
    // y el payload debe coincidir exactamente con los hold registers
    // o la dirección AR vigente. Errores aquí producen escrituras
    // en direcciones incorrectas o lecturas con datos erróneos.
    // ============================================================

    // [CRÍTICA] El MSB del bus WR REQ debe ser 1 (opcode WR) en cada push.
    // Protege: el Scheduler decodifica correctamente la operación —
    // un opcode incorrecto enviaría una lectura a la SRAM en lugar
    // de una escritura, corrompiendo datos silenciosamente.
    a_wr_req_data_format: assert property (
        @(posedge clk) disable iff (!rst_n)
        wr_req_push |->
            (wr_req_data[ADDR_W + AXI_DATA_WIDTH + AXI_DATA_WIDTH/8] == 1'b1)
    ) else $error("[SVA FAIL] a_wr_req_data_format: opcode WR incorrecto en bus packed");

    // [CRÍTICA] El MSB del bus RD REQ debe ser 0 (opcode RD) en cada push.
    // Protege: análogo al anterior — un opcode 1 enviaría una escritura
    // al banco en lugar de una lectura.
    a_rd_req_data_format: assert property (
        @(posedge clk) disable iff (!rst_n)
        rd_req_push |-> (rd_req_data[ADDR_W] == 1'b0)
    ) else $error("[SVA FAIL] a_rd_req_data_format: opcode RD incorrecto en bus packed");

    // [ALTA] El contenido completo del bus WR REQ coincide con los hold
    // registers en el momento del push.
    // Protege: no hay corrupción ni mezcla de campos en el formatter —
    // addr, data y strb que llegan al banco son exactamente los capturados.
    a_wr_req_data_content: assert property (
        @(posedge clk) disable iff (!rst_n)
        wr_req_push |->
            (wr_req_data ==
             {1'b1, aw_hold_addr, w_hold_data, w_hold_strb})
    ) else $error("[SVA FAIL] a_wr_req_data_content: contenido bus WR no coincide con hold regs");

    // [ALTA] El contenido del bus RD REQ coincide con la dirección AR vigente.
    // Protege: el path RD es puramente combinacional — la dirección que
    // llega al banco en un push es exactamente la que el master presentó.
    a_rd_req_data_content: assert property (
        @(posedge clk) disable iff (!rst_n)
        rd_req_push |-> (rd_req_data == {1'b0, s_axi_araddr})
    ) else $error("[SVA FAIL] a_rd_req_data_content: contenido bus RD no coincide con araddr");

    // ============================================================
    // CATEGORÍA 10 — RD path directo (sin capture register)
    //
    // A diferencia del canal WR que usa hold registers y requiere
    // que AW y W lleguen independientemente, el canal AR es
    // completamente combinacional: el push ocurre exactamente en
    // el mismo ciclo del handshake AR, sin latencia ni buffering.
    // ============================================================

    // [ALTA] rd_req_push ocurre si y solo si hay handshake AR en ese ciclo.
    // Protege: no hay descarte silencioso (AR sin push) ni push fantasma
    // (push sin AR fire) — bijección exacta entre handshakes y entradas FIFO.
    a_rd_push_reflects_ar_fire: assert property (
        @(posedge clk) disable iff (!rst_n)
        rd_req_push == (s_axi_arvalid && s_axi_arready)
    ) else $error("[SVA FAIL] a_rd_push_reflects_ar_fire: rd_req_push no coincide con AR fire");

endmodule


// ============================================================
// BIND — conecta el módulo SVA al DUT sin modificar el RTL
//
// VCS resolverá automáticamente los nombres de señales
// internas de los submódulos a través de la jerarquía
// una vez que bind establece el contexto de instanciación.
// ============================================================
bind axi4_lite_front_end axi4_lite_front_end_sva #(
    .ADDR_W        (ADDR_W),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
) u_axi4_lite_front_end_sva (
    // ── Clock y reset ───────────────────────────────────────
    .clk              (clk),
    .rst_n            (rst_n),

    // ── AXI4-Lite AW Channel ────────────────────────────────
    .s_axi_awvalid    (s_axi_awvalid),
    .s_axi_awready    (s_axi_awready),
    .s_axi_awaddr     (s_axi_awaddr),

    // ── AXI4-Lite W Channel ─────────────────────────────────
    .s_axi_wvalid     (s_axi_wvalid),
    .s_axi_wready     (s_axi_wready),
    .s_axi_wdata      (s_axi_wdata),
    .s_axi_wstrb      (s_axi_wstrb),

    // ── AXI4-Lite AR Channel ────────────────────────────────
    .s_axi_arvalid    (s_axi_arvalid),
    .s_axi_arready    (s_axi_arready),
    .s_axi_araddr     (s_axi_araddr),

    // ── Hacia WR REQ FIFO ───────────────────────────────────
    .wr_req_push      (wr_req_push),
    .wr_req_data      (wr_req_data),
    .wr_req_full      (wr_req_full),

    // ── Hacia RD REQ FIFO ───────────────────────────────────
    .rd_req_push      (rd_req_push),
    .rd_req_data      (rd_req_data),
    .rd_req_full      (rd_req_full),

    // ── Señales internas: u_aw_capture_reg ──────────────────
    .aw_hold_addr     (u_aw_capture_reg.aw_hold_addr),
    .aw_hold_valid    (u_aw_capture_reg.aw_hold_valid),

    // ── Señales internas: u_w_capture_reg ───────────────────
    .w_hold_data      (u_w_capture_reg.w_hold_data),
    .w_hold_strb      (u_w_capture_reg.w_hold_strb),
    .w_hold_valid     (u_w_capture_reg.w_hold_valid)
);
