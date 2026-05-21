// ============================================================
//  File    : scheduler_sva.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : Scheduler — Formal/Simulation Assertions
//  Type    : SVA bind module (no modifica RTL original)
//
//  Descripción:
//    Propiedades SVA para el módulo scheduler. Se conecta
//    mediante `bind` sin tocar el RTL cerrado. Cubre:
//      1. Mutex ok/discard por canal
//      2. Dispatch válido (máx 2 bancos, 2-hot distintos)
//      3. No-silent-pop (principio rector)
//      4. Grant implica precondiciones satisfechas
//      5. Política WRITE-FIRST
//      6. Coherencia dispatch ↔ grant
//      7. Monotonicidad de contadores de error M5
//      8. Backpressure canal B (wr_resp_full congela WR path)
//
//  Uso VCS:
//    vcs -sverilog +lint=all scheduler.sv scheduler_sva.sv ...
// ============================================================
`timescale 1ns/1ps

module scheduler_sva #(
    parameter ADDR_W          = 32,
    parameter AXI_DATA_WIDTH  = 32,
    parameter N_BANKS         = 4,
    parameter BANK_SIZE_BYTES = 1024,
    parameter ERR_CNT_W       = 16
)(
    // ── Reloj y reset ────────────────────────────────────
    input wire clk,
    input wire rst_n,

    // ── Señales externas del scheduler (inputs al DUT) ───
    input wire [$clog2(N_BANKS)-1:0]                            wr_bank_id,
    input wire [$clog2(N_BANKS)-1:0]                            rd_bank_id,
    input wire wr_addr_valid,
    input wire rd_addr_valid,
    input wire wr_req_pndng,
    input wire rd_req_pndng,
    input wire rob_tag_free,
    input wire bank_busy [0:N_BANKS-1],
    input wire wr_resp_full,

    // ── Señales internas del scheduler (wires entre submódulos) ──
    input wire wr_ok,
    input wire wr_discard,
    input wire rd_ok,
    input wire rd_discard,
    input wire same_bank,
    input wire wr_bank_free,
    input wire rd_bank_free,
    input wire grant_wr,
    input wire grant_rd,

    // ── Outputs del scheduler ────────────────────────────
    input wire wr_req_pop,
    input wire rd_req_pop,
    input wire bank_req_valid [0:N_BANKS-1],
    input wire bank_req_op    [0:N_BANKS-1],
    input wire [ERR_CNT_W-1:0] wr_err_cnt,
    input wire [ERR_CNT_W-1:0] rd_err_cnt
);

    // ── Parámetros locales ───────────────────────────────
    localparam BANK_BITS = $clog2(N_BANKS);

    // ================================================================
    // CONVERSIÓN: unpacked array → packed vector para $onehot0
    //   bank_req_valid_packed[i] = bank_req_valid[i]
    //   bank_req_op_packed[i]    = bank_req_op[i]
    // ================================================================
    wire [N_BANKS-1:0] bank_req_valid_packed;
    wire [N_BANKS-1:0] bank_req_op_packed;

    genvar gi;
    generate
        for (gi = 0; gi < N_BANKS; gi = gi + 1) begin : gen_pack
            assign bank_req_valid_packed[gi] = bank_req_valid[gi];
            assign bank_req_op_packed[gi]    = bank_req_op[gi];
        end
    endgenerate

    // ================================================================
    // CATEGORÍA 1 — Mutex ok / discard por canal
    //   Invariante: un request pendiente no puede ser ok Y discard
    //   al mismo tiempo. Son mutuamente excluyentes por construcción
    //   combinacional (una depende de addr_valid, la otra de !addr_valid).
    // ================================================================

    // [CRÍTICA] wr_ok y wr_discard son mutuamente excluyentes.
    // Protege: lógica de valid-check WR no puede emitir señales
    // contradictorias que lleven a pop + discard simultáneos.
    a_wr_ok_discard_mutex: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(wr_ok && wr_discard)
    ) else $error("[SVA FAIL] a_wr_ok_discard_mutex: wr_ok && wr_discard activos simultaneamente");

    // [CRÍTICA] rd_ok y rd_discard son mutuamente excluyentes.
    // Protege: lógica de valid-check RD no puede emitir señales
    // contradictorias que comprometan la integridad del canal RD.
    a_rd_ok_discard_mutex: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(rd_ok && rd_discard)
    ) else $error("[SVA FAIL] a_rd_ok_discard_mutex: rd_ok && rd_discard activos simultaneamente");

    // ================================================================
    // CATEGORÍA 2 — Dispatch válido del bus
    //
    //   El scheduler tiene DOS modos legítimos de dispatch:
    //     a) Un solo banco activo   → WR-only, RD-only, o conflicto
    //        same_bank (WRITE-FIRST pospone RD).
    //     b) Exactamente dos bancos → both_no_conflict: grant_wr y
    //        grant_rd a bancos DISTINTOS en el mismo ciclo.
    //
    //   Invariante 2a — Máximo 2 bits activos en bank_req_valid.
    //     $countones(packed) <= 2 en todo momento.
    //     Más de 2 activos indica un bug en unified_dispatch_logic.
    //
    //   Invariante 2b — Si hay 2 activos, deben ser exactamente
    //     wr_bank_id y rd_bank_id, y esos IDs deben ser distintos
    //     (condición both_no_conflict). No puede haber 2-hot con
    //     bancos iguales (eso sería same_bank, que produce 1-hot).
    //
    //   Invariante 2c — Si grant_wr y grant_rd activos simultáneos
    //     entonces wr_bank_id != rd_bank_id (bancos distintos).
    //     Un double-grant sobre el mismo banco viola WRITE-FIRST.
    // ================================================================

    // [CRÍTICA] Como máximo 2 bancos activos simultáneamente.
    // Protege: unified_dispatch_logic solo puede activar 1 banco
    // (casos WR-only / RD-only / conflicto) o 2 bancos distintos
    // (both_no_conflict). Más de 2 implica un bug en el demux.
    a_bank_req_valid_max2: assert property (
        @(posedge clk) disable iff (!rst_n)
        $countones(bank_req_valid_packed) <= 2
    ) else $error("[SVA FAIL] a_bank_req_valid_max2: mas de 2 bank_req_valid activos");

    // [CRÍTICA] Si hay 2 bancos activos, los IDs WR y RD son distintos.
    // Protege: el caso 2-hot solo ocurre en both_no_conflict;
    // 2-hot con wr_bank_id == rd_bank_id es imposible por diseño y
    // señalaría una corrupción en bank_conflict_check o el resolver.
    a_bank_req_valid_2hot_distinct: assert property (
        @(posedge clk) disable iff (!rst_n)
        ($countones(bank_req_valid_packed) == 2)
        |-> (wr_bank_id != rd_bank_id)
    ) else $error("[SVA FAIL] a_bank_req_valid_2hot_distinct: 2 bancos activos con mismo ID");

    // [CRÍTICA] Double-grant solo ocurre con bancos distintos.
    // Protege: si el resolver emite grant_wr y grant_rd a la vez,
    // deben apuntar a bancos físicamente distintos; lo contrario
    // violaría la exclusión de recursos de la SRAM.
    a_double_grant_distinct_banks: assert property (
        @(posedge clk) disable iff (!rst_n)
        (grant_wr && grant_rd) |-> (wr_bank_id != rd_bank_id)
    ) else $error("[SVA FAIL] a_double_grant_distinct_banks: double-grant sobre mismo banco");

    // ================================================================
    // CATEGORÍA 3 — No-silent-pop (principio rector del diseño)
    //   Invariante: ningún pop ocurre sin razón explícita.
    //   wr_req_pop solo puede surgir de grant_wr (despacho exitoso)
    //   o de wr_discard (dirección inválida contabilizada en M5).
    //   Cualquier otro pop sería un descarte silencioso prohibido.
    // ================================================================

    // [CRÍTICA] wr_req_pop solo ocurre por grant_wr o wr_discard.
    // Protege: ninguna transacción de escritura se pierde silenciosamente;
    // todo pop queda cubierto por una causa auditada.
    a_wr_pop_reason: assert property (
        @(posedge clk) disable iff (!rst_n)
        wr_req_pop |-> (grant_wr || wr_discard)
    ) else $error("[SVA FAIL] a_wr_pop_reason: wr_req_pop sin causa valida");

    // [CRÍTICA] rd_req_pop solo ocurre por grant_rd o rd_discard.
    // Protege: ninguna transacción de lectura se pierde silenciosamente;
    // todo pop queda cubierto por una causa auditada.
    a_rd_pop_reason: assert property (
        @(posedge clk) disable iff (!rst_n)
        rd_req_pop |-> (grant_rd || rd_discard)
    ) else $error("[SVA FAIL] a_rd_pop_reason: rd_req_pop sin causa valida");

    // ================================================================
    // CATEGORÍA 4 — Grant implica precondiciones satisfechas
    //   Invariante: el árbitro solo emite grants cuando todas las
    //   condiciones habilitantes están activas. Un grant sin
    //   precondición indica un bug en priority_resolver.
    // ================================================================

    // [ALTA] grant_wr requiere request válido Y banco libre.
    // Protege: el árbitro no despacha escrituras a bancos ocupados
    // ni cuando no hay request pendiente con dirección válida.
    a_grant_wr_valid: assert property (
        @(posedge clk) disable iff (!rst_n)
        grant_wr |-> (wr_ok && wr_bank_free)
    ) else $error("[SVA FAIL] a_grant_wr_valid: grant_wr sin precondiciones satisfechas");

    // [ALTA] grant_rd requiere request válido, banco libre Y slot ROB disponible.
    // Protege: el árbitro no despacha lecturas cuando el ROB está lleno,
    // lo que causaría aliasing de tags y corrupción de orden en canal R.
    a_grant_rd_valid: assert property (
        @(posedge clk) disable iff (!rst_n)
        grant_rd |-> (rd_ok && rd_bank_free && rob_tag_free)
    ) else $error("[SVA FAIL] a_grant_rd_valid: grant_rd sin precondiciones satisfechas");

    // ================================================================
    // CATEGORÍA 5 — Política WRITE-FIRST
    //   Invariante: cuando WR y RD compiten por el mismo banco y ambos
    //   están listos, solo WR obtiene el grant (RD se pospone).
    //   Esto refleja la política de arbitraje explícita del diseño.
    // ================================================================

    // [ALTA] En conflicto de banco con ambos candidates listos, WR gana y RD espera.
    // Protege: la política WRITE-FIRST se mantiene bajo cualquier condición;
    // una violación permitiría que RD bloquee a WR en conflicto, invertiendo la política.
    a_write_first: assert property (
        @(posedge clk) disable iff (!rst_n)
        (wr_ok && wr_bank_free &&
         rd_ok && rd_bank_free && rob_tag_free &&
         same_bank)
        |-> (grant_wr && !grant_rd)
    ) else $error("[SVA FAIL] a_write_first: politica WRITE-FIRST violada en conflicto de banco");

    // ================================================================
    // CATEGORÍA 6 — Coherencia dispatch ↔ grant
    //   Invariante: si el árbitro emite un grant, el banco destino
    //   debe aparecer activo en el bus de dispatch con la operación
    //   correcta. Detecta desconexión entre árbitro y demux.
    // ================================================================

    // [CRÍTICA] grant_wr activa el banco correcto con operación WR (op=1).
    // Protege: unified_dispatch_logic envía el valid al banco seleccionado
    // por wr_bank_id con op=WR; cualquier desviación corrompe la SRAM.
    a_dispatch_wr_coherent: assert property (
        @(posedge clk) disable iff (!rst_n)
        grant_wr |-> (bank_req_valid[wr_bank_id] && bank_req_op[wr_bank_id])
    ) else $error("[SVA FAIL] a_dispatch_wr_coherent: banco WR no activado correctamente");

    // [CRÍTICA] grant_rd activa el banco correcto con operación RD (op=0).
    // Protege: unified_dispatch_logic envía el valid al banco seleccionado
    // por rd_bank_id con op=RD; una operación WR en lugar de RD destruiría datos.
    a_dispatch_rd_coherent: assert property (
        @(posedge clk) disable iff (!rst_n)
        grant_rd |-> (bank_req_valid[rd_bank_id] && !bank_req_op[rd_bank_id])
    ) else $error("[SVA FAIL] a_dispatch_rd_coherent: banco RD no activado correctamente");

    // ================================================================
    // CATEGORÍA 7 — Monotonicidad de contadores de error M5
    //   Invariante: wr_err_cnt y rd_err_cnt nunca decrecen.
    //   Son registros de métrica acumulativa; un decremento indicaría
    //   un reset espurio o lógica incorrecta en error_counter.
    //   Nota: $past() compara el valor en el ciclo anterior; la aserción
    //   se evalúa en el flanco activo siguiente.
    // ================================================================

    // [MEDIA] wr_err_cnt es monótonamente no decreciente tras reset.
    // Protege: la métrica M5 de escrituras descartadas no puede retroceder;
    // un valor menor en el ciclo siguiente indica corrupción del contador.
    a_wr_err_monotonic: assert property (
        @(posedge clk) disable iff (!rst_n)
        wr_err_cnt >= $past(wr_err_cnt)
    ) else $error("[SVA FAIL] a_wr_err_monotonic: wr_err_cnt decreció");

    // [MEDIA] rd_err_cnt es monótonamente no decreciente tras reset.
    // Protege: la métrica M5 de lecturas descartadas no puede retroceder;
    // un valor menor en el ciclo siguiente indica corrupción del contador.
    a_rd_err_monotonic: assert property (
        @(posedge clk) disable iff (!rst_n)
        rd_err_cnt >= $past(rd_err_cnt)
    ) else $error("[SVA FAIL] a_rd_err_monotonic: rd_err_cnt decreció");

    // ================================================================
    // CATEGORÍA 8 — Backpressure del canal B (wr_resp_full)
    //   Invariante: cuando el contador de respuestas WR está lleno,
    //   el Scheduler no puede emitir grant_wr ni wr_discard.
    //   Versión A del diseño: wr_resp_full congela TODO el WR path
    //   (ni dispatch ni discard) hasta que el master consuma respuestas.
    //   Protege: evitar overflow del pending_counter y pérdida silenciosa
    //   de respuestas B cuando el master mantiene bready=0.
    // ================================================================

    // [CRÍTICA] Cuando wr_resp_full=1, no hay grant_wr ni wr_discard.
    // Protege: el WR path queda congelado con backpressure alta;
    // cualquier pop de la WR REQ FIFO en ese estado sería un descarte
    // silencioso porque la respuesta no tendría adónde ir.
    a_no_wr_activity_when_resp_full: assert property (
        @(posedge clk) disable iff (!rst_n)
        wr_resp_full |-> (!grant_wr && !wr_discard)
    ) else $error("[SVA FAIL] a_no_wr_activity_when_resp_full: grant_wr o wr_discard activo con wr_resp_full=1");

endmodule


// ============================================================
//  BIND — conecta scheduler_sva al DUT scheduler sin
//  modificar el RTL original.
//
//  Señales internas del scheduler accesibles por bind:
//    wr_ok, wr_discard  → u_wr_req_valid_check.*
//    rd_ok, rd_discard  → u_rd_req_valid_check.*
//    same_bank          → u_bank_conflict_check.*
//    wr_bank_free,
//    rd_bank_free       → u_bank_availability_check.*
//    grant_wr, grant_rd → u_priority_resolver.*
// ============================================================
bind scheduler scheduler_sva #(
    .ADDR_W         (ADDR_W),
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .N_BANKS        (N_BANKS),
    .BANK_SIZE_BYTES(BANK_SIZE_BYTES),
    .ERR_CNT_W      (ERR_CNT_W)
) u_scheduler_sva (
    // Reloj y reset
    .clk              (clk),
    .rst_n            (rst_n),

    // Entradas externas del scheduler
    .wr_bank_id       (wr_bank_id),
    .rd_bank_id       (rd_bank_id),
    .wr_addr_valid    (wr_addr_valid),
    .rd_addr_valid    (rd_addr_valid),
    .wr_req_pndng     (wr_req_pndng),
    .rd_req_pndng     (rd_req_pndng),
    .rob_tag_free     (rob_tag_free),
    .bank_busy        (bank_busy),
    .wr_resp_full     (wr_resp_full),

    // Señales internas (wires entre submódulos, accesibles por bind)
    .wr_ok            (wr_ok),
    .wr_discard       (wr_discard),
    .rd_ok            (rd_ok),
    .rd_discard       (rd_discard),
    .same_bank        (same_bank),
    .wr_bank_free     (wr_bank_free),
    .rd_bank_free     (rd_bank_free),
    .grant_wr         (grant_wr),
    .grant_rd         (grant_rd),

    // Outputs del scheduler
    .wr_req_pop       (wr_req_pop),
    .rd_req_pop       (rd_req_pop),
    .bank_req_valid   (bank_req_valid),
    .bank_req_op      (bank_req_op),
    .wr_err_cnt       (wr_err_cnt),
    .rd_err_cnt       (rd_err_cnt)
);
