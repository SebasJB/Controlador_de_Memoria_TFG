// ============================================================
//  File    : tb_wr_response_path.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : Write Response Path (B Channel AXI4-Lite)
//  Type    : Testbench directed — sin UVM, compatible VCS
//
//  Descripción:
//    Testbench directed para verificar wr_response_path con
//    8 casos de prueba que cubren: respuesta simple, carga
//    sostenida, backpressure, llenado completo, cancelación
//    push/pop, violación one-hot (contra-check SVA), stress
//    aleatorio y recuperación desde full.
//
//    El principio rector es *no silent discards*: al final de
//    T7 (stress aleatorio) se verifica que
//    bresp_received == bank_completions.
//
//  Parámetros:
//    N_BANKS       = 4
//    WR_FIFO_DEPTH = 8
//  (Para probar con WR_FIFO_DEPTH=2 cambiar el parámetro y
//   volver a compilar — los casos T3/T4 estresan rápido.)
//
//  Estilo: wire/reg, rst_n síncrono activo en bajo, snake_case.
// ============================================================

`timescale 1ns/1ps

module tb_wr_response_path;

    // ================================================================
    // Parámetros locales
    // ================================================================
    localparam int N_BANKS       = 4;
    localparam int WR_FIFO_DEPTH = 8;
    localparam int CLK_PERIOD    = 10;  // 100 MHz
    localparam int RESET_CYCLES  = 5;
    localparam int WATCHDOG_CYC  = 10_000;

    // ================================================================
    // Señales del DUT
    // ================================================================
    reg  clk;
    reg  rst_n;

    reg  wr_resp_valid_tb [0:N_BANKS-1];

    wire       s_axi_bvalid;
    wire [1:0] s_axi_bresp;
    reg        s_axi_bready;

    wire wr_resp_full;

    // ================================================================
    // Contadores de verificación
    // ================================================================
    int errors          = 0;
    int bresp_received  = 0;
    int bank_completions = 0;

    // ================================================================
    // Waveform dump
    // ================================================================
    initial begin
        $dumpfile("tb_wr_response_path.vcd");
        $dumpvars(0, tb_wr_response_path);
    end

    // ================================================================
    // Clock generator
    // ================================================================
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ================================================================
    // Watchdog
    // ================================================================
    initial begin
        #(CLK_PERIOD * WATCHDOG_CYC);
        $display("[WATCHDOG] Timeout — TB colgado. Terminando.");
        $finish;
    end

    // ================================================================
    // Instancia DUT
    // ================================================================
    wr_response_path #(
        .N_BANKS       (N_BANKS),
        .WR_FIFO_DEPTH (WR_FIFO_DEPTH)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .wr_resp_valid  (wr_resp_valid_tb),
        .s_axi_bvalid   (s_axi_bvalid),
        .s_axi_bresp    (s_axi_bresp),
        .s_axi_bready   (s_axi_bready),
        .wr_resp_full   (wr_resp_full)
    );

    // ================================================================
    // Monitor — cuenta handshakes B (bvalid && bready)
    // ================================================================
    always @(posedge clk) begin
        if (rst_n && s_axi_bvalid && s_axi_bready) begin
            bresp_received <= bresp_received + 1;
        end
    end

    // ================================================================
    // Monitor — cuenta bank completions (cualquier wr_resp_valid[i])
    // Nota: el Scheduler garantiza one-hot; si T6 viola esto, el
    // contador sube de a 1 por ciclo (OR-reduce interno).
    // Para T7 contamos usando OR-reduce de la misma forma que el DUT.
    // ================================================================
    always @(posedge clk) begin : mon_bank_completions
        integer k;
        if (rst_n) begin
            for (k = 0; k < N_BANKS; k = k + 1) begin
                if (wr_resp_valid_tb[k])
                    bank_completions <= bank_completions + 1;
            end
        end
    end

    // ================================================================
    // Helpers internos
    // ================================================================

    // Inicializa todas las entradas a valores seguros
    task automatic init_inputs();
        integer k;
        for (k = 0; k < N_BANKS; k = k + 1)
            wr_resp_valid_tb[k] <= 1'b0;
        s_axi_bready <= 1'b0;
    endtask

    // Espera N flancos de reloj
    task automatic wait_clk(input int n);
        repeat (n) @(posedge clk);
    endtask

    // ── Task: un banco completa escritura (pulso 1 ciclo) ──────────
    // Respeta one-hot: solo un banco activo por ciclo.
    task automatic bank_complete_write(input int bank_idx);
        @(posedge clk);
        wr_resp_valid_tb[bank_idx] <= 1'b1;
        @(posedge clk);
        wr_resp_valid_tb[bank_idx] <= 1'b0;
    endtask

    // ── Task: master acepta una respuesta B cuando bvalid sube ─────
    // Respeta protocolo AXI4-Lite: bready solo cambia en flancos.
    task automatic master_accept_bresp();
        // Espera activa hasta que bvalid suba
        @(posedge clk);
        while (!s_axi_bvalid) @(posedge clk);
        // Presenta bready en el mismo flanco donde bvalid=1
        s_axi_bready <= 1'b1;
        @(posedge clk);
        s_axi_bready <= 1'b0;
    endtask

    // ── Task: master acepta todo (bready sostenido) ─────────────────
    task automatic master_bready_always();
        s_axi_bready <= 1'b1;
    endtask

    // ── Task: master con bready sostenido off ───────────────────────
    task automatic master_bready_never();
        s_axi_bready <= 1'b0;
    endtask

    // ── Task: bready aleatorio por num_cycles ciclos ─────────────────
    // Genera backpressure no determinístico para T7.
    task automatic master_bready_random(input int num_cycles);
        integer i;
        for (i = 0; i < num_cycles; i = i + 1) begin
            @(posedge clk);
            s_axi_bready <= $random & 1'b1;
        end
        @(posedge clk);
        s_axi_bready <= 1'b0;
    endtask

    // ── Task: drenar completamente el contador ────────────────────────
    task automatic drain_all();
        master_bready_always();
        // Esperar hasta que bvalid baje (contador vacío) con timeout
        begin : wait_drain
            integer t;
            for (t = 0; t < 500; t = t + 1) begin
                @(posedge clk);
                if (!s_axi_bvalid) disable wait_drain;
            end
        end
        @(posedge clk);
        s_axi_bready <= 1'b0;
        @(posedge clk);
    endtask

    // ── Task: verificación con mensaje ───────────────────────────────
    task automatic check(
        input string   label,
        input reg       got,
        input reg       expected
    );
        if (got !== expected) begin
            $display("[FAIL] %s: got=%0b, expected=%0b", label, got, expected);
            errors = errors + 1;
        end else begin
            $display("[PASS] %s", label);
        end
    endtask

    task automatic check_int(
        input string label,
        input int    got,
        input int    expected
    );
        if (got !== expected) begin
            $display("[FAIL] %s: got=%0d, expected=%0d", label, got, expected);
            errors = errors + 1;
        end else begin
            $display("[PASS] %s: value=%0d", label, got);
        end
    endtask

    // ================================================================
    // Secuencia de tests principal
    // ================================================================
    initial begin : tb_main

        integer k;
        int snap_bresp;
        int snap_bank;

        // ── Reset ─────────────────────────────────────────────────
        $display("\n=== RESET ===");
        init_inputs();
        rst_n <= 1'b0;
        repeat (RESET_CYCLES) @(posedge clk);
        @(posedge clk);
        rst_n <= 1'b1;
        @(posedge clk);
        $display("Reset liberado.");

        // ============================================================
        // T1 — Una respuesta simple
        // Escenario: un banco completa escritura; el master acepta.
        // Verifica que bvalid sube y la respuesta se entrega.
        // ============================================================
        $display("\n=== T1: Una respuesta simple ===");
        snap_bresp = bresp_received;

        fork
            bank_complete_write(0);
            begin
                wait_clk(1);
                master_accept_bresp();
            end
        join

        wait_clk(2);
        check_int("T1: bresp_received+1", bresp_received, snap_bresp + 1);
        // Verificar que el contador volvió a 0 (bvalid debe bajar)
        check("T1: bvalid=0 post-handshake", s_axi_bvalid, 1'b0);
        wait_clk(2);

        // ============================================================
        // T2 — Múltiples respuestas secuenciales con bready sostenido
        // Escenario: bready siempre alto; 5 escrituras espaciadas.
        // Verifica throughput máximo: el contador nunca pasa de 1.
        // ============================================================
        $display("\n=== T2: Multiples respuestas, bready sostenido ===");
        snap_bresp = bresp_received;
        master_bready_always();
        wait_clk(1);

        for (k = 0; k < 5; k = k + 1) begin
            @(posedge clk);
            wr_resp_valid_tb[0] <= 1'b1;
            @(posedge clk);
            wr_resp_valid_tb[0] <= 1'b0;
            wait_clk(1); // Espaciado mínimo de 1 ciclo entre pulsos
        end

        wait_clk(4);
        s_axi_bready <= 1'b0;
        wait_clk(2);

        check_int("T2: bresp_received+5", bresp_received, snap_bresp + 5);
        wait_clk(2);

        // ============================================================
        // T3 — Back-to-back con backpressure (bready bajo)
        // Escenario: 3 escrituras consecutivas; bready=0 durante la
        // carga; luego drenar.
        // Verifica: bvalid se mantiene alto y el contador acumula.
        // ============================================================
        $display("\n=== T3: Backpressure — 3 respuestas acumuladas ===");
        snap_bresp = bresp_received;
        master_bready_never();

        // 3 pulsos back-to-back (bancos distintos para respetar one-hot
        // entre ciclos, aunque sea el mismo banco consecutivo aquí porque
        // cada pulso dura 1 ciclo y el siguiente empieza en el siguiente)
        @(posedge clk); wr_resp_valid_tb[0] <= 1'b1;
        @(posedge clk); wr_resp_valid_tb[0] <= 1'b0;
        @(posedge clk); wr_resp_valid_tb[1] <= 1'b1;
        @(posedge clk); wr_resp_valid_tb[1] <= 1'b0;
        @(posedge clk); wr_resp_valid_tb[2] <= 1'b1;
        @(posedge clk); wr_resp_valid_tb[2] <= 1'b0;
        wait_clk(2);

        // bvalid debe estar alto (hay respuestas pendientes)
        check("T3: bvalid=1 con backpressure", s_axi_bvalid, 1'b1);
        // wr_resp_full no debe activarse (depth=8, solo tenemos 3)
        check("T3: wr_resp_full=0 (cnt<depth)", wr_resp_full, 1'b0);

        // Ahora drenar
        drain_all();
        check_int("T3: bresp_received+3", bresp_received, snap_bresp + 3);
        check("T3: bvalid=0 post-drain", s_axi_bvalid, 1'b0);
        wait_clk(2);

        // ============================================================
        // T4 — Llenado completo del contador
        // Escenario: WR_FIFO_DEPTH pulsos con bready=0; verificar full.
        // Luego un pulso adicional con full=1 (stress test para SVA
        // a_push_full_but_valid — esperar que el warning se dispare).
        // Finalmente drenar y verificar recuperación.
        // ============================================================
        $display("\n=== T4: Llenado completo hasta wr_resp_full ===");
        snap_bresp = bresp_received;
        snap_bank  = bank_completions;
        master_bready_never();

        // Llenar hasta WR_FIFO_DEPTH pulsando en round-robin
        for (k = 0; k < WR_FIFO_DEPTH; k = k + 1) begin
            @(posedge clk);
            wr_resp_valid_tb[k % N_BANKS] <= 1'b1;
            @(posedge clk);
            wr_resp_valid_tb[k % N_BANKS] <= 1'b0;
        end
        wait_clk(2);

        check("T4: wr_resp_full=1 despues de llenar", wr_resp_full, 1'b1);

        // Stress test deliberado: push adicional con full=1.
        // Esto debería disparar la aserción a_push_full_but_valid (warning).
        // El RTL bloquea el push internamente (push_ok=0), no hay silent drop
        // en el contador, pero se DOCUMENTA como violación de contrato.
        $display("[T4-STRESS] Intentando push con full=1 — espera warning SVA a_push_full_but_valid");
        @(posedge clk);
        wr_resp_valid_tb[0] <= 1'b1;
        @(posedge clk);
        wr_resp_valid_tb[0] <= 1'b0;
        wait_clk(2);

        // Drenar todo
        drain_all();
        wait_clk(4);

        check_int("T4: bresp_received==WR_FIFO_DEPTH", bresp_received - snap_bresp, WR_FIFO_DEPTH);
        check("T4: bvalid=0 post-drain", s_axi_bvalid, 1'b0);
        check("T4: wr_resp_full=0 post-drain", wr_resp_full, 1'b0);
        wait_clk(2);

        // ============================================================
        // T5 — Push y pop simultáneos (cancel)
        // Escenario: hay una respuesta en el contador (bvalid=1,
        // bready=1 sostenido) y llega otro push en el mismo ciclo.
        // Verifica que el contador no cambia (se cancelan) y que
        // el handshake B sí se contabiliza.
        // ============================================================
        $display("\n=== T5: Push y pop simultaneos se cancelan ===");
        snap_bresp = bresp_received;

        // Cargar 1 respuesta sin que el master la acepte
        master_bready_never();
        @(posedge clk); wr_resp_valid_tb[0] <= 1'b1;
        @(posedge clk); wr_resp_valid_tb[0] <= 1'b0;
        wait_clk(1);
        check("T5: bvalid=1 pre-cancel", s_axi_bvalid, 1'b1);

        // Activar bready y hacer push simultáneo en el mismo ciclo
        @(posedge clk);
        s_axi_bready        <= 1'b1;  // pop
        wr_resp_valid_tb[1] <= 1'b1;  // push simultáneo
        @(posedge clk);
        s_axi_bready        <= 1'b0;
        wr_resp_valid_tb[1] <= 1'b0;
        wait_clk(2);

        // El handshake ocurrió → bresp_received subió
        // El push entró al mismo tiempo → el contador no cambió neto,
        // pero ahora tiene 1 respuesta nueva (la del banco 1).
        // bvalid debe seguir en 1 (push y pop se cancelaron → cnt=1→1)
        check("T5: bvalid=1 post-cancel (nueva resp pendiente)", s_axi_bvalid, 1'b1);
        check_int("T5: bresp+1 por handshake", bresp_received, snap_bresp + 1);

        // Limpiar
        drain_all();
        wait_clk(2);

        // ============================================================
        // T6 — One-hot check (sanity / contra-check SVA)
        // ADVERTENCIA: este test viola deliberadamente el contrato del
        // Scheduler (dos bancos con wr_resp_valid=1 en el mismo ciclo).
        // Su propósito es verificar que la aserción a_wr_resp_onehot
        // del módulo SVA se dispara. No es un test de funcionalidad
        // del bloque wr_response_path.
        // ============================================================
        $display("\n=== T6: One-hot violation (espera falla SVA a_wr_resp_onehot) ===");
        $display("[T6] Activando wr_resp_valid[0] y wr_resp_valid[1] simultaneamente.");
        $display("[T6] Esto debe disparar la asercion a_wr_resp_onehot en el SVA.");

        @(posedge clk);
        wr_resp_valid_tb[0] <= 1'b1;
        wr_resp_valid_tb[1] <= 1'b1;
        @(posedge clk);
        wr_resp_valid_tb[0] <= 1'b0;
        wr_resp_valid_tb[1] <= 1'b0;
        wait_clk(2);

        // Limpiar respuestas que pudieron quedar en el contador
        drain_all();
        wait_clk(2);

        // ============================================================
        // T7 — Backpressure aleatorio + carga aleatoria (stress)
        // Escenario más importante: 200 ciclos de actividad aleatoria.
        // Al final: bresp_received debe igualar bank_completions.
        // Si difieren → silent drop detectado → FAIL.
        // ============================================================
        $display("\n=== T7: Stress aleatorio 200 ciclos ===");

        // Resetear contadores para este test (no acumulado)
        // Como no podemos reiniciarlos fácilmente (ya acumulamos de T1-T6),
        // guardamos snapshots y verificamos el delta.
        snap_bresp = bresp_received;
        snap_bank  = bank_completions;

        // Correr lógica de wr_resp_valid aleatoria en fork con bready random
        fork
            // Generador de respuestas de banco (baja probabilidad, one-hot)
            begin : gen_rand_valid
                integer i;
                integer rand_val;
                integer rand_bank;
                for (i = 0; i < 200; i = i + 1) begin
                    @(posedge clk);
                    rand_val = $random;
                    // Probabilidad ~1/4 de que algún banco complete en este ciclo
                    if ((rand_val & 3) == 0) begin
                        rand_bank = rand_val[3:2] % N_BANKS;
                        wr_resp_valid_tb[rand_bank] <= 1'b1;
                    end else begin
                        for (k = 0; k < N_BANKS; k = k + 1)
                            wr_resp_valid_tb[k] <= 1'b0;
                    end
                end
                for (k = 0; k < N_BANKS; k = k + 1)
                    wr_resp_valid_tb[k] <= 1'b0;
            end

            // Generador de bready aleatorio
            master_bready_random(200);
        join

        wait_clk(2);

        // Drenar lo que quede pendiente con bready sostenido
        drain_all();
        wait_clk(10);

        $display("[T7] Bank completions delta: %0d", bank_completions - snap_bank);
        $display("[T7] BRESP received delta:   %0d", bresp_received - snap_bresp);

        if ((bresp_received - snap_bresp) !== (bank_completions - snap_bank)) begin
            $display("[FAIL] T7: SILENT DROP DETECTADO — bresp(%0d) != bank_completions(%0d)",
                     bresp_received - snap_bresp, bank_completions - snap_bank);
            errors = errors + 1;
        end else begin
            $display("[PASS] T7: no-silent-drop verificado bajo stress aleatorio");
        end
        wait_clk(2);

        // ============================================================
        // T8 — Recuperación desde full
        // Escenario: llenar hasta full; drenar parcialmente;
        // verificar que se aceptan nuevos pushes; llenar de nuevo;
        // drenar completamente.
        // ============================================================
        $display("\n=== T8: Recuperacion desde full ===");
        snap_bresp = bresp_received;
        master_bready_never();

        // Fase 1: llenar hasta full
        for (k = 0; k < WR_FIFO_DEPTH; k = k + 1) begin
            @(posedge clk);
            wr_resp_valid_tb[k % N_BANKS] <= 1'b1;
            @(posedge clk);
            wr_resp_valid_tb[k % N_BANKS] <= 1'b0;
        end
        wait_clk(2);
        check("T8: full=1 fase1", wr_resp_full, 1'b1);

        // Fase 2: drenar parcialmente (4 respuestas)
        s_axi_bready <= 1'b1;
        repeat (4) @(posedge clk);
        s_axi_bready <= 1'b0;
        wait_clk(2);
        check("T8: full=0 post-drain-parcial", wr_resp_full, 1'b0);

        // Fase 3: llenar de nuevo hasta full
        master_bready_never();
        for (k = 0; k < 4; k = k + 1) begin
            @(posedge clk);
            wr_resp_valid_tb[k % N_BANKS] <= 1'b1;
            @(posedge clk);
            wr_resp_valid_tb[k % N_BANKS] <= 1'b0;
        end
        wait_clk(2);
        check("T8: full=1 fase3 (rellenado)", wr_resp_full, 1'b1);

        // Fase 4: drenar completamente
        drain_all();
        wait_clk(4);
        check("T8: bvalid=0 post-drain-total", s_axi_bvalid, 1'b0);
        check("T8: full=0 post-drain-total", wr_resp_full, 1'b0);
        wait_clk(2);

        // ================================================================
        // REPORTE FINAL
        // ================================================================
        $display("\n==================================");
        $display(" TB wr_response_path — Resultados");
        $display("==================================");
        $display(" Bank completions: %0d", bank_completions);
        $display(" BRESP received:   %0d", bresp_received);
        $display(" Errors:           %0d", errors);
        if (errors == 0)
            $display(" *** TEST PASS ***");
        else
            $display(" *** TEST FAIL ***");
        $display("==================================\n");

        $finish;
    end

endmodule
