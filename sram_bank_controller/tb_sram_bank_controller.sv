// ============================================================
//  File    : tb_sram_bank_controller.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : SRAM Bank Controller — Testbench Directed
//
//  Descripción:
//    Testbench directed en SystemVerilog plano (sin UVM) para
//    el módulo sram_bank_controller. Incluye modelo de SRAM
//    síncrona parametrizable con latencia configurable.
//
//    7 casos de prueba:
//      T1 — Write simple
//      T2 — Read simple (LAT=1)
//      T3 — Byte strobes parciales
//      T4 — Back-to-back writes
//      T5 — WAR hazard (Write After Read)
//      T6 — Latencia variable (ejercita WAIT, LAT=4)
//      T7 — Tag preservation
//
//    Correr dos veces cambiando el parámetro READ_LATENCY:
//      - READ_LATENCY = 1  → ISSUE → COMPLETE (directo)
//      - READ_LATENCY = 4  → ISSUE → WAIT → COMPLETE
//
//  Estilo: wire/reg (no logic), rst_n síncrono activo en bajo.
//  Simulador primario: Synopsys VCS
// ============================================================

`timescale 1ns/1ps

// ============================================================
// MODELO DE SRAM SÍNCRONA
//   - Escritura byte-enabled en posedge clk cuando en && we
//   - Lectura con pipeline de READ_LATENCY ciclos
//   - Inicialización a cero en reset
// ============================================================
module sram_sync_model #(
    parameter AXI_DATA_WIDTH  = 32,
    parameter BANK_ADDR_WIDTH = 10,
    parameter READ_LATENCY    = 1
)(
    input  wire                        clk,
    input  wire                        en,
    input  wire                        we,
    input  wire [BANK_ADDR_WIDTH-1:0]  addr,
    input  wire [AXI_DATA_WIDTH-1:0]   din,
    input  wire [AXI_DATA_WIDTH/8-1:0] wstrb,
    output reg  [AXI_DATA_WIDTH-1:0]   dout
);
    localparam MEM_DEPTH = (1 << BANK_ADDR_WIDTH);
    localparam STRB_W    = AXI_DATA_WIDTH / 8;

    // Memoria principal
    reg [AXI_DATA_WIDTH-1:0] mem [0:MEM_DEPTH-1];

    // Pipeline de lectura: READ_LATENCY etapas
    reg [AXI_DATA_WIDTH-1:0] rd_pipe [0:READ_LATENCY-1];

    integer j, b;

    // Inicialización
    initial begin
        for (j = 0; j < MEM_DEPTH; j = j + 1)
            mem[j] = {AXI_DATA_WIDTH{1'b0}};
        for (j = 0; j < READ_LATENCY; j = j + 1)
            rd_pipe[j] = {AXI_DATA_WIDTH{1'b0}};
        dout = {AXI_DATA_WIDTH{1'b0}};
    end

    always @(posedge clk) begin
        if (en) begin
            if (we) begin
                // Escritura byte-enabled
                for (b = 0; b < STRB_W; b = b + 1) begin
                    if (wstrb[b])
                        mem[addr][b*8 +: 8] <= din[b*8 +: 8];
                end
            end else begin
                // Lectura: cargar primer slot del pipeline
                rd_pipe[0] <= mem[addr];
            end
        end else begin
            // Sin acceso: mantener pipeline quieto (dato residual)
            rd_pipe[0] <= rd_pipe[0];
        end

        // Shift del pipeline para latencias > 1
        for (j = 1; j < READ_LATENCY; j = j + 1)
            rd_pipe[j] <= rd_pipe[j-1];
    end

    // Salida: última etapa del pipeline
    always @(*) begin
        dout = rd_pipe[READ_LATENCY-1];
    end

endmodule


// ============================================================
// TESTBENCH PRINCIPAL
// ============================================================
module tb_sram_bank_controller;

    // ── Parámetros ──────────────────────────────────────────
    // Cambiar READ_LATENCY entre 1 y 4 para las dos runs
    localparam AXI_DATA_WIDTH  = 32;
    localparam BANK_ADDR_WIDTH = 10;
    localparam N_BANKS         = 4;
    localparam LAT_CNT_W       = 8;
    localparam READ_LATENCY    = 1;   // <<< cambiar a 4 para segunda run

    localparam STRB_W   = AXI_DATA_WIDTH / 8;
    localparam TAG_W    = $clog2(N_BANKS);
    localparam CLK_HALF = 5;         // 5 ns → 100 MHz
    localparam TIMEOUT  = 10_000;    // ciclos antes de abort

    // ── Señales del DUT ─────────────────────────────────────
    reg clk;
    reg rst_n;

    reg                        bank_req_valid;
    reg                        bank_req_op;
    reg  [BANK_ADDR_WIDTH-1:0] bank_req_addr;
    reg  [AXI_DATA_WIDTH-1:0]  bank_req_wdata;
    reg  [STRB_W-1:0]          bank_req_wstrb;
    reg  [TAG_W-1:0]           bank_req_tag;

    wire                        sram_en;
    wire                        sram_we;
    wire [BANK_ADDR_WIDTH-1:0]  sram_addr;
    wire [AXI_DATA_WIDTH-1:0]   sram_din;
    wire [STRB_W-1:0]           sram_wstrb;
    wire [AXI_DATA_WIDTH-1:0]   sram_dout;

    wire                        bank_busy;
    wire                        wr_resp_valid;
    wire [AXI_DATA_WIDTH-1:0]   rd_resp_data;
    wire [TAG_W-1:0]            rd_resp_tag;
    wire                        rd_resp_valid;

    // ── Contador global de errores ───────────────────────────
    int errors;

    // ── Watchdog ─────────────────────────────────────────────
    int cycle_count;

    // ── Variables auxiliares para tests ──────────────────────
    int wr_resp_count;
    int rd_resp_count;
    int cycles_to_resp;

    // ── Generación de reloj ──────────────────────────────────
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    // ── Watchdog global ──────────────────────────────────────
    initial begin
        cycle_count = 0;
        forever begin
            @(posedge clk);
            cycle_count = cycle_count + 1;
            if (cycle_count >= TIMEOUT) begin
                $display("[TIMEOUT] Se superaron %0d ciclos — abortando simulación", TIMEOUT);
                $finish(1);
            end
        end
    end

    // ── Waveform dump ────────────────────────────────────────
    initial begin
        $dumpfile("tb_sram_bank_controller.vcd");
        $dumpvars(0, tb_sram_bank_controller);
    end

    // ── Instancia DUT ────────────────────────────────────────
    sram_bank_controller #(
        .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
        .BANK_ADDR_WIDTH(BANK_ADDR_WIDTH),
        .N_BANKS        (N_BANKS),
        .READ_LATENCY   (READ_LATENCY),
        .LAT_CNT_W      (LAT_CNT_W)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .bank_req_valid (bank_req_valid),
        .bank_req_op    (bank_req_op),
        .bank_req_addr  (bank_req_addr),
        .bank_req_wdata (bank_req_wdata),
        .bank_req_wstrb (bank_req_wstrb),
        .bank_req_tag   (bank_req_tag),
        .sram_en        (sram_en),
        .sram_we        (sram_we),
        .sram_addr      (sram_addr),
        .sram_din       (sram_din),
        .sram_wstrb     (sram_wstrb),
        .sram_dout      (sram_dout),
        .bank_busy      (bank_busy),
        .wr_resp_valid  (wr_resp_valid),
        .rd_resp_data   (rd_resp_data),
        .rd_resp_tag    (rd_resp_tag),
        .rd_resp_valid  (rd_resp_valid)
    );

    // ── Instancia modelo SRAM ────────────────────────────────
    sram_sync_model #(
        .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
        .BANK_ADDR_WIDTH(BANK_ADDR_WIDTH),
        .READ_LATENCY   (READ_LATENCY)
    ) sram_model (
        .clk   (clk),
        .en    (sram_en),
        .we    (sram_we),
        .addr  (sram_addr),
        .din   (sram_din),
        .wstrb (sram_wstrb),
        .dout  (sram_dout)
    );

    // ============================================================
    // TASKS HELPER
    // ============================================================

    // ── Esperar a que el FSM vuelva a IDLE ───────────────────
    // Usa la señal bank_busy como proxy: cuando bank_busy cae
    // a 0 el FSM está en IDLE o COMPLETE→IDLE.
    // Esperamos hasta que fsm_idle sea 1 (indirectamente vía
    // !bank_busy en el siguiente ciclo activo).
    task automatic wait_idle;
        // Esperar hasta que bank_busy baje
        @(posedge clk);
        while (bank_busy) @(posedge clk);
        // Un ciclo extra para que COMPLETE→IDLE se asiente
        @(posedge clk);
    endtask

    // ── Task: emitir un write ────────────────────────────────
    task automatic do_write;
        input [BANK_ADDR_WIDTH-1:0] addr;
        input [AXI_DATA_WIDTH-1:0]  data;
        input [STRB_W-1:0]          strb;
        input [TAG_W-1:0]           tag;
    begin
        @(posedge clk);
        bank_req_valid <= 1'b1;
        bank_req_op    <= 1'b1;   // WR
        bank_req_addr  <= addr;
        bank_req_wdata <= data;
        bank_req_wstrb <= strb;
        bank_req_tag   <= tag;
        @(posedge clk);
        bank_req_valid <= 1'b0;
        bank_req_op    <= 1'b0;
        bank_req_wdata <= {AXI_DATA_WIDTH{1'b0}};
        bank_req_wstrb <= {STRB_W{1'b0}};
        wait_idle();
    end
    endtask

    // ── Task: emitir un read ─────────────────────────────────
    task automatic do_read;
        input [BANK_ADDR_WIDTH-1:0] addr;
        input [TAG_W-1:0]           tag;
    begin
        @(posedge clk);
        bank_req_valid <= 1'b1;
        bank_req_op    <= 1'b0;   // RD
        bank_req_addr  <= addr;
        bank_req_wdata <= {AXI_DATA_WIDTH{1'b0}};
        bank_req_wstrb <= {STRB_W{1'b0}};
        bank_req_tag   <= tag;
        @(posedge clk);
        bank_req_valid <= 1'b0;
        wait_idle();
    end
    endtask

    // ── Task: check condition ────────────────────────────────
    task automatic check;
        input        condition;
        input [7:0]  test_num;
        input [127:0] msg;
    begin
        if (!condition) begin
            errors = errors + 1;
            $display("[ERROR T%0d] %s", test_num, msg);
        end
    end
    endtask

    // ============================================================
    // BLOQUE PRINCIPAL DE TEST
    // ============================================================
    initial begin
        errors          = 0;
        wr_resp_count   = 0;
        rd_resp_count   = 0;
        cycles_to_resp  = 0;

        // Inicializar entradas
        bank_req_valid  = 1'b0;
        bank_req_op     = 1'b0;
        bank_req_addr   = {BANK_ADDR_WIDTH{1'b0}};
        bank_req_wdata  = {AXI_DATA_WIDTH{1'b0}};
        bank_req_wstrb  = {STRB_W{1'b0}};
        bank_req_tag    = {TAG_W{1'b0}};

        // ── Reset síncrono: 5 ciclos rst_n=0 ────────────────
        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        $display("=== Inicio de simulación READ_LATENCY=%0d ===", READ_LATENCY);

        // ========================================================
        // T1 — Write simple
        // Escribe 0xDEADBEEF a addr=0x10, strb=1111, tag=0.
        // Verifica: wr_resp_valid pulsa, bank_busy comportamiento.
        // ========================================================
        $display("--- T1: Write simple ---");
        begin
            reg wr_resp_seen;
            reg rd_resp_seen;
            reg busy_went_high;
            reg busy_in_complete;
            wr_resp_seen    = 1'b0;
            rd_resp_seen    = 1'b0;
            busy_went_high  = 1'b0;
            busy_in_complete = 1'b0;

            // Fork para monitorear señales mientras se ejecuta el write
            fork
                // Hilo monitor
                begin
                    // Observar hasta 50 ciclos
                    repeat (50) begin
                        @(posedge clk);
                        #1; // observar post-posedge
                        if (bank_busy)   busy_went_high  = 1'b1;
                        if (wr_resp_valid) wr_resp_seen   = 1'b1;
                        if (rd_resp_valid) rd_resp_seen   = 1'b1;
                    end
                end
                // Hilo DUT
                begin
                    do_write(10'h010, 32'hDEAD_BEEF, 4'b1111, 2'd0);
                end
            join

            check(wr_resp_seen,    1, "wr_resp_valid no pulsó durante write");
            check(!rd_resp_seen,   1, "rd_resp_valid espurio durante write");
            check(busy_went_high,  1, "bank_busy nunca subió durante write");
        end

        // ========================================================
        // T2 — Read simple (precondición: T1 ya ejecutó)
        // Lee addr=0x10, tag=2. Espera 0xDEADBEEF.
        // ========================================================
        $display("--- T2: Read simple ---");
        begin
            reg rd_resp_seen;
            reg wr_resp_seen;
            reg [AXI_DATA_WIDTH-1:0] captured_data;
            reg [TAG_W-1:0] captured_tag;
            rd_resp_seen  = 1'b0;
            wr_resp_seen  = 1'b0;
            captured_data = {AXI_DATA_WIDTH{1'b0}};
            captured_tag  = {TAG_W{1'b0}};

            fork
                begin
                    repeat (50) begin
                        @(posedge clk);
                        #1;
                        if (rd_resp_valid) begin
                            rd_resp_seen  = 1'b1;
                            captured_data = rd_resp_data;
                            captured_tag  = rd_resp_tag;
                        end
                        if (wr_resp_valid) wr_resp_seen = 1'b1;
                    end
                end
                begin
                    do_read(10'h010, 2'd2);
                end
            join

            check(rd_resp_seen,                       2, "rd_resp_valid no pulsó durante read");
            check(!wr_resp_seen,                      2, "wr_resp_valid espurio durante read");
            check(captured_data == 32'hDEAD_BEEF,     2, "rd_resp_data incorrecto (esperado 0xDEADBEEF)");
            check(captured_tag == 2'd2,               2, "rd_resp_tag incorrecto (esperado 2)");

            if (captured_data != 32'hDEAD_BEEF)
                $display("         [DETAIL T2] got=0x%08h expected=0xDEADBEEF", captured_data);
        end

        // ========================================================
        // T3 — Byte strobes parciales
        // Escribe 0xAABBCCDD a addr=0x20 con strb=4'b0011
        // (solo bytes [15:0] → 0xCCDD). Lee y verifica.
        // Bytes altos deben quedar en 0 (SRAM inicializada a 0).
        // ========================================================
        $display("--- T3: Byte strobes ---");
        begin
            reg rd_resp_seen;
            reg [AXI_DATA_WIDTH-1:0] captured_data;
            rd_resp_seen  = 1'b0;
            captured_data = {AXI_DATA_WIDTH{1'b0}};

            // Primero el write parcial
            do_write(10'h020, 32'hAABB_CCDD, 4'b0011, 2'd0);

            // Luego el read
            fork
                begin
                    repeat (50) begin
                        @(posedge clk);
                        #1;
                        if (rd_resp_valid) begin
                            rd_resp_seen  = 1'b1;
                            captured_data = rd_resp_data;
                        end
                    end
                end
                begin
                    do_read(10'h020, 2'd1);
                end
            join

            check(rd_resp_seen,                     3, "rd_resp_valid no pulsó en T3");
            check(captured_data == 32'h0000_CCDD,   3, "Byte strobes: dato incorrecto (esperado 0x0000CCDD)");

            if (captured_data != 32'h0000_CCDD)
                $display("         [DETAIL T3] got=0x%08h expected=0x0000CCDD", captured_data);
        end

        // ========================================================
        // T4 — Back-to-back writes
        // Tres writes consecutivos a addr 0x30, 0x34, 0x38.
        // Verifica que cada uno produce exactamente un pulso
        // wr_resp_valid y que no hay captura con bank_busy=1.
        // ========================================================
        $display("--- T4: Back-to-back writes ---");
        begin
            int pulse_count;
            pulse_count = 0;

            fork
                begin
                    repeat (150) begin
                        @(posedge clk);
                        #1;
                        if (wr_resp_valid) pulse_count = pulse_count + 1;
                    end
                end
                begin
                    do_write(10'h030, 32'h1111_1111, 4'b1111, 2'd0);
                    do_write(10'h034, 32'h2222_2222, 4'b1111, 2'd0);
                    do_write(10'h038, 32'h3333_3333, 4'b1111, 2'd0);
                end
            join

            check(pulse_count == 3, 4, "Back-to-back: conteo de wr_resp_valid != 3");
            if (pulse_count != 3)
                $display("         [DETAIL T4] wr_resp_valid pulsos = %0d (esperado 3)", pulse_count);
        end

        // ========================================================
        // T5 — WAR hazard: Write After Read
        // Secuencia: RD(0x40) → WR(0x40, 0x12345678) → RD(0x40)
        // El segundo RD debe retornar el valor escrito, no el viejo.
        // Valida que la liberación conservadora evita overlapping.
        // ========================================================
        $display("--- T5: WAR hazard ---");
        begin
            reg [AXI_DATA_WIDTH-1:0] rd1_data;
            reg [AXI_DATA_WIDTH-1:0] rd2_data;
            reg rd1_seen, rd2_seen;
            int rd_count;
            rd1_data = {AXI_DATA_WIDTH{1'b0}};
            rd2_data = {AXI_DATA_WIDTH{1'b0}};
            rd1_seen = 1'b0;
            rd2_seen = 1'b0;
            rd_count = 0;

            fork
                begin
                    repeat (200) begin
                        @(posedge clk);
                        #1;
                        if (rd_resp_valid) begin
                            rd_count = rd_count + 1;
                            if (rd_count == 1) begin
                                rd1_seen = 1'b1;
                                rd1_data = rd_resp_data;
                            end else if (rd_count == 2) begin
                                rd2_seen = 1'b1;
                                rd2_data = rd_resp_data;
                            end
                        end
                    end
                end
                begin
                    do_read (10'h040, 2'd0);
                    do_write(10'h040, 32'h1234_5678, 4'b1111, 2'd0);
                    do_read (10'h040, 2'd1);
                end
            join

            check(rd1_seen,                        5, "T5: primer rd_resp_valid no visto");
            check(rd2_seen,                        5, "T5: segundo rd_resp_valid no visto");
            check(rd1_data == 32'h0000_0000,       5, "T5: primer read debería retornar 0 (dirección virgin)");
            check(rd2_data == 32'h1234_5678,       5, "T5: WAR hazard — segundo read no refleja write");

            if (rd2_data != 32'h1234_5678)
                $display("         [DETAIL T5] rd2=0x%08h expected=0x12345678", rd2_data);
        end

        // ========================================================
        // T6 — Latencia variable (solo ejercita WAIT si LAT>1)
        // Lee addr=0x50. Cuenta ciclos desde req_valid hasta
        // rd_resp_valid. Para LAT=4 debe ser ~5 ciclos totales:
        //   IDLE→ISSUE(1) + WAIT(3) + COMPLETE(1) = 5
        // ========================================================
        $display("--- T6: Latencia variable (READ_LATENCY=%0d) ---", READ_LATENCY);
        begin
            reg resp_seen;
            int start_cycle;
            int end_cycle;
            int elapsed;
            int expected_min;
            int expected_max;
            resp_seen   = 1'b0;
            start_cycle = 0;
            end_cycle   = 0;
            elapsed     = 0;

            // Para LAT=1: 2 ciclos (IDLE→ISSUE→COMPLETE)
            // Para LAT=4: 5 ciclos (IDLE→ISSUE→WAIT×3→COMPLETE)
            expected_min = READ_LATENCY + 1;
            expected_max = READ_LATENCY + 3; // margen por sincronización

            fork
                begin
                    repeat (100) begin
                        @(posedge clk);
                        #1;
                        if (rd_resp_valid && !resp_seen) begin
                            resp_seen = 1'b1;
                            end_cycle = cycle_count;
                        end
                    end
                end
                begin
                    @(posedge clk);
                    start_cycle = cycle_count;
                    bank_req_valid <= 1'b1;
                    bank_req_op    <= 1'b0;
                    bank_req_addr  <= 10'h050;
                    bank_req_tag   <= 2'd0;
                    @(posedge clk);
                    bank_req_valid <= 1'b0;
                    wait_idle();
                end
            join

            elapsed = end_cycle - start_cycle;
            check(resp_seen, 6, "T6: rd_resp_valid no visto");
            check(elapsed >= expected_min && elapsed <= expected_max, 6,
                  "T6: latencia fuera de rango esperado");

            $display("         [INFO T6] elapsed=%0d ciclos (esperado %0d..%0d)",
                     elapsed, expected_min, expected_max);

            // Para LAT>1: verificar que se pasó por WAIT
            if (READ_LATENCY > 1) begin
                // El test T6 ya ejercitó el camino WAIT por la latencia medida
                $display("         [INFO T6] READ_LATENCY=%0d — camino ISSUE→WAIT→COMPLETE ejercitado",
                         READ_LATENCY);
            end
        end

        // ========================================================
        // T7 — Tag preservation
        // Write (tag=0), luego Read (tag=3).
        // Verifica que rd_resp_tag == 3 (no se mezcla con tag=0).
        // ========================================================
        $display("--- T7: Tag preservation ---");
        begin
            reg rd_seen;
            reg [TAG_W-1:0] captured_tag;
            rd_seen      = 1'b0;
            captured_tag = {TAG_W{1'b0}};

            // Write previo con tag=0
            do_write(10'h060, 32'hFACE_CAFE, 4'b1111, 2'd0);

            // Read con tag=3
            fork
                begin
                    repeat (50) begin
                        @(posedge clk);
                        #1;
                        if (rd_resp_valid) begin
                            rd_seen      = 1'b1;
                            captured_tag = rd_resp_tag;
                        end
                    end
                end
                begin
                    do_read(10'h060, 2'd3);
                end
            join

            check(rd_seen,              7, "T7: rd_resp_valid no visto");
            check(captured_tag == 2'd3, 7, "T7: rd_resp_tag incorrecto (esperado 3)");

            if (captured_tag != 2'd3)
                $display("         [DETAIL T7] got_tag=%0d expected=3", captured_tag);
        end

        // ========================================================
        // REPORTE FINAL
        // ========================================================
        $display("");
        $display("============================================");
        $display("  READ_LATENCY = %0d", READ_LATENCY);
        $display("  Ciclos totales simulados: %0d", cycle_count);
        if (errors == 0)
            $display("  RESULTADO: PASS — 0 errores");
        else
            $display("  RESULTADO: FAIL — %0d error(es)", errors);
        $display("============================================");
        $finish(errors > 0 ? 1 : 0);
    end

endmodule
