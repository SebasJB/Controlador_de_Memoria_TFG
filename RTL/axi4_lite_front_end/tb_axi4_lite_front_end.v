// ============================================================
//  File    : tb_axi4_lite_front_end.v
//  Project : Banked Memory Controller — TFG ITCR
//  Bring-up testbench — AXI4-Lite Front End (v2 timing corregido)
//
//  Correcciones vs v1:
//    - awready/wready se chequean ANTES del fire (no después)
//    - wr_req_push se chequea JUSTO DESPUÉS del tick que seteó
//      ambos holds, no un tick después (ya estaba consumido)
//    - T2/T3: push chequeado en el mismo tick del segundo fire
//    - T7: misma corrección que T1
//
//  Principio clave de timing:
//    wr_req_push es COMBINACIONAL — sube en el mismo ciclo en
//    que los hold_valids quedan en 1 (después del posedge que
//    los registró). En el SIGUIENTE posedge ya se limpian los
//    holds, por lo que push baja. El testbench debe chequear
//    push ENTRE esos dos posedges.
// ============================================================

`timescale 1ns/1ps

module tb_axi4_lite_front_end;

    parameter ADDR_W         = 32;
    parameter AXI_DATA_WIDTH = 32;
    parameter CLK_PERIOD     = 10;

    // ── Señales DUT ──────────────────────────────────────
    reg  clk, rst_n;

    reg  [ADDR_W-1:0]           s_axi_awaddr;
    reg                         s_axi_awvalid;
    wire                        s_axi_awready;

    reg  [AXI_DATA_WIDTH-1:0]   s_axi_wdata;
    reg  [AXI_DATA_WIDTH/8-1:0] s_axi_wstrb;
    reg                         s_axi_wvalid;
    wire                        s_axi_wready;

    reg  [ADDR_W-1:0]           s_axi_araddr;
    reg                         s_axi_arvalid;
    wire                        s_axi_arready;

    wire [ADDR_W+AXI_DATA_WIDTH+AXI_DATA_WIDTH/8:0] wr_req_data;
    wire                        wr_req_push;
    reg                         wr_req_full;

    wire [ADDR_W:0]             rd_req_data;
    wire                        rd_req_push;
    reg                         rd_req_full;

    integer pass_count = 0;
    integer fail_count = 0;

    // ── DUT ──────────────────────────────────────────────
    axi4_lite_front_end #(
        .ADDR_W(ADDR_W), .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_wvalid(s_axi_wvalid),   .s_axi_wready(s_axi_wready),
        .s_axi_wdata(s_axi_wdata),     .s_axi_wstrb(s_axi_wstrb),
        .s_axi_arvalid(s_axi_arvalid), .s_axi_arready(s_axi_arready),
        .s_axi_araddr(s_axi_araddr),
        .wr_req_push(wr_req_push),     .wr_req_data(wr_req_data),
        .wr_req_full(wr_req_full),
        .rd_req_push(rd_req_push),     .rd_req_data(rd_req_data),
        .rd_req_full(rd_req_full)
    );

    // ── Clock ────────────────────────────────────────────
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ── Helpers ──────────────────────────────────────────
    task tick; @(posedge clk); #1; endtask

    task check;
        input [255:0] msg;
        input         cond;
        begin
            if (cond) begin
                $display("  [PASS] %s", msg);
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] %s  (t=%0t)", msg, $time);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // ── Extracción bus packed WR ──────────────────────────
    // Bus: {op(1), addr(ADDR_W), data(DATA_W), strb(STRB_W)}
    localparam STRB_W   = AXI_DATA_WIDTH / 8;
    localparam WR_BUS_W = 1 + ADDR_W + AXI_DATA_WIDTH + STRB_W;

    `define WR_OP   wr_req_data[WR_BUS_W-1]
    `define WR_ADDR wr_req_data[WR_BUS_W-2          -: ADDR_W]
    `define WR_DATA wr_req_data[AXI_DATA_WIDTH+STRB_W-1 -: AXI_DATA_WIDTH]
    `define WR_STRB wr_req_data[STRB_W-1            : 0]

    // ── Reset ────────────────────────────────────────────
    task do_reset;
        begin
            rst_n         = 0;
            s_axi_awvalid = 0; s_axi_awaddr = 0;
            s_axi_wvalid  = 0; s_axi_wdata  = 0; s_axi_wstrb = 0;
            s_axi_arvalid = 0; s_axi_araddr = 0;
            wr_req_full   = 0; rd_req_full  = 0;
            tick; tick;
            rst_n = 1;
            tick;
        end
    endtask

    // ═══════════════════════════════════════════════════
    // MAIN
    // ═══════════════════════════════════════════════════
    initial begin
        $dumpfile("tb_axi4_lite_front_end.vcd");
        $dumpvars(0, tb_axi4_lite_front_end);
        do_reset;

        // ─────────────────────────────────────────────
        // T1: Write simple — AW y W simultáneos
        //
        // Timing correcto:
        //   1. Chequear ready ANTES de presentar valids
        //   2. Presentar valids y hacer tick → fire ocurre,
        //      holds quedan en 1, push sube (combinacional)
        //   3. Chequear push INMEDIATAMENTE después del tick
        //      (antes de bajar valids y hacer otro tick)
        //   4. Bajar valids, tick → holds se limpian, push baja
        //   5. Chequear estado limpio
        // ─────────────────────────────────────────────
        $display("\n--- T1: Write simple (AW y W simultaneos) ---");

        // Paso 1: ready antes del fire
        check("awready=1 antes de presentar valids", s_axi_awready);
        check("wready=1 antes de presentar valids",  s_axi_wready);

        // Paso 2: presentar AW y W, tick — fire ocurre
        s_axi_awvalid <= 1; s_axi_awaddr <= 32'hDEAD_0004;
        s_axi_wvalid  <= 1; s_axi_wdata  <= 32'hCAFE_BABE;
                            s_axi_wstrb  <= 4'hF;
        tick;
        // Paso 3: push es combinacional — ya está activo
        check("wr_req_push=1 justo tras fire de AW+W", wr_req_push);
        check("op=WR(1) en bus packed",   `WR_OP  == 1'b1);
        check("addr=DEAD_0004 en packed", `WR_ADDR == 32'hDEAD_0004);
        check("data=CAFE_BABE en packed", `WR_DATA == 32'hCAFE_BABE);
        check("strb=F en packed",         `WR_STRB == 4'hF);

        // Paso 4: bajar valids, tick — holds se limpian
        s_axi_awvalid <= 0; s_axi_wvalid <= 0;
        tick;

        // Paso 5: estado limpio
        check("wr_req_push=0 tras limpiar holds",  !wr_req_push);
        check("awready=1 tras limpiar aw_hold",     s_axi_awready);
        check("wready=1 tras limpiar w_hold",       s_axi_wready);

        // ─────────────────────────────────────────────
        // T2: Write separado — AW llega primero
        // ─────────────────────────────────────────────
        $display("\n--- T2: Write separado (AW primero, W despues) ---");

        // Presentar AW
        s_axi_awvalid <= 1; s_axi_awaddr <= 32'hAAAA_0008;
        tick; // AW fire
        s_axi_awvalid <= 0;

        // Después del fire de AW: hold capturado, W aún no llega
        check("no push aun (W no ha llegado)", !wr_req_push);
        check("awready=0 con aw_hold_valid=1", !s_axi_awready);
        check("wready=1 esperando W",           s_axi_wready);

        // Presentar W
        s_axi_wvalid <= 1; s_axi_wdata <= 32'h1234_5678;
                           s_axi_wstrb <= 4'hA;
        tick; // W fire — ahora ambos holds en 1 → push inmediato
        // Chequear push en este mismo ciclo (antes de bajar wvalid)
        check("wr_req_push=1 tras W fire",      wr_req_push);
        check("addr correcta en T2", `WR_ADDR == 32'hAAAA_0008);
        check("data correcta en T2", `WR_DATA == 32'h1234_5678);
        check("strb correcta en T2", `WR_STRB == 4'hA);

        s_axi_wvalid <= 0;
        tick; // holds limpios
        check("push limpio tras T2", !wr_req_push);

        // ─────────────────────────────────────────────
        // T3: Write separado — W llega primero
        // ─────────────────────────────────────────────
        $display("\n--- T3: Write separado (W primero, AW despues) ---");

        s_axi_wvalid <= 1; s_axi_wdata <= 32'hBEEF_DEAD;
                           s_axi_wstrb <= 4'hC;
        tick; // W fire
        s_axi_wvalid <= 0;

        check("no push aun (AW no ha llegado)", !wr_req_push);
        check("wready=0 con w_hold_valid=1",    !s_axi_wready);
        check("awready=1 esperando AW",          s_axi_awready);

        s_axi_awvalid <= 1; s_axi_awaddr <= 32'hCCCC_000C;
        tick; // AW fire — ambos holds en 1 → push
        check("wr_req_push=1 tras AW fire",     wr_req_push);
        check("addr correcta en T3", `WR_ADDR == 32'hCCCC_000C);
        check("data correcta en T3", `WR_DATA == 32'hBEEF_DEAD);
        check("strb correcta en T3", `WR_STRB == 4'hC);

        s_axi_awvalid <= 0;
        tick;
        check("push limpio tras T3", !wr_req_push);

        // ─────────────────────────────────────────────
        // T4: Read simple
        // ─────────────────────────────────────────────
        $display("\n--- T4: Read simple ---");

        check("arready=1 antes del fire", s_axi_arready);

        s_axi_arvalid <= 1; s_axi_araddr <= 32'h0000_0010;
        // rd_req_push es completamente combinacional —
        // sube EN EL MISMO ciclo que arvalid & arready
        // No hace falta tick para verlo
        #1; // pequeño delta para que combinacional se propague
        check("rd_req_push=1 mismo ciclo fire (comb)", rd_req_push);
        check("addr RD correcta", rd_req_data[ADDR_W-1:0] == 32'h0000_0010);
        check("op=RD(0) en packed", rd_req_data[ADDR_W] == 1'b0);

        tick;
        s_axi_arvalid <= 0;
        tick;
        check("rd_req_push=0 tras bajar arvalid", !rd_req_push);

        // ─────────────────────────────────────────────
        // T5: Backpressure WR
        // ─────────────────────────────────────────────
        $display("\n--- T5: Backpressure WR (wr_req_full=1) ---");

        wr_req_full <= 1;
        tick;
        check("awready=0 con wr_req_full=1", !s_axi_awready);
        check("wready=0 con wr_req_full=1",  !s_axi_wready);

        // Intento de write con FIFO llena — awready=0 bloquea fire
        s_axi_awvalid <= 1; s_axi_awaddr <= 32'hFFFF_0000;
        s_axi_wvalid  <= 1; s_axi_wdata  <= 32'hDEAD_BEEF;
                            s_axi_wstrb  <= 4'hF;
        tick;
        check("no fire ni push con full=1", !wr_req_push);

        s_axi_awvalid <= 0; s_axi_wvalid <= 0;
        wr_req_full   <= 0;
        tick;
        check("awready=1 tras bajar wr_req_full", s_axi_awready);
        check("wready=1 tras bajar wr_req_full",  s_axi_wready);

        // ─────────────────────────────────────────────
        // T6: Backpressure RD
        // ─────────────────────────────────────────────
        $display("\n--- T6: Backpressure RD (rd_req_full=1) ---");

        rd_req_full <= 1;
        tick;
        check("arready=0 con rd_req_full=1", !s_axi_arready);

        s_axi_arvalid <= 1; s_axi_araddr <= 32'hEEEE_0014;
        tick;
        check("no rd_req_push con rd_req_full=1", !rd_req_push);
        s_axi_arvalid <= 0;
        rd_req_full   <= 0;
        tick;
        check("arready=1 tras bajar rd_req_full", s_axi_arready);

        // ─────────────────────────────────────────────
        // T7: Write y Read simultáneos
        // ─────────────────────────────────────────────
        $display("\n--- T7: Write y Read simultaneos ---");

        s_axi_awvalid <= 1; s_axi_awaddr <= 32'h0000_0020;
        s_axi_wvalid  <= 1; s_axi_wdata  <= 32'h9876_5432;
                            s_axi_wstrb  <= 4'hF;
        s_axi_arvalid <= 1; s_axi_araddr <= 32'h0000_0024;
        tick;
        // RD: push combinacional inmediato
        check("rd_req_push=1 en ciclo fire AR",      rd_req_push);
        check("rd addr correcta en T7", rd_req_data[ADDR_W-1:0] == 32'h0000_0024);
        // WR: push también sube combinacionalmente tras el tick
        check("wr_req_push=1 mismo ciclo fire AW+W", wr_req_push);
        check("wr addr correcta en T7", `WR_ADDR == 32'h0000_0020);
        check("wr data correcta en T7", `WR_DATA == 32'h9876_5432);

        s_axi_awvalid <= 0; s_axi_wvalid <= 0; s_axi_arvalid <= 0;
        tick;
        check("wr_req_push=0 limpio en T7", !wr_req_push);
        check("rd_req_push=0 limpio en T7", !rd_req_push);

        // ─────────────────────────────────────────────
        // Resumen
        // ─────────────────────────────────────────────
        $display("\n==========================================");
        $display("  RESULTADOS: %0d PASS  /  %0d FAIL",
                 pass_count, fail_count);
        $display("==========================================");
        if (fail_count == 0)
            $display("  *** BRING-UP OK ***\n");
        else
            $display("  *** HAY FALLOS — ver VCD ***\n");

        $finish;
    end

    initial begin #20000; $display("TIMEOUT"); $finish; end

endmodule
