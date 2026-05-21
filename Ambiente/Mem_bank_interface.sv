// ============================================================
//  File    : mem_bank_interface.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Verification Environment — AXI4-Lite Interface
//
//  Propósito:
//    Interfaz AXI4-Lite unificada para conectar el environment
//    UVM al DUT (mem_handler_top). Provee tres modports:
//      - master_write : drive AW/W, observa B   (write_driver)
//      - master_read  : drive AR, observa R     (read_driver)
//      - monitor      : observa todos los canales (monitors)
//
//  Decisiones de diseño:
//    - Una sola interfaz para los 5 canales AXI4-Lite. Aunque
//      tenemos dos agentes UVM separados, el bus físico es uno.
//      Los modports dan visión partitada por canal.
//    - Clocking blocks con `default input #1step output #0`
//      para evitar races entre driver y monitor. Drivers
//      escriben "en el ciclo actual" y monitor lee "del ciclo
//      anterior estabilizado".
//    - Sin awprot/arprot/awid/arid/etc. — el DUT es AXI4-Lite
//      estricto y no los implementa.
//    - wstrb parametrizado a [DATA_W/8-1:0], no [3:0].
//
//  Reset:
//    rst_n es activo en bajo, síncrono — matchea convención
//    del DUT. Drivers y monitor esperan rst_n=1 antes de
//    operar.
// ============================================================

`timescale 1ns/1ps

interface mem_bank_interface #(
    parameter int ADDR_W = 32,
    parameter int DATA_W = 32
)(
    input wire clk,
    input wire rst_n
);

    // ── Canal AW (Write Address) ─────────────────────────
    logic              awvalid;
    logic              awready;
    logic [ADDR_W-1:0] awaddr;

    // ── Canal W (Write Data) ─────────────────────────────
    logic                  wvalid;
    logic                  wready;
    logic [DATA_W-1:0]     wdata;
    logic [DATA_W/8-1:0]   wstrb;

    // ── Canal B (Write Response) ─────────────────────────
    logic       bvalid;
    logic       bready;
    logic [1:0] bresp;

    // ── Canal AR (Read Address) ──────────────────────────
    logic              arvalid;
    logic              arready;
    logic [ADDR_W-1:0] araddr;

    // ── Canal R (Read Response) ──────────────────────────
    logic              rvalid;
    logic              rready;
    logic [DATA_W-1:0] rdata;
    logic [1:0]        rresp;


    // ========================================================
    // Clocking blocks
    //
    // Convención:
    //   - input  #1step → muestreo justo antes del posedge
    //                     (valor estable del ciclo anterior)
    //   - output #0     → drive inmediato en el posedge
    //
    // Esto elimina races: el monitor siempre ve el valor que
    // el driver puso en el ciclo previo, no el del ciclo en
    // curso. Sin esto, el orden de ejecución de hilos UVM
    // puede dar resultados no deterministas en simulación.
    // ========================================================

    // ── Clocking para driver de WRITE (drive AW/W, observa B)
    clocking master_write_cb @(posedge clk);
        default input #1step output #0;
        // Drives
        output awvalid, awaddr;
        output wvalid,  wdata,  wstrb;
        output bready;
        // Samples
        input  awready;
        input  wready;
        input  bvalid,  bresp;
    endclocking

    // ── Clocking para driver de READ (drive AR, observa R)
    clocking master_read_cb @(posedge clk);
        default input #1step output #0;
        // Drives
        output arvalid, araddr;
        output rready;
        // Samples
        input  arready;
        input  rvalid,  rdata,  rresp;
    endclocking

    // ── Clocking para monitor (todo input, passive)
    clocking monitor_cb @(posedge clk);
        default input #1step;
        input awvalid, awready, awaddr;
        input wvalid,  wready,  wdata,  wstrb;
        input bvalid,  bready,  bresp;
        input arvalid, arready, araddr;
        input rvalid,  rready,  rdata,  rresp;
    endclocking


    // ========================================================
    // Modports
    //
    // master_write : write_driver — drive AW/W, accept B
    // master_read  : read_driver  — drive AR, accept R
    // monitor      : write_monitor + read_monitor — passive
    // dut          : conexión al RTL (mem_handler_top)
    // ========================================================

    modport master_write (
        clocking master_write_cb,
        input    clk,
        input    rst_n
    );

    modport master_read (
        clocking master_read_cb,
        input    clk,
        input    rst_n
    );

    modport monitor (
        clocking monitor_cb,
        input    clk,
        input    rst_n
    );

    modport dut (
        // Canal AW
        input  awvalid, awaddr,
        output awready,
        // Canal W
        input  wvalid, wdata, wstrb,
        output wready,
        // Canal B
        output bvalid, bresp,
        input  bready,
        // Canal AR
        input  arvalid, araddr,
        output arready,
        // Canal R
        output rvalid, rdata, rresp,
        input  rready,
        // Reloj y reset
        input  clk, rst_n
    );

endinterface
