// ============================================================
// 12. TB_TOP
// ============================================================
// ============================================================
//  Resumen de tests disponibles (+UVM_TESTNAME=<nombre>):
//
//    mem_smoke_test              — 5 pares WR/RD básicos
//    mem_single_bank_stress_test — stress por banco, verifica serialización
//    mem_hazard_test             — RAW y WAR hazards en misma dirección
//    mem_multibank_test          — acceso paralelo a todos los bancos
//    mem_regression_test         — todos los patrones juntos
// ============================================================
module tb_top;
  import uvm_pkg::*;
  `include "uvm_macros.svh"

  // Parámetros de simulación
  localparam CLK_PERIOD = 10;  // ns

  // Clock y reset
  logic clk;
  logic rst_n;

  initial clk = 0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    rst_n = 0;
    repeat (8) @(posedge clk);
    rst_n = 1;
  end

  // Interfaz AXI4-Lite
  axi4_lite_if dut_if (.clk(clk), .rst_n(rst_n));

  // --------------------------------------------------------
  // DUT: Controlador de memoria bancarizado
  // Descomentar y conectar señales según el DUT real
  // --------------------------------------------------------
  // banked_mem_ctrl #(
  //   .NUM_BANKS  (`NUM_BANKS),
  //   .BANK_DEPTH (`BANK_DEPTH)
  // ) u_dut (
  //   .clk      (clk),
  //   .rst_n    (rst_n),
  //   // AW
  //   .awvalid  (dut_if.awvalid), .awready (dut_if.awready),
  //   .awaddr   (dut_if.awaddr),  .awprot  (dut_if.awprot),
  //   // W
  //   .wvalid   (dut_if.wvalid),  .wready  (dut_if.wready),
  //   .wdata    (dut_if.wdata),   .wstrb   (dut_if.wstrb),
  //   // B
  //   .bvalid   (dut_if.bvalid),  .bready  (dut_if.bready),
  //   .bresp    (dut_if.bresp),
  //   // AR
  //   .arvalid  (dut_if.arvalid), .arready (dut_if.arready),
  //   .araddr   (dut_if.araddr),  .arprot  (dut_if.arprot),
  //   // R
  //   .rvalid   (dut_if.rvalid),  .rready  (dut_if.rready),
  //   .rdata    (dut_if.rdata),   .rresp   (dut_if.rresp)
  // );

  // Registrar interfaz y lanzar test
  initial begin
    uvm_config_db #(virtual axi4_lite_if)::set(
      null, "uvm_test_top.*", "vif", dut_if
    );
    run_test();  // Test seleccionado con +UVM_TESTNAME=<nombre>
  end

  // Timeout de simulación
  initial begin
    #5_000_000;
    `uvm_fatal("TIMEOUT", "Simulación excedió el tiempo máximo permitido")
  end

endmodule : tb_top

