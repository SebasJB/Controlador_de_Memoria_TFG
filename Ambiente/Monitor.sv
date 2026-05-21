// ============================================================
// 6. MONITOR
//    Observa el bus en modo pasivo y publica transacciones completadas
// ============================================================
class mem_ctrl_monitor extends uvm_monitor;
  `uvm_component_utils(mem_ctrl_monitor)

  virtual axi4_lite_if vif;
  uvm_analysis_port #(mem_ctrl_seq_item) ap;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
    if (!uvm_config_db #(virtual axi4_lite_if)::get(this, "", "vif", vif))
      `uvm_fatal("NO_VIF", "Interface no encontrada en config_db")
  endfunction

  task run_phase(uvm_phase phase);
    mem_ctrl_seq_item trans;
    @(posedge vif.rst_n);  // Esperar fin de reset

    // Monitorear escrituras y lecturas en paralelo
    fork
      monitor_writes();
      monitor_reads();
    join
  endtask

  // Captura transacciones de escritura completas (AW+W+B)
  task monitor_writes();
    mem_ctrl_seq_item trans;
    forever begin
      trans = mem_ctrl_seq_item::type_id::create("wr_mon");
      trans.is_write = 1;

      // Capturar AW channel
      do @(posedge vif.clk); while (!(vif.awvalid && vif.awready));
      trans.addr = vif.awaddr;

      // Capturar W channel
      do @(posedge vif.clk); while (!(vif.wvalid && vif.wready));
      trans.wdata = vif.wdata;
      trans.wstrb = vif.wstrb;

      // Capturar B channel (respuesta)
      do @(posedge vif.clk); while (!(vif.bvalid && vif.bready));
      trans.bresp   = vif.bresp;
      trans.bank_id = `GET_BANK(trans.addr);

      `uvm_info("MON", trans.convert2string(), UVM_HIGH)
      ap.write(trans);
    end
  endtask

  // Captura transacciones de lectura completas (AR+R)
  task monitor_reads();
    mem_ctrl_seq_item trans;
    forever begin
      trans = mem_ctrl_seq_item::type_id::create("rd_mon");
      trans.is_write = 0;

      // Capturar AR channel
      do @(posedge vif.clk); while (!(vif.arvalid && vif.arready));
      trans.addr = vif.araddr;

      // Capturar R channel
      do @(posedge vif.clk); while (!(vif.rvalid && vif.rready));
      trans.rdata   = vif.rdata;
      trans.rresp   = vif.rresp;
      trans.bank_id = `GET_BANK(trans.addr);

      `uvm_info("MON", trans.convert2string(), UVM_HIGH)
      ap.write(trans);
    end
  endtask
endclass : mem_ctrl_monitor