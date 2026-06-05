// ============================================================
//  File    : write_driver.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Write Driver
//
//  Knobs:
//    - b_backpressure_cycles : stall en bready antes de aceptar B.
//
//  Verbosity:
//    Avisos UVM_HIGH para debug detallado del handshake.
//    Activar con +UVM_VERBOSITY=UVM_HIGH
// ============================================================

class write_driver #(
    parameter int ADDR_W  = 32,
    parameter int DATA_W  = 32,
    parameter int N_BANKS = 4
) extends uvm_driver #(mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS));

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS) item_t;

    virtual mem_bank_interface #(ADDR_W, DATA_W) vif;

    int b_backpressure_cycles = 0;

    `uvm_component_param_utils(write_driver #(ADDR_W, DATA_W, N_BANKS))

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual mem_bank_interface #(ADDR_W, DATA_W))::get(this, "", "axi_vif", vif))
            `uvm_fatal("WR_DRV", "axi_vif not set")
        void'(uvm_config_db#(int)::get(this, "", "b_backpressure_cycles", b_backpressure_cycles));
        `uvm_info("WR_DRV", $sformatf("build_phase: b_backpressure_cycles=%0d", b_backpressure_cycles), UVM_LOW)
    endfunction

    task run_phase(uvm_phase phase);
        // Inicializar salidas
        vif.master_write_cb.awvalid <= 1'b0;
        vif.master_write_cb.wvalid  <= 1'b0;
        vif.master_write_cb.bready  <= 1'b0;
        vif.master_write_cb.awaddr  <= '0;
        vif.master_write_cb.wdata   <= '0;
        vif.master_write_cb.wstrb   <= '0;

        wait (vif.rst_n === 1'b1);
        @(vif.master_write_cb);
        `uvm_info("WR_DRV", "Reset released — driver ready", UVM_HIGH)

        fork
            accept_b_responses();
        join_none

        forever begin
            item_t req;
            seq_item_port.get_next_item(req);
            `uvm_info("WR_DRV_REQ", $sformatf("got seq_item txn_id=%0d addr=0x%08h data=0x%08h wstrb=0x%h", req.txn_id, req.addr, req.data, req.wstrb),UVM_HIGH)
            repeat (req.delay_cycles) @(vif.master_write_cb);
            drive_aw_w(req);
            seq_item_port.item_done();
        end
    endtask

    task drive_aw_w(item_t item);
        fork
            drive_aw(item);
            drive_w(item);
        join
        item.t_req_fire = $time;
        `uvm_info("WR_DRV",
            $sformatf("AW+W complete txn_id=%0d @ %0t",
                      item.txn_id, item.t_req_fire),
            UVM_HIGH)
    endtask

    task drive_aw(item_t item);
        @(vif.master_write_cb);
        vif.master_write_cb.awvalid <= 1'b1;
        vif.master_write_cb.awaddr  <= item.addr;
        do @(vif.master_write_cb); while (vif.master_write_cb.awready !== 1'b1);
        vif.master_write_cb.awvalid <= 1'b0;
        `uvm_info("WR_DRV_AW",
            $sformatf("AW fire txn_id=%0d addr=0x%08h @ %0t",
                      item.txn_id, item.addr, $time),
            UVM_HIGH)
    endtask

    task drive_w(item_t item);
        @(vif.master_write_cb);
        vif.master_write_cb.wvalid <= 1'b1;
        vif.master_write_cb.wdata  <= item.data;
        vif.master_write_cb.wstrb  <= item.wstrb;
        do @(vif.master_write_cb); while (vif.master_write_cb.wready !== 1'b1);
        vif.master_write_cb.wvalid <= 1'b0;
        `uvm_info("WR_DRV_W",
            $sformatf("W fire txn_id=%0d data=0x%08h wstrb=0x%h @ %0t",
                      item.txn_id, item.data, item.wstrb, $time),
            UVM_HIGH)
    endtask

    task accept_b_responses();
    int b_idx = 0;
    forever begin
        @(vif.master_write_cb);
        if (vif.master_write_cb.bvalid !== 1'b1) continue;

        if (b_backpressure_cycles > 0) begin
            `uvm_info("WR_DRV_B",
                $sformatf("B stall %0d cycles before accepting @ %0t",
                          b_backpressure_cycles, $time),
                UVM_LOW)
            repeat (b_backpressure_cycles) @(vif.master_write_cb);
        end

        vif.master_write_cb.bready <= 1'b1;
        @(vif.master_write_cb);
        vif.master_write_cb.bready <= 1'b0;

        `uvm_info("WR_DRV_B",
            $sformatf("B accepted #%0d bresp=%0b @ %0t",
                      b_idx, vif.master_write_cb.bresp, $time),
            UVM_HIGH)
        b_idx++;
    end
endtask

endclass
