// ============================================================
//  File    : read_driver.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Read Driver
//
//  Knobs:
//    - r_backpressure_cycles : stall en rready antes de aceptar R.
//      Default 0 = sin stall.
// ============================================================

class read_driver #(
    parameter int ADDR_W  = 32,
    parameter int DATA_W  = 32,
    parameter int N_BANKS = 4
) extends uvm_driver #(mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS));

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS) item_t;

    virtual mem_bank_interface #(ADDR_W, DATA_W) vif;

    int r_backpressure_cycles = 0;

    `uvm_component_param_utils(read_driver #(ADDR_W, DATA_W, N_BANKS))

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual mem_bank_interface #(ADDR_W, DATA_W))::get(
                this, "", "axi_vif", vif))
            `uvm_fatal("RD_DRV", "axi_vif not set")
        void'(uvm_config_db#(int)::get(this, "", "r_backpressure_cycles",
                                       r_backpressure_cycles));
    endfunction

    task run_phase(uvm_phase phase);
        vif.master_read_cb.arvalid <= 1'b0;
        vif.master_read_cb.rready  <= 1'b0;
        vif.master_read_cb.araddr  <= '0;

        wait (vif.rst_n === 1'b1);
        @(vif.master_read_cb);

        fork
            accept_r_responses();
        join_none

        forever begin
            item_t req;
            seq_item_port.get_next_item(req);
            drive_ar(req);
            seq_item_port.item_done();
        end
    endtask

    task drive_ar(item_t item);
        vif.master_read_cb.arvalid <= 1'b1;
        vif.master_read_cb.araddr  <= item.addr;
        do @(vif.master_read_cb); while (vif.master_read_cb.arready !== 1'b1);
        vif.master_read_cb.arvalid <= 1'b0;
        item.t_req_fire = $time;
    endtask

    task accept_r_responses();
        forever begin
            @(vif.master_read_cb);
            if (vif.master_read_cb.rvalid === 1'b1) begin
                if (r_backpressure_cycles > 0)
                    repeat (r_backpressure_cycles) @(vif.master_read_cb);
                vif.master_read_cb.rready <= 1'b1;
                @(vif.master_read_cb);
                vif.master_read_cb.rready <= 1'b0;
            end
        end
    endtask

endclass
