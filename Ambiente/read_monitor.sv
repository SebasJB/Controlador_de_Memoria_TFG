// ============================================================
//  File    : read_monitor.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Read Monitor — observa AR, R (passive)
//
//  Analysis ports:
//    - ap_ar : dispara en AR fire (captura addr esperada)
//    - ap_r  : dispara en R fire (captura rdata, rresp)
//
//  Correlación AR ↔ R: orden FIFO. AXI4-Lite garantiza que
//  las respuestas R llegan en el mismo orden que los AR.
// ============================================================

class read_monitor #(
    parameter int ADDR_W  = 32,
    parameter int DATA_W  = 32,
    parameter int N_BANKS = 4
) extends uvm_monitor;

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS) item_t;

    virtual mem_bank_interface #(ADDR_W, DATA_W) vif;

    uvm_analysis_port #(item_t) ap_ar;
    uvm_analysis_port #(item_t) ap_r;

    item_t pending_r[$];

    `uvm_component_param_utils(read_monitor #(ADDR_W, DATA_W, N_BANKS))

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap_ar = new("ap_ar", this);
        ap_r  = new("ap_r",  this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual mem_bank_interface #(ADDR_W, DATA_W))::get(
                this, "", "axi_vif", vif))
            `uvm_fatal("RD_MON", "axi_vif not set")
    endfunction

    task run_phase(uvm_phase phase);
        wait (vif.rst_n === 1'b1);
        @(vif.monitor_cb);
        fork
            monitor_ar();
            monitor_r();
        join
    endtask

    task monitor_ar();
        forever begin
            @(vif.monitor_cb);
            if (vif.monitor_cb.arvalid === 1'b1 &&
                vif.monitor_cb.arready === 1'b1) begin
                item_t item;
                item = item_t::type_id::create("ar_obs");
                item.is_write   = 1'b0;
                item.addr       = vif.monitor_cb.araddr;
                item.t_req_fire = $time;
                ap_ar.write(item);
                pending_r.push_back(item);
            end
        end
    endtask

    task monitor_r();
        forever begin
            @(vif.monitor_cb);
            if (vif.monitor_cb.rvalid === 1'b1 &&
                vif.monitor_cb.rready === 1'b1) begin
                if (pending_r.size() == 0) begin
                    `uvm_error("RD_MON",
                        "R fire without pending AR in queue")
                end else begin
                    item_t r_item;
                    r_item = pending_r.pop_front();
                    r_item.data        = vif.monitor_cb.rdata;
                    r_item.resp        = vif.monitor_cb.rresp;
                    r_item.t_resp_fire = $time;
                    ap_r.write(r_item);
                end
            end
        end
    endtask

endclass
