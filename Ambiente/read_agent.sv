// ============================================================
//  File    : read_agent.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Read Agent
// ============================================================

class read_agent #(
    parameter int ADDR_W  = 32,
    parameter int DATA_W  = 32,
    parameter int N_BANKS = 4
) extends uvm_agent;

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS) item_t;

    uvm_sequencer #(item_t)                          sequencer;
    read_driver   #(ADDR_W, DATA_W, N_BANKS)          driver;
    read_monitor  #(ADDR_W, DATA_W, N_BANKS)          monitor;

    uvm_analysis_port #(item_t) ap_ar;
    uvm_analysis_port #(item_t) ap_r;

    `uvm_component_param_utils(read_agent #(ADDR_W, DATA_W, N_BANKS))

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap_ar = new("ap_ar", this);
        ap_r  = new("ap_r",  this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        monitor = read_monitor #(ADDR_W, DATA_W, N_BANKS)::type_id::create(
                    "monitor", this);
        if (get_is_active() == UVM_ACTIVE) begin
            sequencer = uvm_sequencer #(item_t)::type_id::create(
                        "sequencer", this);
            driver    = read_driver #(ADDR_W, DATA_W, N_BANKS)::type_id::create(
                        "driver", this);
        end
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        if (get_is_active() == UVM_ACTIVE) begin
            driver.seq_item_port.connect(sequencer.seq_item_export);
        end
        monitor.ap_ar.connect(ap_ar);
        monitor.ap_r.connect(ap_r);
    endfunction

endclass
