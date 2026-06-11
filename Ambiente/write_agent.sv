// ============================================================
//  File    : write_agent.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Write Agent — sequencer + driver + monitor
//
//  Configuración: ACTIVE por defecto (drives + observes).
//  Para uso PASSIVE (solo monitor) configurar is_active=UVM_PASSIVE
//  vía uvm_config_db antes de build_phase.
// ============================================================

class write_agent #(
    parameter int ADDR_W  = 32,
    parameter int DATA_W  = 32,
    parameter int N_BANKS = 4,
    parameter int BANK_SIZE_BYTES = 8192
) extends uvm_agent;

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS) item_t;

    uvm_sequencer #(item_t)                          sequencer;
    write_driver  #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES)          driver;
    write_monitor #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES)          monitor;

    // Re-exposición de los analysis ports del monitor (atajo
    // para que el env conecte directamente al scoreboard/coverage)
    uvm_analysis_port #(item_t) ap_wr;
    uvm_analysis_port #(item_t) ap_b;

    `uvm_component_param_utils(write_agent #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap_wr = new("ap_wr", this);
        ap_b  = new("ap_b",  this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        monitor = write_monitor #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES)::type_id::create(
                    "monitor", this);
        if (get_is_active() == UVM_ACTIVE) begin
            sequencer = uvm_sequencer #(item_t)::type_id::create(
                        "sequencer", this);
            driver    = write_driver #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES)::type_id::create(
                        "driver", this);
        end
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        if (get_is_active() == UVM_ACTIVE) begin
            driver.seq_item_port.connect(sequencer.seq_item_export);
        end
        // Re-exportar los analysis ports del monitor
        monitor.ap_wr.connect(ap_wr);
        monitor.ap_b.connect(ap_b);
    endfunction

endclass
