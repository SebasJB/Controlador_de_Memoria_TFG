// ============================================================
//  File    : mem_ctrl_tests.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Tests — enfoque híbrido
//
//  4 tests:
//    1. mem_smoke_test           — bring-up (1 par WR/RD por banco)
//    2. mem_full_test            — backbone (master_seq orquesta
//                                   GENERAL/SINGLE_BANK/CONFLICT/
//                                   INVALID_ADDR/WSTRB_STRESS/
//                                   ROB_WRAP/FIFO_SAT en paralelo)
//    3. mem_b_backpressure_test  — wr_resp_full aislado
//    4. mem_r_backpressure_test  — !rob_tag_free aislado
//
//  Plusargs:
//    +PHASE_ONLY=GENERAL|SINGLE_BANK|CONFLICT|INVALID_ADDR|
//                WSTRB_STRESS|ROB_WRAP|FIFO_SAT|ALL
//      Permite saltar fases en mem_full_test para debug.
// ============================================================

// Parámetros consistentes con tb_top
parameter int TEST_ADDR_W          = 32;
parameter int TEST_DATA_W          = 32;
parameter int TEST_N_BANKS         = 4;
parameter int TEST_BANK_SIZE_BYTES = 1024;
parameter int TEST_WR_FIFO_DEPTH   = 8;
parameter int TEST_RD_FIFO_DEPTH   = 8;

typedef mem_ctrl_env #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS,
                       TEST_BANK_SIZE_BYTES,
                       TEST_WR_FIFO_DEPTH, TEST_RD_FIFO_DEPTH) env_t;


// ============================================================
// Base test
// ============================================================
class mem_base_test extends uvm_test;
    env_t env;

    `uvm_component_utils(mem_base_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        env = env_t::type_id::create("env", this);
    endfunction

    task drain_responses(int cycles = 500);
        repeat (cycles) @(posedge env.wr_agent.driver.vif.clk);
    endtask
endclass


// ============================================================
// 1. mem_smoke_test — bring-up
// ============================================================
class mem_smoke_test extends mem_base_test;
    `uvm_component_utils(mem_smoke_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction

    task run_phase(uvm_phase phase);
        mem_smoke_wr_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES) wr_seq;
        mem_smoke_rd_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES) rd_seq;
        phase.raise_objection(this);
        wr_seq = mem_smoke_wr_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES)::type_id::create("wr");
        rd_seq = mem_smoke_rd_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES)::type_id::create("rd");
        wr_seq.start(env.wr_agent.sequencer);
        drain_responses(50);
        rd_seq.start(env.rd_agent.sequencer);
        drain_responses(100);
        phase.drop_objection(this);
    endtask
endclass


// ============================================================
// 2. mem_full_test — backbone con master sequences
//   Las dos master_seq corren en paralelo y se sincronizan
//   implícitamente porque ambas pasan por las mismas fases.
//   ROB_WRAP necesita backpressure → se setea config_db antes
//   de comenzar (toda la corrida tiene r_backpressure_cycles=10,
//   suficiente para wrap pero no para colapsar).
// ============================================================
class mem_full_test extends mem_base_test;
    `uvm_component_utils(mem_full_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction

    function void build_phase(uvm_phase phase);
        
        // En build_phase de mem_full_test
        string csv_dir_arg;
        int    b_bp_arg, r_bp_arg;

        super.build_phase(phase);

        if (!$value$plusargs("CSV_DIR=%s", csv_dir_arg)) csv_dir_arg = ".";
        if (!$value$plusargs("B_BP=%d", b_bp_arg))       b_bp_arg = 0;
        if (!$value$plusargs("R_BP=%d", r_bp_arg))       r_bp_arg = 0;

        uvm_config_db#(string)::set(this, "env.sb*", "csv_dir",     csv_dir_arg);
        uvm_config_db#(int)::set   (this, "env.sb*", "b_bp_value",  b_bp_arg);
        uvm_config_db#(int)::set   (this, "env.sb*", "r_bp_value",  r_bp_arg);
        uvm_config_db#(string)::set(this, "env.sb*", "run_label",   "mem_full_test");

        uvm_config_db#(int)::set(this, "env.wr_agent.driver", "b_backpressure_cycles", b_bp_arg);
        uvm_config_db#(int)::set(this, "env.rd_agent.driver", "r_backpressure_cycles", r_bp_arg);
    endfunction

    task run_phase(uvm_phase phase);
        mem_master_wr_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES) m_wr;
        mem_master_rd_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES) m_rd;
        phase.raise_objection(this);
        m_wr = mem_master_wr_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES)::type_id::create("m_wr");
        m_rd = mem_master_rd_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES)::type_id::create("m_rd");
    
        // PARALELO: writes y reads compiten por el scheduler
//        fork
//            m_wr.start(env.wr_agent.sequencer);
//            m_rd.start(env.rd_agent.sequencer);
//        join   // espera a que AMBAS sequences terminen
//    
//        // Drain final: deja que salgan las respuestas en vuelo
//        drain_responses(2000);

        m_wr.start(env.wr_agent.sequencer);
        drain_responses(2000);
        m_rd.start(env.rd_agent.sequencer);
        drain_responses(2000);
    
        phase.drop_objection(this);
    endtask
endclass


// ============================================================
// 3. mem_b_backpressure_test — aislado para wr_resp_full
// ============================================================
class mem_b_backpressure_test extends mem_base_test;
    `uvm_component_utils(mem_b_backpressure_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        uvm_config_db#(int)::set(this, "env.wr_agent.driver",
                                  "b_backpressure_cycles", 20);
    endfunction

    task run_phase(uvm_phase phase);
        mem_phased_wr_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES) ph;
        phase.raise_objection(this);
        ph = mem_phased_wr_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES)::type_id::create("ph");
        ph.phase_name = "GENERAL"; ph.n_txns = 200;
        ph.start(env.wr_agent.sequencer);
        drain_responses(3000);
        phase.drop_objection(this);
    endtask
endclass


// ============================================================
// 4. mem_r_backpressure_test — aislado para !rob_tag_free
// ============================================================
class mem_r_backpressure_test extends mem_base_test;
    `uvm_component_utils(mem_r_backpressure_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        uvm_config_db#(int)::set(this, "env.rd_agent.driver",
                                  "r_backpressure_cycles", 30);
    endfunction

    task run_phase(uvm_phase phase);
        mem_phased_wr_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES) ph_wr;
        mem_phased_rd_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES) ph_rd;
        phase.raise_objection(this);
        // Pre-fill: writes sin stall
        ph_wr = mem_phased_wr_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES)::type_id::create("ph_wr");
        ph_wr.phase_name = "GENERAL"; ph_wr.n_txns = 100;
        ph_wr.start(env.wr_agent.sequencer);
        drain_responses(150);
        // Reads con stall agresivo
        ph_rd = mem_phased_rd_seq #(TEST_ADDR_W, TEST_DATA_W, TEST_N_BANKS, TEST_BANK_SIZE_BYTES)::type_id::create("ph_rd");
        ph_rd.phase_name = "GENERAL"; ph_rd.n_txns = 200;
        ph_rd.start(env.rd_agent.sequencer);
        drain_responses(5000);
        phase.drop_objection(this);
    endtask
endclass
