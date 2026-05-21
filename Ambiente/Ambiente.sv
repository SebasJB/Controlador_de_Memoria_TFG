// ============================================================
// 10. ENVIRONMENT
// ============================================================
class mem_ctrl_env extends uvm_env;
  `uvm_component_utils(mem_ctrl_env)

  mem_ctrl_agent      agent;
  mem_ctrl_scoreboard scoreboard;
  mem_ctrl_coverage   coverage;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    agent      = mem_ctrl_agent::type_id::create("agent", this);
    scoreboard = mem_ctrl_scoreboard::type_id::create("scoreboard", this);
    coverage   = mem_ctrl_coverage::type_id::create("coverage", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    agent.ap.connect(scoreboard.analysis_export);  // Monitor → Scoreboard
    agent.ap.connect(coverage.analysis_export);    // Monitor → Coverage
  endfunction
endclass : mem_ctrl_env


// ============================================================
// 11. TESTS
// ============================================================

// --- Test base ---
class mem_ctrl_base_test extends uvm_test;
  `uvm_component_utils(mem_ctrl_base_test)

  mem_ctrl_env env;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    env = mem_ctrl_env::type_id::create("env", this);
  endfunction

  // Helper común: correr una secuencia en el sequencer del agente
  task run_seq(uvm_sequence #(mem_ctrl_seq_item) seq);
    seq.start(env.agent.sequencer);
  endtask
endclass : mem_ctrl_base_test


// --- Test 1: Smoke — escritura y lectura básica ---
class mem_smoke_test extends mem_ctrl_base_test;
  `uvm_component_utils(mem_smoke_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    mem_write_then_read_seq seq;
    phase.raise_objection(this);
    seq = mem_write_then_read_seq::type_id::create("seq");
    void'(seq.randomize() with { num_pairs == 5; });
    run_seq(seq);
    phase.drop_objection(this);
  endtask
endclass : mem_smoke_test


// --- Test 2: Stress de un solo banco (verifica serialización) ---
class mem_single_bank_stress_test extends mem_ctrl_base_test;
  `uvm_component_utils(mem_single_bank_stress_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    mem_single_bank_seq seq;
    phase.raise_objection(this);
    // Estresar cada banco por separado
    for (int b = 0; b < `NUM_BANKS; b++) begin
      seq = mem_single_bank_seq::type_id::create($sformatf("seq_b%0d", b));
      void'(seq.randomize() with { target_bank == b; num_transactions == 30; });
      run_seq(seq);
    end
    phase.drop_objection(this);
  endtask
endclass : mem_single_bank_stress_test


// --- Test 3: Hazards RAW/WAR a la misma dirección ---
class mem_hazard_test extends mem_ctrl_base_test;
  `uvm_component_utils(mem_hazard_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    mem_hazard_seq seq;
    phase.raise_objection(this);
    seq = mem_hazard_seq::type_id::create("seq");
    run_seq(seq);
    phase.drop_objection(this);
  endtask
endclass : mem_hazard_test


// --- Test 4: Round-robin multi-banco (verifica paralelismo) ---
class mem_multibank_test extends mem_ctrl_base_test;
  `uvm_component_utils(mem_multibank_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    mem_multibank_round_robin_seq seq;
    phase.raise_objection(this);
    seq = mem_multibank_round_robin_seq::type_id::create("seq");
    void'(seq.randomize() with { rounds == 8; });
    run_seq(seq);
    phase.drop_objection(this);
  endtask
endclass : mem_multibank_test


// --- Test 5: Regresión completa (random + todos los patrones) ---
class mem_regression_test extends mem_ctrl_base_test;
  `uvm_component_utils(mem_regression_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    mem_random_seq              rnd_seq;
    mem_write_then_read_seq     wtr_seq;
    mem_single_bank_seq         sb_seq;
    mem_hazard_seq              haz_seq;
    mem_multibank_round_robin_seq mb_seq;

    phase.raise_objection(this);

    rnd_seq = mem_random_seq::type_id::create("rnd_seq");
    void'(rnd_seq.randomize());
    run_seq(rnd_seq);

    wtr_seq = mem_write_then_read_seq::type_id::create("wtr_seq");
    void'(wtr_seq.randomize());
    run_seq(wtr_seq);

    haz_seq = mem_hazard_seq::type_id::create("haz_seq");
    run_seq(haz_seq);

    for (int b = 0; b < `NUM_BANKS; b++) begin
      sb_seq = mem_single_bank_seq::type_id::create($sformatf("sb_%0d", b));
      void'(sb_seq.randomize() with { target_bank == b; });
      run_seq(sb_seq);
    end

    mb_seq = mem_multibank_round_robin_seq::type_id::create("mb_seq");
    void'(mb_seq.randomize());
    run_seq(mb_seq);

    phase.drop_objection(this);
  endtask
endclass : mem_regression_test