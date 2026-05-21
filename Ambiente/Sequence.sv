// ============================================================
// 2. SEQUENCE ITEM
//    Representa una sola transacción AXI4-Lite hacia el controlador.
//    El scoreboard usará bank_id y txn_id para rastrear el ordering.
// ============================================================
class mem_ctrl_seq_item extends uvm_sequence_item;
  `uvm_object_utils_begin(mem_ctrl_seq_item)
    `uvm_field_int(txn_id,   UVM_ALL_ON)
    `uvm_field_int(is_write, UVM_ALL_ON)
    `uvm_field_int(addr,     UVM_ALL_ON)
    `uvm_field_int(wdata,    UVM_ALL_ON)
    `uvm_field_int(wstrb,    UVM_ALL_ON)
    `uvm_field_int(rdata,    UVM_ALL_ON)
    `uvm_field_int(bresp,    UVM_ALL_ON)
    `uvm_field_int(rresp,    UVM_ALL_ON)
  `uvm_object_utils_end

  // --- Campos del estímulo (enviados por el driver) ---
  static int unsigned id_counter = 0;
  int unsigned        txn_id;        // ID único para tracking de orden

  rand logic              is_write;
  rand logic [31:0]       addr;
  rand logic [31:0]       wdata;
  rand logic [3:0]        wstrb;

  // --- Campos de respuesta (capturados por el monitor) ---
  logic [31:0] rdata;
  logic [1:0]  bresp;
  logic [1:0]  rresp;

  // --- Campo calculado: banco destino (para coverage y scoreboard) ---
  int unsigned bank_id;

  // --- Constraints ---
  // Dirección alineada a 4 bytes
  constraint addr_align_c  { addr[1:0] == 2'b00; }

  // Dirección dentro del rango válido del mapa de memoria
  // Ajustar según el memory map real del DUT
  constraint addr_range_c  {
    addr inside {[32'h0000_0000 : 32'h0000_3FFC]};  // 4 bancos × 1024 words × 4 bytes
  }

  // strb válido en escrituras
  constraint wstrb_valid_c { is_write -> (wstrb != 4'h0); }

  function new(string name = "mem_ctrl_seq_item");
    super.new(name);
    txn_id  = id_counter++;
    bank_id = `GET_BANK(addr);
  endfunction

  // Recalcular bank_id después de randomizar
  function void post_randomize();
    bank_id = `GET_BANK(addr);
  endfunction

  function string convert2string();
    return $sformatf(
      "txn#%0d %s bank=%0d addr=0x%08h data=0x%08h strb=%04b | resp=%02b rdata=0x%08h",
      txn_id, is_write ? "WR" : "RD", bank_id, addr, wdata, wstrb,
      is_write ? bresp : rresp, rdata
    );
  endfunction
endclass : mem_ctrl_seq_item

// ============================================================
// 3. SEQUENCER
// ============================================================
class mem_ctrl_sequencer extends uvm_sequencer #(mem_ctrl_seq_item);
  `uvm_component_utils(mem_ctrl_sequencer)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction
endclass : mem_ctrl_sequencer

// ============================================================
// 4. SEQUENCES
//    Diferentes patrones de acceso para ejercitar el árbitro
// ============================================================

// --- 4.0 Secuencia base ---
class mem_base_seq extends uvm_sequence #(mem_ctrl_seq_item);
  `uvm_object_utils(mem_base_seq)

  function new(string name = "mem_base_seq");
    super.new(name);
  endfunction

  // Helper: enviar una escritura
  task do_write(logic [31:0] addr, logic [31:0] data, logic [3:0] strb = 4'hF);
    mem_ctrl_seq_item item = mem_ctrl_seq_item::type_id::create("wr");
    start_item(item);
    if (!item.randomize() with { is_write == 1; addr == local::addr;
                                  wdata   == local::data; wstrb == local::strb; })
      `uvm_fatal("RAND", "Write randomization falló")
    finish_item(item);
  endtask

  // Helper: enviar una lectura
  task do_read(logic [31:0] addr);
    mem_ctrl_seq_item item = mem_ctrl_seq_item::type_id::create("rd");
    start_item(item);
    if (!item.randomize() with { is_write == 0; addr == local::addr; })
      `uvm_fatal("RAND", "Read randomization falló")
    finish_item(item);
  endtask
endclass : mem_base_seq


// --- 4.1 Secuencia aleatoria: mix de lecturas y escrituras a bancos aleatorios ---
class mem_random_seq extends mem_base_seq;
  `uvm_object_utils(mem_random_seq)

  rand int unsigned num_transactions;
  constraint num_c { num_transactions inside {[20:100]}; }

  function new(string name = "mem_random_seq");
    super.new(name);
  endfunction

  task body();
    mem_ctrl_seq_item item;
    repeat (num_transactions) begin
      item = mem_ctrl_seq_item::type_id::create("item");
      start_item(item);
      if (!item.randomize())
        `uvm_fatal("RAND", "Randomization falló")
      finish_item(item);
    end
  endtask
endclass : mem_random_seq


// --- 4.2 Secuencia de banco único: muchas requests al mismo banco (ejercita serialización) ---
class mem_single_bank_seq extends mem_base_seq;
  `uvm_object_utils(mem_single_bank_seq)

  rand int unsigned target_bank;
  rand int unsigned num_transactions;
  constraint bank_c { target_bank inside {[0:`NUM_BANKS-1]}; }
  constraint num_c  { num_transactions inside {[10:50]}; }

  function new(string name = "mem_single_bank_seq");
    super.new(name);
  endfunction

  task body();
    mem_ctrl_seq_item item;
    `uvm_info("SEQ", $sformatf("Single-bank stress: banco=%0d txns=%0d",
               target_bank, num_transactions), UVM_MEDIUM)
    repeat (num_transactions) begin
      item = mem_ctrl_seq_item::type_id::create("item");
      start_item(item);
      // Forzar dirección al banco destino
      if (!item.randomize() with {
            // Banco seleccionado por bits[3:2] de la dirección
            addr[3:2]  == target_bank[1:0];
            addr[1:0]  == 2'b00;
          })
        `uvm_fatal("RAND", "Randomization falló")
      finish_item(item);
    end
  endtask
endclass : mem_single_bank_seq


// --- 4.3 Secuencia Write-then-Read: verifica integridad de datos ---
class mem_write_then_read_seq extends mem_base_seq;
  `uvm_object_utils(mem_write_then_read_seq)

  rand int unsigned num_pairs;
  constraint num_c { num_pairs inside {[5:20]}; }

  function new(string name = "mem_write_then_read_seq");
    super.new(name);
  endfunction

  task body();
    logic [31:0] wr_addr, wr_data;
    repeat (num_pairs) begin
      wr_addr = {$urandom_range(0, 16'h3FF), 2'b00};  // alinear
      wr_data = $urandom();
      do_write(wr_addr, wr_data);   // Escribir
      do_read(wr_addr);              // Leer: scoreboard verificará el valor
    end
  endtask
endclass : mem_write_then_read_seq


// --- 4.4 Secuencia multi-banco simultáneo: ejercita paralelismo del árbitro ---
//         Envía una request a CADA banco en ronda robín
class mem_multibank_round_robin_seq extends mem_base_seq;
  `uvm_object_utils(mem_multibank_round_robin_seq)

  rand int unsigned rounds;
  constraint rounds_c { rounds inside {[4:16]}; }

  function new(string name = "mem_multibank_round_robin_seq");
    super.new(name);
  endfunction

  task body();
    for (int r = 0; r < rounds; r++) begin
      for (int b = 0; b < `NUM_BANKS; b++) begin
        mem_ctrl_seq_item item = mem_ctrl_seq_item::type_id::create("item");
        start_item(item);
        if (!item.randomize() with {
              addr[3:2] == b[1:0];
              addr[1:0] == 2'b00;
            })
          `uvm_fatal("RAND", "Randomization falló")
        finish_item(item);
      end
    end
  endtask
endclass : mem_multibank_round_robin_seq


// --- 4.5 Secuencia de hazard WAR/RAW: Read-after-Write y Write-after-Read
//         a la MISMA dirección — el árbitro NO debe reordenarlas ---
class mem_hazard_seq extends mem_base_seq;
  `uvm_object_utils(mem_hazard_seq)

  function new(string name = "mem_hazard_seq");
    super.new(name);
  endfunction

  task body();
    logic [31:0] addr = 32'h0000_0010;   // misma dirección para todos

    // Patrón: W0 → R0 (RAW hazard: lectura debe ver W0)
    do_write(addr, 32'hDEAD_BEEF);
    do_read(addr);

    // Patrón: R1 → W1 → R2 (WAR hazard: R2 debe ver W1, R1 ve dato anterior)
    do_read(addr);
    do_write(addr, 32'hCAFE_BABE);
    do_read(addr);
  endtask
endclass : mem_hazard_seq
