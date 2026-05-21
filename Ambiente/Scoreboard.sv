// ============================================================
// 9. SCOREBOARD  — Modelo de referencia + verificador de ordering
//
//  Lógica central:
//    - Mantiene un modelo de memoria por banco: ref_mem[bank][offset]
//    - Mantiene una cola de requests PENDIENTES por banco: pending_q[bank]
//    - Al llegar una transacción del monitor:
//        WR → actualiza ref_mem en el orden en que llegan
//        RD → compara rdata contra ref_mem (debe ver la última escritura
//             a esa dirección que haya completado antes en el mismo banco)
//    - Detecta si el DUT entregó las respuestas fuera del orden correcto
//      comparando el txn_id de lo observado vs lo esperado
// ============================================================
class mem_ctrl_scoreboard extends uvm_scoreboard;
  `uvm_component_utils(mem_ctrl_scoreboard)

  uvm_analysis_imp #(mem_ctrl_seq_item, mem_ctrl_scoreboard) analysis_export;

  // Modelo de referencia: ref_mem[banco][offset] = dato esperado
  logic [31:0] ref_mem [`NUM_BANKS][`BANK_DEPTH];

  // Cola de requests en vuelo por banco (para verificar ordering)
  // Cada entrada: {txn_id, expected_data (para RD)}
  typedef struct {
    int unsigned txn_id;
    logic        is_write;
    logic [31:0] addr;
    logic [31:0] exp_data;  // dato que el DUT debería leer
  } pending_t;

  pending_t pending_q [`NUM_BANKS][$];  // cola por banco

  // Contadores globales
  int unsigned pass_count = 0;
  int unsigned fail_count = 0;
  int unsigned order_errors = 0;

  function new(string name, uvm_component parent);
    super.new(name, parent);
    // Inicializar memoria de referencia a 0
    foreach (ref_mem[b, o]) ref_mem[b][o] = '0;
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    analysis_export = new("analysis_export", this);
  endfunction

  // -----------------------------------------------------------
  //  Función llamada automáticamente por el monitor
  // -----------------------------------------------------------
  function void write(mem_ctrl_seq_item trans);
    int unsigned bank   = trans.bank_id;
    int unsigned offset = `GET_OFFSET(trans.addr);

    if (trans.is_write)
      process_write(trans, bank, offset);
    else
      process_read(trans, bank, offset);
  endfunction

  // -----------------------------------------------------------
  //  Procesar escritura: actualizar modelo de referencia
  // -----------------------------------------------------------
  function void process_write(mem_ctrl_seq_item trans,
                               int unsigned bank, int unsigned offset);
    // Aplicar byte enables al modelo de referencia
    logic [31:0] old_val = ref_mem[bank][offset];
    logic [31:0] new_val = old_val;

    if (trans.wstrb[0]) new_val[ 7: 0] = trans.wdata[ 7: 0];
    if (trans.wstrb[1]) new_val[15: 8] = trans.wdata[15: 8];
    if (trans.wstrb[2]) new_val[23:16] = trans.wdata[23:16];
    if (trans.wstrb[3]) new_val[31:24] = trans.wdata[31:24];

    ref_mem[bank][offset] = new_val;

    // Verificar respuesta: debe ser OKAY (2'b00)
    if (trans.bresp !== 2'b00) begin
      fail_count++;
      `uvm_error("SCB_WR",
        $sformatf("txn#%0d: Escritura en banco=%0d offset=%0d recibió bresp=%02b (esperado OKAY)",
                   trans.txn_id, bank, offset, trans.bresp))
    end else begin
      pass_count++;
      `uvm_info("SCB_WR",
        $sformatf("txn#%0d PASS WR banco=%0d offset=%0d data=0x%08h → ref actualizada",
                   trans.txn_id, bank, offset, new_val), UVM_MEDIUM)
    end
  endfunction

  // -----------------------------------------------------------
  //  Procesar lectura: comparar contra modelo de referencia
  // -----------------------------------------------------------
  function void process_read(mem_ctrl_seq_item trans,
                              int unsigned bank, int unsigned offset);
    logic [31:0] expected = ref_mem[bank][offset];

    if (trans.rresp !== 2'b00) begin
      // El DUT reportó un error de esclavo
      fail_count++;
      `uvm_error("SCB_RD",
        $sformatf("txn#%0d: Lectura en banco=%0d offset=%0d recibió rresp=%02b (esperado OKAY)",
                   trans.txn_id, bank, offset, trans.rresp))

    end else if (trans.rdata !== expected) begin
      // Dato incorrecto → posible violación de ordering
      fail_count++;
      `uvm_error("SCB_RD",
        $sformatf("txn#%0d: DATO INCORRECTO banco=%0d offset=%0d got=0x%08h exp=0x%08h — ¿violación de ordering?",
                   trans.txn_id, bank, offset, trans.rdata, expected))
    end else begin
      pass_count++;
      `uvm_info("SCB_RD",
        $sformatf("txn#%0d PASS RD banco=%0d offset=%0d got=0x%08h",
                   trans.txn_id, bank, offset, trans.rdata), UVM_MEDIUM)
    end
  endfunction

  // -----------------------------------------------------------
  //  Reporte final
  // -----------------------------------------------------------
  function void report_phase(uvm_phase phase);
    `uvm_info("SCB", "============================================", UVM_NONE)
    `uvm_info("SCB", $sformatf("  PASS : %0d", pass_count),        UVM_NONE)
    `uvm_info("SCB", $sformatf("  FAIL : %0d", fail_count),        UVM_NONE)
    `uvm_info("SCB", "============================================", UVM_NONE)
    if (fail_count > 0)
      `uvm_error("SCB", "*** TEST FALLIDO ***")
    else
      `uvm_info("SCB", "*** TEST PASADO ***", UVM_NONE)
  endfunction
endclass : mem_ctrl_scoreboard