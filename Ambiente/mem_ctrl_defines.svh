// ============================================================
//  File    : mem_ctrl_defines.svh
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Verification Environment — global defines
//
//  Propósito:
//    Header file con parámetros por defecto, typedefs del
//    protocolo AXI4-Lite, y funciones helper parametrizadas
//    que reemplazan los macros `GET_BANK / `GET_OFFSET del
//    environment original.
//
//  Uso:
//    `include "mem_ctrl_defines.svh" en mem_ctrl_pkg.sv y
//    en tb_top.sv. Las funciones helper son puras y
//    pueden llamarse desde cualquier scope.
//
//  Reglas de parametrización (decisión TFG):
//    - Los valores aquí son DEFAULTS. Cada clase UVM acepta
//      ADDR_W / DATA_W / N_BANKS como parámetros propios y
//      los propaga. tb_top fija los valores concretos.
//    - Solo las constantes del protocolo AXI4-Lite (códigos
//      de respuesta) son inmutables: son del estándar, no del
//      diseño.
// ============================================================

`ifndef MEM_CTRL_DEFINES_SVH
`define MEM_CTRL_DEFINES_SVH


// ============================================================
// Default parameters — pueden ser sobreescritos por tb_top
// ============================================================
// Anchos de bus del DUT (deben matchear mem_handler_top)
parameter int DEF_ADDR_W           = 32;
parameter int DEF_DATA_W           = 32;
parameter int DEF_N_BANKS          = 4;
parameter int DEF_BANK_SIZE_BYTES  = 1024;

// Parámetros de SRAM
parameter int DEF_READ_LATENCY     = 1;
parameter int DEF_LAT_CNT_W        = 8;

// Profundidades de FIFOs internas
parameter int DEF_WR_REQ_FIFO_DEPTH = 8;
parameter int DEF_RD_REQ_FIFO_DEPTH = 8;
parameter int DEF_WR_RESP_DEPTH     = 8;

// Métricas
parameter int DEF_ERR_CNT_W         = 16;


// ============================================================
// AXI4-Lite — constantes del protocolo (inmutables)
// ============================================================
// Códigos de respuesta para BRESP y RRESP
parameter bit [1:0] AXI_RESP_OKAY   = 2'b00;
parameter bit [1:0] AXI_RESP_EXOKAY = 2'b01;  // no aplica a AXI4-Lite (no exclusive)
parameter bit [1:0] AXI_RESP_SLVERR = 2'b10;
parameter bit [1:0] AXI_RESP_DECERR = 2'b11;


// ============================================================
// Typedefs auxiliares
// ============================================================
// Tipo de operación AXI capturada en seq_item.
// No es un enum sintetizable: solo se usa en clases UVM.
typedef enum bit {
    OP_READ  = 1'b0,
    OP_WRITE = 1'b1
} mem_ctrl_op_e;


// ============================================================
// Funciones helper parametrizadas
//
// Reemplazan los macros `GET_BANK / `GET_OFFSET del env
// original. Son funciones puras (sin side effects), parametri-
// zadas por ADDR_W / DATA_W / N_BANKS / BANK_SIZE_BYTES, y
// pueden llamarse desde cualquier scope (clase, módulo, prog).
//
// Política de address mapping (debe matchear addr_decoder.sv):
//   byte_offset = addr[OFF_BITS-1:0]          (siempre 0 en alignment)
//   bank_id     = addr[BANK_BITS+OFF_BITS-1:OFF_BITS]
//   bank_offset = addr[BANK_BITS+BANK_ADDR_W+OFF_BITS-1:
//                      BANK_BITS+OFF_BITS]
//
// Interleaving: palabras consecutivas → bancos distintos.
// ============================================================

// ── get_bank(addr) ──────────────────────────────────────────
// Extrae el bank_id de una dirección byte-addressable.
function automatic int get_bank(
    input bit [63:0] addr,           // ancho amplio para evitar truncado
    input int        DATA_W,
    input int        N_BANKS
);
    int word_bytes;
    int off_bits;
    int bank_bits;
    int word_addr;
    int mask;
    word_bytes = DATA_W / 8;
    off_bits   = $clog2(word_bytes);
    bank_bits  = $clog2(N_BANKS);
    word_addr  = addr >> off_bits;
    mask       = (1 << bank_bits) - 1;
    return word_addr & mask;
endfunction


// ── get_offset(addr) ────────────────────────────────────────
// Extrae el offset dentro del banco (word-addressed).
function automatic int get_offset(
    input bit [63:0] addr,
    input int        DATA_W,
    input int        N_BANKS,
    input int        BANK_SIZE_BYTES
);
    int word_bytes;
    int off_bits;
    int bank_bits;
    int bank_words;
    int bank_addr_w;
    int word_addr;
    int mask;
    word_bytes  = DATA_W / 8;
    off_bits    = $clog2(word_bytes);
    bank_bits   = $clog2(N_BANKS);
    bank_words  = BANK_SIZE_BYTES / word_bytes;
    bank_addr_w = $clog2(bank_words);
    word_addr   = addr >> off_bits;
    mask        = (1 << bank_addr_w) - 1;
    return (word_addr >> bank_bits) & mask;
endfunction


// ── is_addr_valid(addr) ─────────────────────────────────────
// Replica la lógica de range_checker.sv:
//   addr_valid = (word_addr < N_BANKS * BANK_WORDS)
// Usado por el scoreboard para predecir si el DUT descartará
// la transacción (incrementa wr_err_cnt / rd_err_cnt — M5).
function automatic bit is_addr_valid(
    input bit [63:0] addr,
    input int        DATA_W,
    input int        N_BANKS,
    input int        BANK_SIZE_BYTES
);
    int word_bytes;
    int off_bits;
    int bank_words;
    int total_words;
    int word_addr;
    word_bytes  = DATA_W / 8;
    off_bits    = $clog2(word_bytes);
    bank_words  = BANK_SIZE_BYTES / word_bytes;
    total_words = N_BANKS * bank_words;
    word_addr   = addr >> off_bits;
    return (word_addr < total_words);
endfunction


// ── total_bank_words() ──────────────────────────────────────
// Helper que devuelve el número total de palabras direcciona-
// bles por banco. Útil para construir bins de coverage.
function automatic int bank_words(
    input int DATA_W,
    input int BANK_SIZE_BYTES
);
    return BANK_SIZE_BYTES / (DATA_W / 8);
endfunction


// ── max_valid_byte_addr() ───────────────────────────────────
// Mayor dirección byte-addressable válida en el rango del DUT.
// Útil para constraints de randomización en sequences.
function automatic bit [63:0] max_valid_byte_addr(
    input int N_BANKS,
    input int BANK_SIZE_BYTES
);
    return (N_BANKS * BANK_SIZE_BYTES) - 1;
endfunction


// ============================================================
// Macros de utilidad
// ============================================================
// Macro para reportar mismatch en scoreboard con formato uniforme.
`define MEM_CTRL_REPORT_MISMATCH(tag, txn_id, bank, offset, exp, got) \
    `uvm_error(tag, $sformatf("DATA MISMATCH txn_id=%0d bank=%0d offset=0x%0h exp=0x%0h got=0x%0h", \
                              txn_id, bank, offset, exp, got))


`endif // MEM_CTRL_DEFINES_SVH
