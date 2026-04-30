// ============================================================
//  File    : addr_decoder.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Completamente combinacional
// ============================================================
`timescale 1ns/1ps

// 1. BYTE_OFFSET_REMOVER
// req_word_addr = req_addr >> OFF_BITS  (slicing, sin lógica)
module byte_offset_remover #(
    parameter ADDR_W         = 32,
    parameter AXI_DATA_WIDTH = 32
)(
    input  wire [ADDR_W-1:0]                           req_addr,
    output wire [ADDR_W-$clog2(AXI_DATA_WIDTH/8)-1:0] req_word_addr
);
    localparam OFF_BITS = $clog2(AXI_DATA_WIDTH / 8);
    assign req_word_addr = req_addr[ADDR_W-1 : OFF_BITS];
endmodule

// 2. BANK_SELECTOR
module bank_selector #(
    parameter ADDR_W         = 32,
    parameter AXI_DATA_WIDTH = 32,
    parameter N_BANKS        = 4
)(
    input  wire [ADDR_W-$clog2(AXI_DATA_WIDTH/8)-1:0] req_word_addr,
    output wire [$clog2(N_BANKS)-1:0]                  bank_id
);
    localparam BANK_BITS = $clog2(N_BANKS);
    assign bank_id = req_word_addr[BANK_BITS-1:0];
endmodule

// 3. LOCAL_ADDR_GEN
module local_addr_gen #(
    parameter ADDR_W          = 32,
    parameter AXI_DATA_WIDTH  = 32,
    parameter N_BANKS         = 4,
    parameter BANK_SIZE_BYTES = 1024
)(
    input  wire [ADDR_W-$clog2(AXI_DATA_WIDTH/8)-1:0]            req_word_addr,
    output wire [$clog2(BANK_SIZE_BYTES/(AXI_DATA_WIDTH/8))-1:0] bank_word_addr
);
    localparam BANK_BITS       = $clog2(N_BANKS);
    localparam WORD_BYTES      = AXI_DATA_WIDTH / 8;
    localparam BANK_WORDS      = BANK_SIZE_BYTES / WORD_BYTES;
    localparam BANK_ADDR_WIDTH = $clog2(BANK_WORDS);
    assign bank_word_addr = req_word_addr[BANK_BITS+BANK_ADDR_WIDTH-1 : BANK_BITS];
endmodule

// 4. RANGE_CHECKER
module range_checker #(
    parameter ADDR_W          = 32,
    parameter AXI_DATA_WIDTH  = 32,
    parameter N_BANKS         = 4,
    parameter BANK_SIZE_BYTES = 1024
)(
    input  wire [ADDR_W-$clog2(AXI_DATA_WIDTH/8)-1:0] req_word_addr,
    output wire                                         addr_valid
);
    localparam WORD_BYTES  = AXI_DATA_WIDTH / 8;
    localparam BANK_WORDS  = BANK_SIZE_BYTES / WORD_BYTES;
    localparam TOTAL_WORDS = N_BANKS * BANK_WORDS;
    assign addr_valid = (req_word_addr < TOTAL_WORDS);
endmodule

// 5. ADDR_DECODER — top (instancia 1-4)
module addr_decoder #(
    parameter ADDR_W          = 32,
    parameter AXI_DATA_WIDTH  = 32,
    parameter N_BANKS         = 4,
    parameter BANK_SIZE_BYTES = 1024
)(
    input  wire [ADDR_W-1:0]                                      req_addr,
    output wire [$clog2(N_BANKS)-1:0]                             bank_id,
    output wire [$clog2(BANK_SIZE_BYTES/(AXI_DATA_WIDTH/8))-1:0] bank_word_addr,
    output wire                                                    addr_valid
);
    localparam OFF_BITS        = $clog2(AXI_DATA_WIDTH / 8);
    localparam WORD_ADDR_WIDTH = ADDR_W - OFF_BITS;

    wire [WORD_ADDR_WIDTH-1:0] req_word_addr;

    byte_offset_remover #(.ADDR_W(ADDR_W),.AXI_DATA_WIDTH(AXI_DATA_WIDTH))
    u_offset_remover (
        .req_addr     (req_addr),
        .req_word_addr(req_word_addr)
    );

    bank_selector #(.ADDR_W(ADDR_W),.AXI_DATA_WIDTH(AXI_DATA_WIDTH),.N_BANKS(N_BANKS))
    u_bank_selector (
        .req_word_addr(req_word_addr),
        .bank_id      (bank_id)
    );

    local_addr_gen #(.ADDR_W(ADDR_W),.AXI_DATA_WIDTH(AXI_DATA_WIDTH),
                     .N_BANKS(N_BANKS),.BANK_SIZE_BYTES(BANK_SIZE_BYTES))
    u_local_addr_gen (
        .req_word_addr (req_word_addr),
        .bank_word_addr(bank_word_addr)
    );

    range_checker #(.ADDR_W(ADDR_W),.AXI_DATA_WIDTH(AXI_DATA_WIDTH),
                    .N_BANKS(N_BANKS),.BANK_SIZE_BYTES(BANK_SIZE_BYTES))
    u_range_checker (
        .req_word_addr(req_word_addr),
        .addr_valid   (addr_valid)
    );
endmodule
