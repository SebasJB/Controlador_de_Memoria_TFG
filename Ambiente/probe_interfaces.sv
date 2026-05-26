// ============================================================
//  File    : probe_interfaces.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : Probe interfaces para coverage y métricas
//
//  Estas interfaces se bindean al scheduler y a cada bank
//  controller en tb_top para dar al coverage acceso pasivo
//  a señales internas sin modificar el RTL.
// ============================================================

interface scheduler_probe_if (
    input wire clk,
    input wire rst_n
);
    // Decisiones del árbitro (de scheduler.sv)
    logic grant_wr;
    logic grant_rd;
    logic same_bank;

    // Estado de FIFOs REQ (pndng)
    logic wr_req_pndng;
    logic rd_req_pndng;

    // Backpressures
    logic wr_resp_full;
    logic rob_tag_free;

    // Eventos M5
    logic wr_discard;
    logic rd_discard;

    // Contadores M5 (16 bits)
    logic [15:0] wr_err_cnt;
    logic [15:0] rd_err_cnt;
endinterface


interface bank_probe_if #(
    parameter int N_BANKS = 4
)(
    input wire clk,
    input wire rst_n
);
    logic       bank_busy   [0:N_BANKS-1];
    logic [1:0] bank_fsm_st [0:N_BANKS-1];   // 00=IDLE 01=ISSUE 10=WAIT 11=COMPLETE
endinterface


interface fifo_probe_if #(
    parameter int WR_DEPTH      = 8,
    parameter int RD_DEPTH      = 8,
    // Default = WR_DEPTH para que el subscriber y el env sigan
    // funcionando sin modificar. El tb_top puede sobreescribirlo
    // si WR_RESP_DEPTH difiere de WR_REQ_FIFO_DEPTH, pero los
    // tipos virtuales en uvm_config_db deben usar los 2
    // parámetros explícitos para que set/get coincidan.
    parameter int WR_RESP_DEPTH = WR_DEPTH
)(
    input wire clk,
    input wire rst_n
);
    logic [$clog2(WR_DEPTH+1)-1:0]      wr_req_count;
    logic [$clog2(RD_DEPTH+1)-1:0]      rd_req_count;
    logic [$clog2(WR_RESP_DEPTH+1)-1:0] wr_resp_count;
endinterface
