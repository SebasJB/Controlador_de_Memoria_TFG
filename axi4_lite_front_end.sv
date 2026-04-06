// ============================================================
//  File    : axi4_lite_front_end.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : AXI4-Lite Front End
//
//  Módulos:
//    1. aw_ready_ctrl       — lógica combinacional awready
//    2. aw_capture_reg      — registro awaddr + aw_hold_valid
//    3. w_ready_ctrl        — lógica combinacional wready
//    4. w_capture_reg       — registro wdata/wstrb + w_hold_valid
//    5. wr_write_check_logic— condición de push WR
//    6. wr_request_formatter— bus packed {op,addr,data,strb}
//    7. wr_request_assembler— wrapper de 5 y 6
//    8. rd_request_formatter— canal AR combinacional
//    9. axi4_lite_front_end — top
//
//
//  Clocking : flanco positivo (clk)
//  Reset    : síncrono activo en bajo (rst_n)
// ============================================================

`timescale 1ns/1ps

// ============================================================
// 1. AW_READY_CTRL
//    Lógica combinacional pura — sin registros internos.
//    Se niega aw_hold_valid porque si ya hay una dirección
//    retenida no se puede aceptar otra hasta que el assembler
//    haga push (wr_req_push limpia aw_hold_valid).
// ============================================================
module aw_ready_ctrl (
    input  wire aw_hold_valid,   // desde aw_capture_reg
    input  wire wr_req_full,     // backpressure desde WR REQ FIFO
    output wire s_axi_awready
);
    assign s_axi_awready = !aw_hold_valid && !wr_req_full;
endmodule


// ============================================================
// 2. AW_CAPTURE_REG
//    Registra s_axi_awaddr en el flanco de aw_fire.
//    Nota: si aw_fire y wr_req_push ocurren simultáneamente,
//    el nuevo dato se captura y valid se mantiene en 1
//    (aw_fire tiene prioridad sobre el clear).
// ============================================================
module aw_capture_reg #(
    parameter ADDR_W = 32
)(
    input  wire              clk,
    input  wire              rst_n,
    // Canal AW
    input  wire              s_axi_awvalid,
    input  wire              s_axi_awready,
    input  wire [ADDR_W-1:0] s_axi_awaddr,
    // Control desde assembler
    input  wire              wr_req_push,
    // Salidas hacia assembler
    output reg  [ADDR_W-1:0] aw_hold_addr,
    output reg               aw_hold_valid
);
    wire aw_fire;
    assign aw_fire = s_axi_awvalid & s_axi_awready;

    always @(posedge clk) begin
        if (!rst_n) begin
            aw_hold_addr  <= {ADDR_W{1'b0}};
            aw_hold_valid <= 1'b0;
        end else begin
            if (aw_fire) begin
                aw_hold_addr  <= s_axi_awaddr;
                aw_hold_valid <= 1'b1;
            end else if (wr_req_push) begin
                aw_hold_valid <= 1'b0;
            end
        end
    end
endmodule


// ============================================================
// 3. W_READY_CTRL
//    Lógica combinacional pura — sin registros internos.
// ============================================================
module w_ready_ctrl (
    input  wire w_hold_valid,    // desde w_capture_reg
    input  wire wr_req_full,     // backpressure desde WR REQ FIFO
    output wire s_axi_wready
);
    assign s_axi_wready = !w_hold_valid && !wr_req_full;
endmodule

// ============================================================
// 4. W_CAPTURE_REG
//    Registra wdata y wstrb en el flanco de w_fire.
//    Misma prioridad que AW: w_fire gana sobre wr_req_push.
// ============================================================
module w_capture_reg #(
    parameter AXI_DATA_WIDTH = 32
)(
    input  wire                          clk,
    input  wire                          rst_n,
    // Canal W
    input  wire                          s_axi_wvalid,
    input  wire                          s_axi_wready,
    input  wire [AXI_DATA_WIDTH-1:0]     s_axi_wdata,
    input  wire [AXI_DATA_WIDTH/8-1:0]   s_axi_wstrb,
    // Control desde assembler
    input  wire                          wr_req_push,
    // Salidas hacia assembler
    output reg  [AXI_DATA_WIDTH-1:0]     w_hold_data,
    output reg  [AXI_DATA_WIDTH/8-1:0]   w_hold_strb,
    output reg                           w_hold_valid
);
    wire w_fire;
    assign w_fire = s_axi_wvalid & s_axi_wready;

    always @(posedge clk) begin
        if (!rst_n) begin
            w_hold_data  <= {AXI_DATA_WIDTH{1'b0}};
            w_hold_strb  <= {(AXI_DATA_WIDTH/8){1'b0}};
            w_hold_valid <= 1'b0;
        end else begin
            if (w_fire) begin
                w_hold_data  <= s_axi_wdata;
                w_hold_strb  <= s_axi_wstrb;
                w_hold_valid <= 1'b1;
            end else if (wr_req_push) begin
                w_hold_valid <= 1'b0;
            end
        end
    end
endmodule


// ============================================================
// 5. WR_WRITE_CHECK_LOGIC
//    Submódulo del Write Request Assembler.
//    Salida: wr_req_push — controla el push al WR REQ FIFO
//    y el clear de aw_hold_valid / w_hold_valid.
// ============================================================
module wr_write_check_logic (
    input  wire aw_hold_valid,
    input  wire w_hold_valid,
    input  wire wr_req_full,
    output wire wr_req_push
);
    assign wr_req_push = aw_hold_valid & w_hold_valid & !wr_req_full;
endmodule


// ============================================================
// 6. WR_REQUEST_FORMATTER
//    Submódulo del Write Request Assembler:
//      wr_req_data.op   = WR  (1'b1)
//      wr_req_data.addr = aw_hold_addr
//      wr_req_data.data = w_hold_data
//      wr_req_data.strb = w_hold_strb
//
//    Se implementa como bus packed concatenado:
//      wr_req_data = {op[0], addr[ADDR_W-1:0],
//                     data[DATA_W-1:0], strb[STRB_W-1:0]}
// ============================================================
module wr_request_formatter #(
    parameter ADDR_W         = 32,
    parameter AXI_DATA_WIDTH = 32
)(
    input  wire [ADDR_W-1:0]                                       aw_hold_addr,
    input  wire [AXI_DATA_WIDTH-1:0]                               w_hold_data,
    input  wire [AXI_DATA_WIDTH/8-1:0]                             w_hold_strb,
    // Bus packed hacia WR REQ FIFO
    // MSB → LSB: {op(1), addr(ADDR_W), data(DATA_W), strb(STRB_W)}
    output wire [ADDR_W + AXI_DATA_WIDTH + AXI_DATA_WIDTH/8 : 0]  wr_req_data
);
    // op = 1'b1 → WR (constante cableada)
    assign wr_req_data = {1'b1, aw_hold_addr, w_hold_data, w_hold_strb};
endmodule


// ============================================================
// 7. WR_REQUEST_ASSEMBLER
//    Wrapper que instancia Write Check Logic (5) y
//    WR Request Formatter (6)
// ============================================================
module wr_request_assembler #(
    parameter ADDR_W         = 32,
    parameter AXI_DATA_WIDTH = 32
)(
    // Desde capture regs
    input  wire                                                    aw_hold_valid,
    input  wire [ADDR_W-1:0]                                       aw_hold_addr,
    input  wire                                                    w_hold_valid,
    input  wire [AXI_DATA_WIDTH-1:0]                               w_hold_data,
    input  wire [AXI_DATA_WIDTH/8-1:0]                             w_hold_strb,
    // Backpressure
    input  wire                                                    wr_req_full,
    // Hacia WR REQ FIFO
    output wire                                                    wr_req_push,
    output wire [ADDR_W + AXI_DATA_WIDTH + AXI_DATA_WIDTH/8 : 0]  wr_req_data
);
    wr_write_check_logic u_check (
        .aw_hold_valid (aw_hold_valid),
        .w_hold_valid  (w_hold_valid),
        .wr_req_full   (wr_req_full),
        .wr_req_push   (wr_req_push)
    );

    wr_request_formatter #(
        .ADDR_W        (ADDR_W),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) u_formatter (
        .aw_hold_addr  (aw_hold_addr),
        .w_hold_data   (w_hold_data),
        .w_hold_strb   (w_hold_strb),
        .wr_req_data   (wr_req_data)
    );
endmodule


// ============================================================
// 8. RD_REQUEST_FORMATTER
//    Canal AR
//      rd_req_push          = arvalid && !rd_req_full
//      rd_req_data.op       = RD (1'b0)
//      rd_req_data.addr     = s_axi_araddr
//
//    Bus packed: {op(1), addr(ADDR_W)}
// ============================================================
module rd_request_formatter #(
    parameter ADDR_W = 32
)(
    // Canal AR
    input  wire              s_axi_arvalid,
    input  wire [ADDR_W-1:0] s_axi_araddr,
    // Backpressure
    input  wire              rd_req_full,
    // Hacia RD REQ FIFO
    output wire              s_axi_arready,
    output wire              rd_req_push,
    output wire [ADDR_W:0]   rd_req_data    // {op(1), addr(ADDR_W)}
);
    assign s_axi_arready = !rd_req_full;
    assign rd_req_push   = s_axi_arvalid & s_axi_arready;
    // op = 1'b0 → RD
    assign rd_req_data   = {1'b0, s_axi_araddr};
endmodule


// ============================================================
// 9. AXI4_LITE_FRONT_END — TOP
//    Instancia y conecta todos los módulos anteriores.
//    Las señales internas se declaran como wire (combinacionales)
//    o reg (registradas en capture regs, propagadas como wire).
// ============================================================
module axi4_lite_front_end #(
    parameter ADDR_W         = 32,
    parameter AXI_DATA_WIDTH = 32
)(
    input  wire clk,
    input  wire rst_n,

    // ── AXI4-Lite AW Channel ─────────────────────────────
    input  wire              s_axi_awvalid,
    output wire              s_axi_awready,
    input  wire [ADDR_W-1:0] s_axi_awaddr,

    // ── AXI4-Lite W Channel ──────────────────────────────
    input  wire                        s_axi_wvalid,
    output wire                        s_axi_wready,
    input  wire [AXI_DATA_WIDTH-1:0]   s_axi_wdata,
    input  wire [AXI_DATA_WIDTH/8-1:0] s_axi_wstrb,

    // ── AXI4-Lite AR Channel ─────────────────────────────
    input  wire              s_axi_arvalid,
    output wire              s_axi_arready,
    input  wire [ADDR_W-1:0] s_axi_araddr,

    // ── Hacia WR REQ FIFO ────────────────────────────────
    output wire                                                   wr_req_push,
    output wire [ADDR_W + AXI_DATA_WIDTH + AXI_DATA_WIDTH/8 : 0] wr_req_data,
    input  wire                                                   wr_req_full,

    // ── Hacia RD REQ FIFO ────────────────────────────────
    output wire              rd_req_push,
    output wire [ADDR_W:0]   rd_req_data,
    input  wire              rd_req_full
);

    // ── Señales internas ─────────────────────────────────
    // Salidas de capture regs (reg internamente, wire aquí)
    wire [ADDR_W-1:0]           aw_hold_addr;
    wire                        aw_hold_valid;
    wire [AXI_DATA_WIDTH-1:0]   w_hold_data;
    wire [AXI_DATA_WIDTH/8-1:0] w_hold_strb;
    wire                        w_hold_valid;

    // ── 1. AW Ready Ctrl ─────────────────────────────────
    aw_ready_ctrl u_aw_ready_ctrl (
        .aw_hold_valid (aw_hold_valid),
        .wr_req_full   (wr_req_full),
        .s_axi_awready (s_axi_awready)
    );

    // ── 2. AW Capture Reg ────────────────────────────────
    aw_capture_reg #(
        .ADDR_W(ADDR_W)
    ) u_aw_capture_reg (
        .clk           (clk),
        .rst_n         (rst_n),
        .s_axi_awvalid (s_axi_awvalid),
        .s_axi_awready (s_axi_awready),
        .s_axi_awaddr  (s_axi_awaddr),
        .wr_req_push   (wr_req_push),
        .aw_hold_addr  (aw_hold_addr),
        .aw_hold_valid (aw_hold_valid)
    );

    // ── 3. W Ready Ctrl ──────────────────────────────────
    w_ready_ctrl u_w_ready_ctrl (
        .w_hold_valid  (w_hold_valid),
        .wr_req_full   (wr_req_full),
        .s_axi_wready  (s_axi_wready)
    );

    // ── 4. W Capture Reg ─────────────────────────────────
    w_capture_reg #(
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) u_w_capture_reg (
        .clk           (clk),
        .rst_n         (rst_n),
        .s_axi_wvalid  (s_axi_wvalid),
        .s_axi_wready  (s_axi_wready),
        .s_axi_wdata   (s_axi_wdata),
        .s_axi_wstrb   (s_axi_wstrb),
        .wr_req_push   (wr_req_push),
        .w_hold_data   (w_hold_data),
        .w_hold_strb   (w_hold_strb),
        .w_hold_valid  (w_hold_valid)
    );

    // ── 7. WR Request Assembler (contiene 5 y 6) ─────────
    wr_request_assembler #(
        .ADDR_W        (ADDR_W),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) u_wr_assembler (
        .aw_hold_valid (aw_hold_valid),
        .aw_hold_addr  (aw_hold_addr),
        .w_hold_valid  (w_hold_valid),
        .w_hold_data   (w_hold_data),
        .w_hold_strb   (w_hold_strb),
        .wr_req_full   (wr_req_full),
        .wr_req_push   (wr_req_push),
        .wr_req_data   (wr_req_data)
    );

    // ── 8. RD Request Formatter ──────────────────────────
    rd_request_formatter #(
        .ADDR_W(ADDR_W)
    ) u_rd_formatter (
        .s_axi_arvalid (s_axi_arvalid),
        .s_axi_araddr  (s_axi_araddr),
        .rd_req_full   (rd_req_full),
        .s_axi_arready (s_axi_arready),
        .rd_req_push   (rd_req_push),
        .rd_req_data   (rd_req_data)
    );

endmodule
