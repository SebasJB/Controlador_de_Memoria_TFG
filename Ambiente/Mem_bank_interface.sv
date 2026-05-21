// ============================================================
// 1. INTERFACE
// ============================================================
interface axi4_lite_if #(
  parameter ADDR_W = `ADDR_WIDTH,
  parameter DATA_W = `DATA_WIDTH
)(input logic clk, input logic rst_n);

  // Write Address Channel
  logic              awvalid;
  logic              awready;
  logic [ADDR_W-1:0] awaddr;
  logic [2:0]        awprot;

  // Write Data Channel
  logic              wvalid;
  logic              wready;
  logic [DATA_W-1:0] wdata;
  logic [3:0]        wstrb;

  // Write Response Channel
  logic              bvalid;
  logic              bready;
  logic [1:0]        bresp;

  // Read Address Channel
  logic              arvalid;
  logic              arready;
  logic [ADDR_W-1:0] araddr;
  logic [2:0]        arprot;

  // Read Data Channel
  logic              rvalid;
  logic              rready;
  logic [DATA_W-1:0] rdata;
  logic [1:0]        rresp;

endinterface : axi4_lite_if

