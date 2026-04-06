
//////////////////////////////////////////////////////////
// Definition of a D flip flop with asyncronous reset  //
/////////////////////////////////////////////////////////

module dff_async_rst (
  input data,
  input clk,
  input reset,
  output reg q);

  always @ ( posedge clk or posedge reset)    
    if (reset) begin
      q <= 1'b0;
    end  else begin
      q <= data;
    end

endmodule

//////////////////////////////////////////////////////////
// Definition of a D Latch  with asyncronous reset  //
/////////////////////////////////////////////////////////

module dltch_async_rst (
  input data,
  input clk,
  input reset,
  output reg q);

  always @ (clk or reset or data)    
    if (reset) begin
      q <= 1'b0;
    end  else if (clk) begin
      q <= data;
    end

endmodule

//////////////////////////////////////////////////////////
// Definition of a D Latch  without  reset  //
/////////////////////////////////////////////////////////

module dltch (
  input data,
  input clk,
  output reg q);

  always @ (clk or data)    
    if (clk) begin
      q <= data;
    end

endmodule

///////////////////////////////////////////////////////////////////////
// Definition of the prll D register with flops
///////////////////////////////////////////////////////////////////////

module prll_d_reg #(parameter bits = 32)(
  input clk,
  input reset,
  input [bits-1:0] D_in,
  output [bits-1:0] D_out
);
  genvar i;
  generate
    for(i = 0; i < bits; i=i+1) begin:bit_
      dff_async_rst prll_regstr_(.data(D_in[i]),.clk(clk),.reset(reset),.q(D_out[i]));
    end
  endgenerate

endmodule

///////////////////////////////////////////////////////////////////////
// Definition of the prll D register with Lathces 
///////////////////////////////////////////////////////////////////////

module prll_d_ltch_no_rst #(parameter bits = 32)(
  input clk,
  input [bits-1:0] D_in,
  output [bits-1:0] D_out
);
  genvar i;
  generate
    for(i = 0; i < bits; i=i+1) begin:bit_
      dltch prll_regstr_(.data(D_in[i]),.clk(clk),.q(D_out[i]));
    end
  endgenerate

endmodule

///////////////////////////////////////////////////////////////////////
// Definition of the prll D register with Lathces 
///////////////////////////////////////////////////////////////////////

module prll_d_ltch #(parameter bits = 32)(
  input clk,
  input reset,
  input [bits-1:0] D_in,
  output [bits-1:0] D_out
);
  genvar i;
  generate
    for(i = 0; i < bits; i=i+1) begin:bit_
      dltch_async_rst prll_regstr_(.data(D_in[i]),.clk(clk),.reset(reset),.q(D_out[i]));
    end
  endgenerate

endmodule


///////////////////////////////////////////////////////////////////////
// Definition of the FIFO with Flip_Flops 
///////////////////////////////////////////////////////////////////////
module fifo_flops #(parameter depth = 16,parameter bits = 32)(
  input [bits-1:0] Din,
  output reg [bits-1:0] Dout,
  input push,
  input pop,
  input clk,
  output reg full,
  output reg pndng,
  input rst
);
  wire [bits-1:0] q[depth-1:0];
  reg [$clog2(depth):0] count;
  reg [bits-1:0] aux_mux [depth-1:0];
  reg [bits-1:0] aux_mux_or [depth-2:0];

  genvar i;
  generate
    for(i=0;i<depth;i=i+1)begin:_dp_
       if(i==0)begin: _dp2_
         prll_d_reg #(bits) D_reg(.clk(push),.reset(rst),.D_in(Din),.D_out(q[i]));
         always@(*)begin
           aux_mux[i]=(count==i+1)?q[i]:{bits{1'b0}};
         end    
       end else begin: _dp3_
         prll_d_reg #(bits) D_reg(.clk(push),.reset(rst),.D_in(q[i-1]),.D_out(q[i]));
         always@(*)begin
           aux_mux[i]=(count==i+1)?q[i]:{bits{1'b0}};
         end    
       end
    end
  endgenerate

  generate
  for(i=0;i<depth-2;i=i+1)begin:_nu_
    always@(*)begin
      aux_mux_or[i]=aux_mux[i] | aux_mux_or[i+1];
    end
  end
  endgenerate

  always@(*)begin
    aux_mux_or[depth-2] = aux_mux [depth-1]|aux_mux[depth-2];
    Dout=aux_mux_or[0];  
  end

  always@(posedge clk)begin
  if(rst) begin
    count <= 0;
  end else begin
  
    case({push,pop})
      2'b00: count <= count;
      2'b01: begin
        if(count == 0) begin
          count <= 0;
        end else begin
          count <=count - 1;
        end
      end
      2'b10:begin
         if(count == depth)begin
           count <= count;
         end else begin
           count <= count+1;
        end
      end
      2'b11: count <= count;
    endcase
  end
  pndng <= (count==0)?{1'b0}:{1'b1};
  full <=(count == depth)?{1'b1}:{1'b0};
end
endmodule

module fifo_generic #(
  parameter  DataWidth = 32,
  parameter  Depth     = 8,
  localparam PtrWidth  = $clog2(Depth)
) (
  input  logic                 clk,
  input  logic                 rst,
  input  logic                 writeEn,
  input  logic [DataWidth-1:0] writeData,
  input  logic                 readEn,
  output logic [DataWidth-1:0] readData,
  output logic                 full,
  output logic                 pndng
);

  logic [DataWidth-1:0] mem[Depth];
  logic [PtrWidth:0] wrPtr, wrPtrNext;
  logic [PtrWidth:0] rdPtr, rdPtrNext;
  logic empty;
  always_comb begin
    wrPtrNext = wrPtr;
    rdPtrNext = rdPtr;
    if (writeEn) begin
      wrPtrNext = wrPtr + 1;
    end
    if (readEn) begin
      rdPtrNext = rdPtr + 1;
    end
  end

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      wrPtr <= '0;
      rdPtr <= '0;
    end else begin
      wrPtr <= wrPtrNext;
      rdPtr <= rdPtrNext;
    end
    if (writeEn) begin
      mem[wrPtr[PtrWidth-1:0]] <= writeData;
    end
  end

  assign readData = mem[rdPtr[PtrWidth-1:0]];

  assign empty = (wrPtr[PtrWidth] == rdPtr[PtrWidth]) && (wrPtr[PtrWidth-1:0] == rdPtr[PtrWidth-1:0]);
  assign full  = (wrPtr[PtrWidth] != rdPtr[PtrWidth]) && (wrPtr[PtrWidth-1:0] == rdPtr[PtrWidth-1:0]);
  assign pndng = !empty;

endmodule
