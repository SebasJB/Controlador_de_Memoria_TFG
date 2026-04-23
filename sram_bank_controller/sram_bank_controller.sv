// ============================================================
//  File    : sram_bank_controller.sv
//  Project : Banked Memory Controller — TFG ITCR
//
//  Módulos:
//    1. request_capture_reg  — latchea req en IDLE
//    2. bank_fsm             — IDLE→ISSUE→WAIT→COMPLETE→IDLE
//    3. latency_counter      — cuenta READ_LATENCY ciclos
//    4. sram_drive_logic     — combinacional → SRAM IP
//    5. rd_resp_assembler    — captura sram_dout en COMPLETE
//    6. sram_bank_controller — top que instancia todo
//
//  Decisiones de diseño:
//    - Liberación conservadora: bank_busy baja en COMPLETE
//      cuando sram_dout ya es estable → sin hazards WAR/WAW
//    - READ_LATENCY configurable en runtime (contador, no shift)
//    - is_wr/is_rd derivadas de lat_op dentro del capture reg
//    - lat_tag viaja paralelo al dato para el ROB
//    - wr_resp_valid sale en COMPLETE&&is_wr → WR Resp Collector
//    - rd_resp_valid sale en COMPLETE&&is_rd → RD Resp Mux
//
//  Tipos: wire combinacional, reg secuencial
//  Clock: clk flanco positivo
//  Reset: rst_n síncrono activo en bajo
// ============================================================

`timescale 1ns/1ps

// ============================================================
// 1. REQUEST_CAPTURE_REG
//    Latchea los campos del request en el flanco en que
//    bank_req_valid=1 y el FSM está en IDLE (fsm_idle=1).
//
//    Genera is_wr / is_rd como señales derivadas de lat_op:
//      is_wr = lat_op        (1 = escritura)
//      is_rd = ~lat_op       (0 = lectura)
//
//    lat_tag viaja como metadata de orden hacia el
//    RD Resp Assembler — nunca pasa por el FSM ni la SRAM.
// ============================================================
module request_capture_reg #(
    parameter AXI_DATA_WIDTH  = 32,
    parameter BANK_ADDR_WIDTH = 10,
    parameter N_BANKS         = 4
)(
    input  wire clk,
    input  wire rst_n,

    // Del Scheduler
    input  wire                        bank_req_valid,
    input  wire                        bank_req_op,       // 1=WR 0=RD
    input  wire [BANK_ADDR_WIDTH-1:0]  bank_req_addr,
    input  wire [AXI_DATA_WIDTH-1:0]   bank_req_wdata,
    input  wire [AXI_DATA_WIDTH/8-1:0] bank_req_wstrb,
    input  wire [$clog2(N_BANKS)-1:0]  bank_req_tag,

    // Del FSM: habilita captura solo en IDLE
    input  wire fsm_idle,

    // Registros latched
    output reg                        lat_op,
    output reg  [BANK_ADDR_WIDTH-1:0] lat_addr,
    output reg  [AXI_DATA_WIDTH-1:0]  lat_wdata,
    output reg  [AXI_DATA_WIDTH/8-1:0]lat_wstrb,
    output reg  [$clog2(N_BANKS)-1:0] lat_tag,

    // Derivadas combinacionales de lat_op
    output wire is_wr,
    output wire is_rd
);
    wire capture_en;
    assign capture_en = bank_req_valid & fsm_idle;

    always @(posedge clk) begin
        if (!rst_n) begin
            lat_op    <= 1'b0;
            lat_addr  <= {BANK_ADDR_WIDTH{1'b0}};
            lat_wdata <= {AXI_DATA_WIDTH{1'b0}};
            lat_wstrb <= {(AXI_DATA_WIDTH/8){1'b0}};
            lat_tag   <= {$clog2(N_BANKS){1'b0}};
        end else if (capture_en) begin
            lat_op    <= bank_req_op;
            lat_addr  <= bank_req_addr;
            lat_wdata <= bank_req_wdata;
            lat_wstrb <= bank_req_wstrb;
            lat_tag   <= bank_req_tag;
        end
    end

    assign is_wr = lat_op;
    assign is_rd = ~lat_op;
endmodule


// ============================================================
// 2. BANK_FSM
//    Máquina de estados de 4 estados — corazón del controlador.
//
//    Estados:
//      IDLE     : banco libre, bank_busy=0, espera req_valid
//      ISSUE    : activa issue_en 1 ciclo, banco ocupa SRAM
//      WAIT     : espera done_pulse del latency counter (RD)
//                 writes van directo ISSUE→COMPLETE
//      COMPLETE : liberación conservadora — sram_dout estable
//                 bank_busy=0, genera wr_resp_valid o rd_capture_en
//
//    Transiciones:
//      IDLE    → ISSUE   : bank_req_valid && fsm_idle
//      ISSUE   → COMPLETE: is_wr || (is_rd && READ_LATENCY==1)
//      ISSUE   → WAIT    : is_rd && READ_LATENCY>1
//      WAIT    → COMPLETE: done_pulse
//      COMPLETE→ IDLE    : siempre (1 ciclo en COMPLETE)
//
//    Salidas:
//      fsm_idle      : HIGH en IDLE → habilita capture reg
//      issue_en      : HIGH en ISSUE → habilita SRAM drive
//      bank_busy     : HIGH en ISSUE+WAIT+COMPLETE → Scheduler
//      wr_resp_valid : HIGH en COMPLETE && is_wr → WR Collector
//      rd_capture_en : HIGH en COMPLETE && is_rd → RD Assembler
// ============================================================
module bank_fsm #(
    parameter READ_LATENCY = 1    // configurable en runtime
)(
    input  wire clk,
    input  wire rst_n,

    // Condición de entrada
    input  wire bank_req_valid,
    input  wire is_wr,
    input  wire is_rd,

    // Del Latency Counter
    input  wire done_pulse,

    // Salidas de control
    output reg  fsm_idle,
    output reg  issue_en,
    output reg  bank_busy,
    output reg  wr_resp_valid,
    output reg  rd_capture_en
);
    // Codificación de estados
    localparam [1:0]
        IDLE     = 2'b00,
        ISSUE    = 2'b01,
        WAIT     = 2'b10,
        COMPLETE = 2'b11;

    reg [1:0] state;

    // ── Transición de estado ──────────────────────────────
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            case (state)
                IDLE : begin
                    if (bank_req_valid)
                        state <= ISSUE;
                end
                ISSUE : begin
                    if (is_wr)
                        state <= COMPLETE;
                    else if (is_rd) begin
                        if (READ_LATENCY == 1)
                            state <= COMPLETE;
                        else
                            state <= WAIT;
                    end
                end
                WAIT : begin
                    if (done_pulse)
                        state <= COMPLETE;
                end
                COMPLETE : begin
                    state <= IDLE;
                end
                default : state <= IDLE;
            endcase
        end
    end

    // ── Salidas (Moore) ───────────────────────────────────
    always @(*) begin
        // Defaults
        fsm_idle      = 1'b0;
        issue_en      = 1'b0;
        bank_busy     = 1'b0;
        wr_resp_valid = 1'b0;
        rd_capture_en = 1'b0;

        case (state)
            IDLE : begin
                fsm_idle  = 1'b1;
                bank_busy = 1'b0;
            end
            ISSUE : begin
                issue_en  = 1'b1;
                bank_busy = 1'b1;
            end
            WAIT : begin
                bank_busy = 1'b1;
            end
            COMPLETE : begin
                // Liberación conservadora: bank_busy=0 aquí
                bank_busy     = 1'b0;
                wr_resp_valid = is_wr;
                rd_capture_en = is_rd;
            end
            default : begin
                fsm_idle  = 1'b1;
            end
        endcase
    end
endmodule


// ============================================================
// 3. LATENCY_COUNTER
//    Contador que mide los ciclos de latencia de la SRAM.
//    Arranca cuando el FSM entra en ISSUE con is_rd=1.
//    Incrementa en WAIT cada ciclo.
//    done_pulse sube cuando cnt == READ_LATENCY-1.
//
//    Si READ_LATENCY==1: done_pulse sube inmediatamente en
//    ISSUE, el FSM va directo a COMPLETE sin pasar por WAIT.
//    En ese caso el contador ni siquiera necesita arrancar.
//
//    Configurable en runtime: el ancho del contador se calcula
//    para cubrir el máximo valor de READ_LATENCY parametrizado.
// ============================================================
module latency_counter #(
    parameter READ_LATENCY = 1,
    parameter CNT_W        = 8    // debe cubrir READ_LATENCY-1
)(
    input  wire clk,
    input  wire rst_n,

    // Arranca cuando FSM pasa a ISSUE con lectura
    input  wire start,     // = issue_en && is_rd
    input  wire counting,  // = FSM en WAIT

    output reg  done_pulse
);
    reg [CNT_W-1:0] cnt;

    always @(posedge clk) begin
        if (!rst_n) begin
            cnt        <= {CNT_W{1'b0}};
            done_pulse <= 1'b0;
        end else begin
            done_pulse <= 1'b0;  // pulso de 1 ciclo por defecto

            if (start) begin
                cnt <= {CNT_W{1'b0}};
                // Para LAT=1 el done sale inmediatamente
                if (READ_LATENCY == 1)
                    done_pulse <= 1'b1;
            end else if (counting) begin
                if (cnt == CNT_W'(READ_LATENCY - 1)) begin
                    done_pulse <= 1'b1;
                    cnt        <= {CNT_W{1'b0}};
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end
        end
    end
endmodule


// ============================================================
// 4. SRAM_DRIVE_LOGIC
//    Lógica combinacional pura — genera las señales físicas
//    hacia la IP de SRAM síncrona.
//
//    sram_en   = issue_en  ← habilitado solo en estado ISSUE
//    sram_we   = is_wr     ← 1 para escritura, 0 para lectura
//    sram_addr = lat_addr
//    sram_din  = lat_wdata
//    sram_wstrb= lat_wstrb ← máscara de bytes activos
//
//    Cuando sram_en=0 (fuera de ISSUE) las demás señales
//    tienen valores residuales del request anterior — la SRAM
//    los ignora porque en=0.
// ============================================================
module sram_drive_logic #(
    parameter AXI_DATA_WIDTH  = 32,
    parameter BANK_ADDR_WIDTH = 10
)(
    // Del FSM
    input  wire issue_en,

    // Del capture reg
    input  wire is_wr,
    input  wire [BANK_ADDR_WIDTH-1:0]  lat_addr,
    input  wire [AXI_DATA_WIDTH-1:0]   lat_wdata,
    input  wire [AXI_DATA_WIDTH/8-1:0] lat_wstrb,

    // Hacia SRAM IP
    output wire                        sram_en,
    output wire                        sram_we,
    output wire [BANK_ADDR_WIDTH-1:0]  sram_addr,
    output wire [AXI_DATA_WIDTH-1:0]   sram_din,
    output wire [AXI_DATA_WIDTH/8-1:0] sram_wstrb
);
    assign sram_en    = issue_en;
    assign sram_we    = is_wr;
    assign sram_addr  = lat_addr;
    assign sram_din   = lat_wdata;
    assign sram_wstrb = lat_wstrb;
endmodule


// ============================================================
// 5. RD_RESP_ASSEMBLER
//    Presenta los tres campos de respuesta de lectura:
//      rd_resp_data  : dato leído de la SRAM
//      rd_resp_tag   : tag de orden (del capture reg)
//      rd_resp_valid : activo en COMPLETE&&is_rd
//
//    DISEÑO COMBINACIONAL — las tres salidas son wires:
//
//    rd_resp_data = sram_dout (combinacional directo).
//      La liberación conservadora del FSM garantiza que
//      sram_dout está estable durante todo el estado COMPLETE:
//        · sram_en = issue_en, que es 0 en COMPLETE → la SRAM
//          no recibe nuevo comando y mantiene su salida.
//        · sram_addr sigue apuntando a lat_addr (registro
//          del capture reg, no cambia hasta IDLE siguiente).
//      Por eso rd_resp_data = sram_dout es válido exactamente
//      cuando rd_resp_valid = 1. El RD Resp Mux y el ROB ven
//      dato y valid alineados en el mismo ciclo de COMPLETE.
//
//    Nota de diseño: registrar rd_resp_data introduce un
//    desfase de 1 ciclo con rd_resp_valid (el registro captura
//    en el posedge de COMPLETE, pero valid ya baja en ese mismo
//    posedge al transitar a IDLE). El paso combinacional elimina
//    ese desfase sin coste en timing, ya que el path crítico
//    es sram_dout → mux → ROB, no sram_dout → FF.
// ============================================================
module rd_resp_assembler #(
    parameter AXI_DATA_WIDTH = 32,
    parameter N_BANKS        = 4
)(
    // Del FSM
    input  wire rd_capture_en,

    // De la SRAM IP
    input  wire [AXI_DATA_WIDTH-1:0]  sram_dout,

    // Del capture reg (metadata de orden)
    input  wire [$clog2(N_BANKS)-1:0] lat_tag,

    // Hacia RD Resp Mux
    output wire [AXI_DATA_WIDTH-1:0]  rd_resp_data,
    output wire [$clog2(N_BANKS)-1:0] rd_resp_tag,
    output wire                       rd_resp_valid
);
    // Dato: combinacional desde sram_dout.
    // Válido solo cuando rd_resp_valid=1 (COMPLETE&&is_rd);
    // el consumidor (ROB) solo muestrea en ese ciclo.
    assign rd_resp_data  = sram_dout;

    // Tag: combinacional directo desde capture reg
    assign rd_resp_tag   = lat_tag;

    // Valid: combinacional desde rd_capture_en
    assign rd_resp_valid = rd_capture_en;
endmodule


// ============================================================
// 6. SRAM_BANK_CONTROLLER — TOP
//    Instancia y conecta los 5 submódulos anteriores.
//    Se instancia N_BANKS veces en el top level (mem_handler).
// ============================================================
module sram_bank_controller #(
    parameter AXI_DATA_WIDTH  = 32,
    parameter BANK_ADDR_WIDTH = 10,
    parameter N_BANKS         = 4,
    parameter READ_LATENCY    = 1,
    parameter LAT_CNT_W       = 8
)(
    input  wire clk,
    input  wire rst_n,

    // ── Del Scheduler (dispatch bus) ─────────────────────
    input  wire                        bank_req_valid,
    input  wire                        bank_req_op,
    input  wire [BANK_ADDR_WIDTH-1:0]  bank_req_addr,
    input  wire [AXI_DATA_WIDTH-1:0]   bank_req_wdata,
    input  wire [AXI_DATA_WIDTH/8-1:0] bank_req_wstrb,
    input  wire [$clog2(N_BANKS)-1:0]  bank_req_tag,

    // ── Hacia SRAM IP ─────────────────────────────────────
    output wire                        sram_en,
    output wire                        sram_we,
    output wire [BANK_ADDR_WIDTH-1:0]  sram_addr,
    output wire [AXI_DATA_WIDTH-1:0]   sram_din,
    output wire [AXI_DATA_WIDTH/8-1:0] sram_wstrb,
    input  wire [AXI_DATA_WIDTH-1:0]   sram_dout,

    // ── Hacia Scheduler ───────────────────────────────────
    output wire                        bank_busy,

    // ── Hacia WR Resp Collector ───────────────────────────
    output wire                        wr_resp_valid,

    // ── Hacia RD Resp Mux ─────────────────────────────────
    output wire [AXI_DATA_WIDTH-1:0]   rd_resp_data,
    output wire [$clog2(N_BANKS)-1:0]  rd_resp_tag,
    output wire                        rd_resp_valid
);

    // ── Señales internas ──────────────────────────────────
    wire                        fsm_idle;
    wire                        issue_en;
    wire                        rd_capture_en;
    wire                        done_pulse;
    wire                        is_wr;
    wire                        is_rd;

    wire [BANK_ADDR_WIDTH-1:0]  lat_addr;
    wire [AXI_DATA_WIDTH-1:0]   lat_wdata;
    wire [AXI_DATA_WIDTH/8-1:0] lat_wstrb;
    wire [$clog2(N_BANKS)-1:0]  lat_tag;
    wire                        lat_op;

    // ── 1. Request Capture Reg ────────────────────────────
    request_capture_reg #(
        .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
        .BANK_ADDR_WIDTH(BANK_ADDR_WIDTH),
        .N_BANKS        (N_BANKS)
    ) u_capture (
        .clk           (clk),
        .rst_n         (rst_n),
        .bank_req_valid(bank_req_valid),
        .bank_req_op   (bank_req_op),
        .bank_req_addr (bank_req_addr),
        .bank_req_wdata(bank_req_wdata),
        .bank_req_wstrb(bank_req_wstrb),
        .bank_req_tag  (bank_req_tag),
        .fsm_idle      (fsm_idle),
        .lat_op        (lat_op),
        .lat_addr      (lat_addr),
        .lat_wdata     (lat_wdata),
        .lat_wstrb     (lat_wstrb),
        .lat_tag       (lat_tag),
        .is_wr         (is_wr),
        .is_rd         (is_rd)
    );

    // ── 2. Bank FSM ───────────────────────────────────────
    bank_fsm #(
        .READ_LATENCY(READ_LATENCY)
    ) u_fsm (
        .clk           (clk),
        .rst_n         (rst_n),
        .bank_req_valid(bank_req_valid),
        .is_wr         (is_wr),
        .is_rd         (is_rd),
        .done_pulse    (done_pulse),
        .fsm_idle      (fsm_idle),
        .issue_en      (issue_en),
        .bank_busy     (bank_busy),
        .wr_resp_valid (wr_resp_valid),
        .rd_capture_en (rd_capture_en)
    );

    // ── 3. Latency Counter ────────────────────────────────
    latency_counter #(
        .READ_LATENCY(READ_LATENCY),
        .CNT_W       (LAT_CNT_W)
    ) u_lat_cnt (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (issue_en & is_rd),
        .counting  (~fsm_idle & ~issue_en & ~rd_capture_en),
        .done_pulse(done_pulse)
    );

    // ── 4. SRAM Drive Logic ───────────────────────────────
    sram_drive_logic #(
        .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
        .BANK_ADDR_WIDTH(BANK_ADDR_WIDTH)
    ) u_drive (
        .issue_en (issue_en),
        .is_wr    (is_wr),
        .lat_addr (lat_addr),
        .lat_wdata(lat_wdata),
        .lat_wstrb(lat_wstrb),
        .sram_en  (sram_en),
        .sram_we  (sram_we),
        .sram_addr(sram_addr),
        .sram_din (sram_din),
        .sram_wstrb(sram_wstrb)
    );

    // ── 5. RD Resp Assembler ──────────────────────────────
    rd_resp_assembler #(
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .N_BANKS       (N_BANKS)
    ) u_rd_resp (
        .rd_capture_en(rd_capture_en),
        .sram_dout    (sram_dout),
        .lat_tag      (lat_tag),
        .rd_resp_data (rd_resp_data),
        .rd_resp_tag  (rd_resp_tag),
        .rd_resp_valid(rd_resp_valid)
    );

endmodule
