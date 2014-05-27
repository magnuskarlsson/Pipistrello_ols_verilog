//--------------------------------------------------------------------------------
//
// sram_interface.v
// Copyright (C) 2011 Ian Davis
// Copyright (C) 2013 Magnus Karlsson
// 
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin St, Fifth Floor, Boston, MA 02110, USA
//
//--------------------------------------------------------------------------------
//
// Details: 
//   http://www.dangerousprototypes.com/ols
//   http://www.gadgetfactory.net/gf/project/butterflylogic
//   http://www.mygizmos.org/ols
//
// Writes data to SRAM incrementally, fully filling a 32bit word before
// moving onto the next one.   On reads, pulls data back out in reverse
// order (to maintain SUMP client compatability).  But really, backwards?!!?
//
//--------------------------------------------------------------------------------
//
// 10/05/2013 - Magnus Karlsson - Rewritten to use LPDDR for buffer memory
//

`timescale 1ns/100ps

module ram_interface(
  sys_clk, 
  sys_reset, 
  wrFlags, 
  config_data,
  write, 
  lastwrite, 
  read, 
  wrdata, 
  rddata, 
  rdvalid,
  clock,
  reset,
  c3_calib_done,
  mcb3_dram_dq,
  mcb3_dram_a,
  mcb3_dram_ba,
  mcb3_dram_cke,
  mcb3_dram_ras_n,
  mcb3_dram_cas_n,
  mcb3_dram_we_n,
  mcb3_dram_dm,
  mcb3_dram_udqs,
  mcb3_rzq,
  mcb3_dram_udm,
  mcb3_dram_dqs,
  mcb3_dram_ck,
  mcb3_dram_ck_n);

input sys_clk;
input sys_reset;
input wrFlags;
input [3:0] config_data;
input read, write, lastwrite;
input [31:0] wrdata;
output [31:0] rddata;
output [3:0] rdvalid;
output clock;
output reset;
output c3_calib_done;
inout  [15:0] mcb3_dram_dq;
output [12:0] mcb3_dram_a;
output [1:0] mcb3_dram_ba;
output mcb3_dram_cke;
output mcb3_dram_ras_n;
output mcb3_dram_cas_n;
output mcb3_dram_we_n;
output mcb3_dram_dm;
inout mcb3_dram_udqs;
inout mcb3_rzq;
output mcb3_dram_udm;
inout mcb3_dram_dqs;
output mcb3_dram_ck;
output mcb3_dram_ck_n;

//
// Registers...
//
reg init, next_init;
reg [1:0] mode, next_mode;
reg [3:0] validmask, next_validmask;

reg [7:0] clkenb, next_clkenb;
reg [23:0] address, next_address;
reg [3:0] rdvalid, next_rdvalid;

reg write_block, next_write_block;
reg read_block, next_read_block;
reg [22:0] dram_address, next_dram_address;
  
reg [2:0] state, next_state;
reg [5:0] burst_cnt, next_burst_cnt;
reg [10:0] ramb_addr, next_ramb_addr;
reg ramb_en, next_ramb_en;
reg [63:0] ramb_wdata, next_ramb_wdata;
reg ramb_we, next_ramb_we;
wire [63:0] ramb_rdata;

reg c3_p0_cmd_en, next_p0_cmd_en;
reg [2:0] c3_p0_cmd_instr, next_p0_cmd_instr;
reg [29:0] c3_p0_cmd_byte_addr, next_p0_cmd_byte_addr;
reg c3_p0_wr_en, next_p0_wr_en;
reg c3_p0_rd_en, next_p0_rd_en;

wire [63:0] c3_p0_rd_data;
wire [63:0] c3_p0_wr_data;
wire c3_p0_rd_empty;

wire uppersel = |clkenb[7:4];

//
// Control logic...
//

always @ (posedge clock or posedge reset)
begin
  if (reset) begin
    init = 1'b0;
    mode = 2'b00;
    validmask = 4'hF;
    clkenb = 8'b00001111;
    address = 0;
    rdvalid = 4'b0000;
    write_block = 1'b0;
    read_block = 1'b0;
    dram_address = 24'd0;
  end else begin
    init = next_init;
    mode = next_mode;
    validmask = next_validmask;
    clkenb = next_clkenb;
    address = next_address;
    rdvalid = next_rdvalid;
    write_block = next_write_block;
    read_block = next_read_block;
    dram_address = next_dram_address;
  end
end


always @*
begin
  #1;
  next_init = 1'b0;
  next_mode = mode;
  next_validmask = validmask;

  next_clkenb = clkenb;
  next_address = address;
  next_rdvalid = (uppersel ? clkenb[7:4] : clkenb[3:0]) & validmask;
  next_write_block = 1'b0;
  next_read_block = 1'b0;
  next_dram_address = dram_address;
  //
  // Setup architecture of RAM based on which groups are enabled/disabled.
  //   If any one group is selected, 24k samples are possible.
  //   If any two groups are selected, 12k samples are possible.
  //   If three or four groups are selected, only 6k samples are possible.
  //
  if (wrFlags)
    begin
      next_init = 1'b1;
      next_mode = 0; // 32 bit wide, 6k deep  +  24 bit wide, 6k deep
      case (config_data)
        4'b1100, 4'b0011, 4'b0110, 4'b1001, 4'b1010, 4'b0101 : next_mode = 2'b10; // 16 bit wide
        4'b1110, 4'b1101, 4'b1011, 4'b0111 : next_mode = 2'b01; // 8 bit wide
      endcase

      // The clkenb register normally indicates which bytes are valid during a read.
      // However in 24-bit mode, all 32-bits of BRAM are being used.  Thus we need to
      // tweak things a bit.  Since data is aligned (see data_align.v), all we need 
      // do is ignore the MSB here...
      next_validmask = 4'hF;
      case (config_data)
        4'b0001, 4'b0010, 4'b0100, 4'b1000 : next_validmask = 4'h7;
      endcase
    end

  //
  // Handle writes & reads.  Fill a given line of RAM completely before
  // moving onward.   
  //
  // This differs from the original SUMP storage which wrapped around 
  // before changing clock enables.  Client sees no difference. However, 
  // it'll eventally allow easier streaming of data to the client...
  //
  casex ({write && !lastwrite, read})
    2'b1x : // inc clkenb/address on all but last write (to avoid first read being bogus)...
      begin
        next_clkenb = {clkenb[3:0], clkenb[7:4]};
        casex (mode[1:0])
          2'bx1 : next_clkenb = {clkenb[6:0],clkenb[7]};   // 8 bit
          2'b1x : next_clkenb = {clkenb[5:0],clkenb[7:6]}; // 16 bit
        endcase
        if (clkenb[7]) begin
          next_address = address + 1'b1;
          if (address[5:0] == 6'h3f) begin
            next_write_block = 1'b1;
            next_dram_address = {address[22:6], 6'h00};
          end
        end
      end

    2'bx1 : 
      begin
        next_clkenb = {clkenb[3:0], clkenb[7:4]};
        casex (mode[1:0])
          2'bx1 : next_clkenb = {clkenb[0],clkenb[7:1]};   // 8 bit
          2'b1x : next_clkenb = {clkenb[1:0],clkenb[7:2]}; // 16 bit
        endcase
        if (clkenb[0]) begin
          next_address = address - 1'b1;
          if (address[5:0] == 6'h00) begin
            next_read_block = 1'b1;
            next_dram_address = address[22:0] - 12'd2048;
          end
        end
      end
  endcase

  //
  // Reset clock enables & ram address...
  //
  if (init) 
    begin
      next_clkenb = 8'b00001111; 
      casex (mode[1:0])
        2'bx1 : next_clkenb = 8'b00000001; // 1 byte writes
        2'b1x : next_clkenb = 8'b00000011; // 2 byte writes
      endcase
      next_address = 0;
    end
end


//
// Prepare RAM input data.  Present write data to all four lanes of RAM.
//
reg [63:0] rama_datain;
always @*
begin
  #1;
  rama_datain = {wrdata[31:0], wrdata[31:0]};
  casex (mode[1:0])
    2'bx1 : rama_datain = {wrdata[7:0],wrdata[7:0],wrdata[7:0],wrdata[7:0],wrdata[7:0],wrdata[7:0],wrdata[7:0],wrdata[7:0]}; // 8 bit memory
    2'b1x : rama_datain = {wrdata[15:0],wrdata[15:0],wrdata[15:0],wrdata[15:0]}; // 16 bit memory
  endcase
end

wire [10:0] #1 rama_address = address[10:0];
wire #1 rama_we = write;

wire [63:0] rama_dataout;


  RAMB16_S9_S9 RAMBG0 (
    // port A
    .DIA(rama_datain[7:0]),
    .DIPA(1'b0),
    .ADDRA(rama_address),
    .ENA(clkenb[0]),
    .WEA(rama_we),
    .SSRA(1'b0),
    .CLKA(clock),
    .DOA(rama_dataout[7:0]),
    .DOPA(),
    // port B
    .DIB(ramb_wdata[7:0]),
    .DIPB(1'b0),
    .ADDRB(ramb_addr),
    .ENB(ramb_en),
    .WEB(ramb_we),
    .SSRB(1'b0),
    .CLKB(clock),
    .DOB(ramb_rdata[7:0]), 
    .DOPB());

  RAMB16_S9_S9 RAMBG1 (
    // port A
    .DIA(rama_datain[15:8]),
    .DIPA(1'b0),
    .ADDRA(rama_address),
    .ENA(clkenb[1]),
    .WEA(rama_we),
    .SSRA(1'b0),
    .CLKA(clock),
    .DOA(rama_dataout[15:8]),
    .DOPA(),
    // port B
    .DIB(ramb_wdata[15:8]),
    .DIPB(1'b0),
    .ADDRB(ramb_addr),
    .ENB(ramb_en),
    .WEB(ramb_we),
    .SSRB(1'b0),
    .CLKB(clock),
    .DOB(ramb_rdata[15:8]), 
    .DOPB());

  RAMB16_S9_S9 RAMBG2 (
    // port A
    .DIA(rama_datain[23:16]),
    .DIPA(1'b0),
    .ADDRA(rama_address),
    .ENA(clkenb[2]),
    .WEA(rama_we),
    .SSRA(1'b0),
    .CLKA(clock),
    .DOA(rama_dataout[23:16]),
    .DOPA(),
    // port B
    .DIB(ramb_wdata[23:16]),
    .DIPB(1'b0),
    .ADDRB(ramb_addr),
    .ENB(ramb_en),
    .WEB(ramb_we),
    .SSRB(1'b0),
    .CLKB(clock),
    .DOB(ramb_rdata[23:16]), 
    .DOPB());

  RAMB16_S9_S9 RAMBG3 (
    // port A
    .DIA(rama_datain[31:24]),
    .DIPA(1'b0),
    .ADDRA(rama_address),
    .ENA(clkenb[3]),
    .WEA(rama_we),
    .SSRA(1'b0),
    .CLKA(clock),
    .DOA(rama_dataout[31:24]),
    .DOPA(),
    // port B
    .DIB(ramb_wdata[31:24]),
    .DIPB(1'b0),
    .ADDRB(ramb_addr),
    .ENB(ramb_en),
    .WEB(ramb_we),
    .SSRB(1'b0),
    .CLKB(clock),
    .DOB(ramb_rdata[31:24]), 
    .DOPB());

  RAMB16_S9_S9 RAMBG4 (
    // port A
    .DIA(rama_datain[39:32]),
    .DIPA(1'b0),
    .ADDRA(rama_address),
    .ENA(clkenb[4]),
    .WEA(rama_we),
    .SSRA(1'b0),
    .CLKA(clock),
    .DOA(rama_dataout[39:32]),
    .DOPA(),
    // port B
    .DIB(ramb_wdata[39:32]),
    .DIPB(1'b0),
    .ADDRB(ramb_addr),
    .ENB(ramb_en),
    .WEB(ramb_we),
    .SSRB(1'b0),
    .CLKB(clock),
    .DOB(ramb_rdata[39:32]), 
    .DOPB());

  RAMB16_S9_S9 RAMBG5 (
    // port A
    .DIA(rama_datain[47:40]),
    .DIPA(1'b0),
    .ADDRA(rama_address),
    .ENA(clkenb[5]),
    .WEA(rama_we),
    .SSRA(1'b0),
    .CLKA(clock),
    .DOA(rama_dataout[47:40]),
    .DOPA(),
    // port B
    .DIB(ramb_wdata[47:40]),
    .DIPB(1'b0),
    .ADDRB(ramb_addr),
    .ENB(ramb_en),
    .WEB(ramb_we),
    .SSRB(1'b0),
    .CLKB(clock),
    .DOB(ramb_rdata[47:40]), 
    .DOPB());

  RAMB16_S9_S9 RAMBG6 (
    // port A
    .DIA(rama_datain[55:48]),
    .DIPA(1'b0),
    .ADDRA(rama_address),
    .ENA(clkenb[6]),
    .WEA(rama_we),
    .SSRA(1'b0),
    .CLKA(clock),
    .DOA(rama_dataout[55:48]),
    .DOPA(),
    // port B
    .DIB(ramb_wdata[55:48]),
    .DIPB(1'b0),
    .ADDRB(ramb_addr),
    .ENB(ramb_en),
    .WEB(ramb_we),
    .SSRB(1'b0),
    .CLKB(clock),
    .DOB(ramb_rdata[55:48]), 
    .DOPB());

  RAMB16_S9_S9 RAMBG7 (
    // port A
    .DIA(rama_datain[63:56]),
    .DIPA(1'b0),
    .ADDRA(rama_address),
    .ENA(clkenb[7]),
    .WEA(rama_we),
    .SSRA(1'b0),
    .CLKA(clock),
    .DOA(rama_dataout[63:56]),
    .DOPA(),
    // port B
    .DIB(ramb_wdata[63:56]),
    .DIPB(1'b0),
    .ADDRB(ramb_addr),
    .ENB(ramb_en),
    .WEB(ramb_we),
    .SSRB(1'b0),
    .CLKB(clock),
    .DOB(ramb_rdata[63:56]), 
    .DOPB());
    
  assign rddata = uppersel ? rama_dataout[63:32] : rama_dataout[31:0];

  assign c3_p0_wr_data = ramb_rdata;

  parameter [2:0]
    IDLE = 3'h0,
    WRITE1 = 3'h1,
    WRITE2 = 3'h2,
    WRITE3 = 3'h3,
    WRITE4 = 3'h4,
    READ1 = 3'h5,
    READ2 = 3'h6,
    READ3 = 3'h7;

  always @ (posedge clock or posedge reset)
  begin
    if (reset) begin
      state = IDLE;
      burst_cnt = 6'd0;
      ramb_addr = 11'd0;
      ramb_en = 1'b0;
      ramb_wdata = 64'd0;
      ramb_we = 1'b0;
      c3_p0_cmd_en = 1'b0;
      c3_p0_cmd_instr = 3'd0;
      c3_p0_cmd_byte_addr = 30'd0;
      c3_p0_wr_en = 1'b0;
      c3_p0_rd_en = 1'b0;
    end else begin
      state = next_state;
      burst_cnt = next_burst_cnt;
      ramb_addr = next_ramb_addr;
      ramb_en = next_ramb_en;
      ramb_wdata = next_ramb_wdata;
      ramb_we = next_ramb_we;
      c3_p0_cmd_en = next_p0_cmd_en;
      c3_p0_cmd_instr = next_p0_cmd_instr;
      c3_p0_cmd_byte_addr = next_p0_cmd_byte_addr;
      c3_p0_wr_en = next_p0_wr_en;
      c3_p0_rd_en = next_p0_rd_en;
    end
  end

  always @*
  begin
    #1;
    next_state = state;
    next_burst_cnt = burst_cnt;
    next_ramb_addr = ramb_en ? ramb_addr + 1'b1 : ramb_addr;
    next_ramb_en = 1'b0;
    next_ramb_wdata = c3_p0_rd_data;
    next_ramb_we = 1'b0;
    next_p0_cmd_en = 1'b0;
    next_p0_cmd_instr = c3_p0_cmd_instr;
    next_p0_cmd_byte_addr = c3_p0_cmd_byte_addr;
    next_p0_wr_en = 1'b0;
    next_p0_rd_en = 1'b0;

    case(state)
      IDLE: begin
        if (write_block) begin
          next_state = WRITE1;
          next_ramb_addr = dram_address[10:0];
          next_ramb_en = 1'b1;
          next_burst_cnt = 6'd0;
          next_p0_cmd_instr = 3'b000;
          next_p0_cmd_byte_addr = {4'd0, dram_address[22:0], 3'b000};
        end else if (read_block) begin
          next_state = READ1;
          next_ramb_addr = dram_address[10:0];
          next_burst_cnt = 6'd0;
          next_p0_cmd_instr = 3'b001;
          next_p0_cmd_en = 1'b1; 
          next_p0_cmd_byte_addr = {4'd0, dram_address[22:0], 3'b000};
        end
      end
      WRITE1: begin
        next_p0_wr_en = 1'b1;
        next_burst_cnt = burst_cnt + 1'b1;
        if (burst_cnt == 6'd63)
          next_state = WRITE2;
        else begin
          next_ramb_en = 1'b1;
        end
      end
      WRITE2: begin
        next_state = IDLE;
        next_p0_cmd_en = 1'b1;
      end
      READ1: begin
        if(c3_p0_rd_empty == 0) begin
          next_p0_rd_en = 1'b1;
          next_state = READ2;
        end
      end
      READ2: begin
        next_burst_cnt = burst_cnt + 1'b1;
        next_ramb_en = 1'b1;
        next_ramb_we = 1'b1;
        if (burst_cnt == 6'd63)
          next_state = IDLE;
        else
          next_state = READ1;
      end
    endcase
  end


	wire [7:0] c3_p0_wr_mask = 8'b00000000;
  wire [5:0] c3_p0_cmd_bl = 6'd63;
  wire c3_p0_cmd_clk = clock;
  wire c3_p0_wr_clk = clock;
  wire c3_p0_rd_clk = clock;
  
  lpddr lpddr(
  .sys_clk(sys_clk),
  .sys_rst(sys_reset),
  .c3_calib_done(c3_calib_done),
  .clock(clock),
  .reset(reset),
  .mcb3_dram_dq(mcb3_dram_dq),
  .mcb3_dram_a(mcb3_dram_a),
  .mcb3_dram_ba(mcb3_dram_ba),
  .mcb3_dram_cke(mcb3_dram_cke),
  .mcb3_dram_ras_n(mcb3_dram_ras_n),
  .mcb3_dram_cas_n(mcb3_dram_cas_n),
  .mcb3_dram_we_n(mcb3_dram_we_n),
  .mcb3_dram_dm(mcb3_dram_dm),
  .mcb3_dram_udqs(mcb3_dram_udqs),
  .mcb3_rzq(mcb3_rzq),
  .mcb3_dram_udm(mcb3_dram_udm),
  .mcb3_dram_dqs(mcb3_dram_dqs),
  .mcb3_dram_ck(mcb3_dram_ck),
  .mcb3_dram_ck_n(mcb3_dram_ck_n),
  .c3_p0_cmd_clk(c3_p0_cmd_clk),
  .c3_p0_cmd_en(c3_p0_cmd_en),
  .c3_p0_cmd_instr(c3_p0_cmd_instr),
  .c3_p0_cmd_bl(c3_p0_cmd_bl),
  .c3_p0_cmd_byte_addr(c3_p0_cmd_byte_addr),
  .c3_p0_cmd_empty(c3_p0_cmd_empty),
  .c3_p0_cmd_full(c3_p0_cmd_full),
  .c3_p0_wr_clk(c3_p0_wr_clk),
  .c3_p0_wr_en(c3_p0_wr_en),
  .c3_p0_wr_mask(c3_p0_wr_mask),
  .c3_p0_wr_data(c3_p0_wr_data),
  .c3_p0_wr_full(c3_p0_wr_full),
  .c3_p0_wr_empty(c3_p0_wr_empty),
  .c3_p0_wr_count(c3_p0_wr_empty),
  .c3_p0_wr_underrun(c3_p0_wr_underrun),
  .c3_p0_wr_error(c3_p0_wr_error),
  .c3_p0_rd_clk(c3_p0_rd_clk),
  .c3_p0_rd_en(c3_p0_rd_en),
  .c3_p0_rd_data(c3_p0_rd_data),
  .c3_p0_rd_full(c3_p0_rd_full),
  .c3_p0_rd_empty(c3_p0_rd_empty),
  .c3_p0_rd_count(c3_p0_rd_count),
  .c3_p0_rd_overflow(c3_p0_rd_overflow),
  .c3_p0_rd_error(c3_p0_rd_error)
  );

endmodule

