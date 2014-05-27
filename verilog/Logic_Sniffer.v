//--------------------------------------------------------------------------------
// Logic_Sniffer.vhd
//
// Copyright (C) 2006 Michael Poppitz
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
// Details: http://www.sump.org/projects/analyzer/
//
// Logic Analyzer top level module. It connects the core with the hardware
// dependend IO modules and defines all inputs and outputs that represent
// phyisical pins of the fpga.
//
// It defines two constants FREQ and RATE. The first is the clock frequency 
// used for receiver and transmitter for generating the proper baud rate.
// The second defines the speed at which to operate the serial port.
//
//--------------------------------------------------------------------------------
//
// 12/29/2010 - Verilog Version + cleanups created by Ian Davis (IED) - mygizmos.org
//
// 05/21/2014 - Magnus Karlsson - Pipistrello version using FTDI async fifo mode
//

`timescale 1ns/100ps


module Logic_Sniffer(
  clock_in,
  reset_in,
  extClockIn,
  extClockOut,
  extTriggerIn,
  extTriggerOut,
  indata,
  usb_txe,
  usb_rxf,
  usb_wr,
  usb_rd,
  usb_siwua,
  usb_data,
  dataReady,
  armLED,
  triggerLED,
  breathLED,
  mcb3_rzq,
  mcb3_dram_we_n,
  mcb3_dram_udqs,
  mcb3_dram_udm,
  mcb3_dram_ras_n,
  mcb3_dram_dm,
  mcb3_dram_dqs,
  mcb3_dram_dq,
  mcb3_dram_ck_n,
  mcb3_dram_ck,
  mcb3_dram_cke,
  mcb3_dram_cas_n,
  mcb3_dram_ba,
  mcb3_dram_a);

input clock_in;
input reset_in;
input extClockIn;
output extClockOut;
input extTriggerIn;
output extTriggerOut;
inout [31:0] indata;
output dataReady;
output armLED;
output triggerLED;
output breathLED;

input usb_txe;
input usb_rxf;
output usb_wr;
output usb_rd;
output usb_siwua;
inout[7:0] usb_data;

inout mcb3_rzq;
output mcb3_dram_we_n;
inout mcb3_dram_udqs;
output mcb3_dram_udm;
output mcb3_dram_ras_n;
output mcb3_dram_dm;
inout mcb3_dram_dqs;
inout [15:0] mcb3_dram_dq;
output mcb3_dram_ck_n;
output mcb3_dram_ck;
output mcb3_dram_cke;
output mcb3_dram_cas_n;
output [1:0] mcb3_dram_ba;
output [12:0] mcb3_dram_a;


wire clock, reset, calib_done;

wire extReset = reset | ~calib_done;
wire [39:0] cmd;
wire [31:0] sram_wrdata;
wire [31:0] sram_rddata; 
wire [3:0] sram_rdvalid;
wire [31:0] stableInput;

wire [7:0] opcode;
wire [31:0] config_data; 
assign {config_data,opcode} = cmd;

wire usb_txe;
wire usb_rxf;
wire usb_wr;
wire usb_rd;
wire usb_siwua = 1'b1;
wire usb_dataoe;
wire[7:0] usb_dataout;
wire[7:0] usb_datain;
reg usb_reset;


  
// Bidirectional I/O buffers
IOBUF data0 (.IO(usb_data[0]), .O(usb_datain[0]), .I(usb_dataout[0]), .T(usb_dataoe));
IOBUF data1 (.IO(usb_data[1]), .O(usb_datain[1]), .I(usb_dataout[1]), .T(usb_dataoe));
IOBUF data2 (.IO(usb_data[2]), .O(usb_datain[2]), .I(usb_dataout[2]), .T(usb_dataoe));
IOBUF data3 (.IO(usb_data[3]), .O(usb_datain[3]), .I(usb_dataout[3]), .T(usb_dataoe));
IOBUF data4 (.IO(usb_data[4]), .O(usb_datain[4]), .I(usb_dataout[4]), .T(usb_dataoe));
IOBUF data5 (.IO(usb_data[5]), .O(usb_datain[5]), .I(usb_dataout[5]), .T(usb_dataoe));
IOBUF data6 (.IO(usb_data[6]), .O(usb_datain[6]), .I(usb_dataout[6]), .T(usb_dataoe));
IOBUF data7 (.IO(usb_data[7]), .O(usb_datain[7]), .I(usb_dataout[7]), .T(usb_dataoe));

  
// Output dataReady (can be used as an indicator for data transmit)
dly_signal dataReady_reg (clock, busy, dataReady);

// Use DDR output buffer to isolate clock & avoid skew penalty...
ddr_clkout extclock_pad (.pad(extClockOut), .clk(extclock));

// breathing LED
reg [26:0] PMW_counter;
reg [5:0] PWM_adj;
reg [6:0] PWM_width;
reg breathLED;
always @(posedge clock or posedge extReset) begin
  if(extReset) begin
    PMW_counter <= 27'b0;
    PWM_width <= 7'b0;
    breathLED <= 1'b0;
    PWM_adj <= 6'b0;
  end else begin
    PMW_counter <= PMW_counter + 1'b1;
    PWM_width <= PWM_width[5:0] + PWM_adj;
    if(PMW_counter[26])
      PWM_adj <= PMW_counter[25:20];
    else 
      PWM_adj <= ~ PMW_counter[25:20];
    breathLED <= PWM_width[6];
  end
end

//
// Configure the probe pins...
//
reg [15:0] test_counter;
always @ (posedge clock or posedge extReset) begin
  if (extReset)
    test_counter <= 0;
  else
    test_counter <= test_counter + 1'b1;
end

wire [15:0] test_pattern = test_counter;
/*
wire [15:0] test_pattern = {
    test_counter[10], test_counter[4],
    test_counter[10], test_counter[4],
    test_counter[10], test_counter[4],
    test_counter[10], test_counter[4],
    test_counter[10], test_counter[4],
    test_counter[10], test_counter[4],
    test_counter[10], test_counter[4],
    test_counter[10], test_counter[4]};
*/

outbuf io_indata31 (.pad(indata[31]), .clk(clock), .outsig(test_pattern[15]), .oe(extTestMode));
outbuf io_indata30 (.pad(indata[30]), .clk(clock), .outsig(test_pattern[14]), .oe(extTestMode));
outbuf io_indata29 (.pad(indata[29]), .clk(clock), .outsig(test_pattern[13]), .oe(extTestMode));
outbuf io_indata28 (.pad(indata[28]), .clk(clock), .outsig(test_pattern[12]), .oe(extTestMode));
outbuf io_indata27 (.pad(indata[27]), .clk(clock), .outsig(test_pattern[11]), .oe(extTestMode));
outbuf io_indata26 (.pad(indata[26]), .clk(clock), .outsig(test_pattern[10]), .oe(extTestMode));
outbuf io_indata25 (.pad(indata[25]), .clk(clock), .outsig(test_pattern[9]), .oe(extTestMode));
outbuf io_indata24 (.pad(indata[24]), .clk(clock), .outsig(test_pattern[8]), .oe(extTestMode));
outbuf io_indata23 (.pad(indata[23]), .clk(clock), .outsig(test_pattern[7]), .oe(extTestMode));
outbuf io_indata22 (.pad(indata[22]), .clk(clock), .outsig(test_pattern[6]), .oe(extTestMode));
outbuf io_indata21 (.pad(indata[21]), .clk(clock), .outsig(test_pattern[5]), .oe(extTestMode));
outbuf io_indata20 (.pad(indata[20]), .clk(clock), .outsig(test_pattern[4]), .oe(extTestMode));
outbuf io_indata19 (.pad(indata[19]), .clk(clock), .outsig(test_pattern[3]), .oe(extTestMode));
outbuf io_indata18 (.pad(indata[18]), .clk(clock), .outsig(test_pattern[2]), .oe(extTestMode));
outbuf io_indata17 (.pad(indata[17]), .clk(clock), .outsig(test_pattern[1]), .oe(extTestMode));
outbuf io_indata16 (.pad(indata[16]), .clk(clock), .outsig(test_pattern[0]), .oe(extTestMode));



//
// Instantiate async-fifo interface....
//
ftdi_async_fifo ftdi (
  .clock(clock), 
  .extReset(extReset),
  .dataIn(stableInput),
  .send(send), 
  .send_data(sram_rddata), 
  .send_valid(sram_rdvalid),
  .cmd(cmd), .execute(execute), 
  .busy(busy),
  .usb_txe(usb_txe),
  .usb_rxf(usb_rxf),
  .usb_wr(usb_wr),
  .usb_rd(usb_rd),
  .usb_datain(usb_datain),
  .usb_dataout(usb_dataout),
  .usb_dataoe(usb_dataoe));


//
// Instantiate core...
//
core core (
  .clock(clock),
  .extReset(extReset),
  .extClock(extClockIn),
  .extTriggerIn(extTriggerIn),
  .opcode(opcode),
  .config_data(config_data),
  .execute(execute),
  .indata(indata),
  .outputBusy(busy),
  // outputs...
  .sampleReady50(),
  .stableInput(stableInput),
  .outputSend(send),
  .memoryWrData(sram_wrdata),
  .memoryRead(read),
  .memoryWrite(write),
  .memoryLastWrite(lastwrite),
  .extTriggerOut(extTriggerOut),
  .extClockOut(extclock), 
  .armLED(armLED),
  .triggerLED(triggerLED),
  .wrFlags(wrFlags),
  .extTestMode(extTestMode));


//
// Instantiate the memory...
//
ram_interface ram_interface (
  .sys_clk(clock_in),
  .sys_reset(reset_in),
  .wrFlags(wrFlags), 
  .config_data(config_data[5:2]),
  .write(write), .lastwrite(lastwrite), .read(read),
  .wrdata(sram_wrdata),
  .rddata(sram_rddata),
  .rdvalid(sram_rdvalid),
  .clock(clock),
  .reset(reset),
  .c3_calib_done(calib_done),
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
  .mcb3_dram_ck_n(mcb3_dram_ck_n));

endmodule

