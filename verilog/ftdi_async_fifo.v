//--------------------------------------------------------------------------------
// spi_slave.v
//
// Copyright (C) 2006 Michael Poppitz
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
// Details: http://www.sump.org/projects/analyzer/
//
// spi_slave
//
//--------------------------------------------------------------------------------
//
// 01/22/2011 - Ian Davis - Added meta data generator.
//
// 10/05/2013 - Magnus Karlsson - Converted to FTDI async FIFO mode
//

`timescale 1ns/100ps

module ftdi_async_fifo(
  clock, extReset, 
  dataIn,
  send, send_data, send_valid,
  cmd, execute, busy,
  usb_txe,
  usb_rxf,
  usb_wr,
  usb_rd,
  usb_datain,
  usb_dataout,
  usb_dataoe);

input clock;
input extReset;
input send;
input [31:0] send_data;
input [3:0] send_valid;
input [31:0] dataIn;

output [39:0] cmd;
output execute;
output busy;

input usb_txe; // do not write if high
input usb_rxf; // do not read if high
output usb_wr;
output usb_rd;
input [7:0] usb_datain;
output [7:0] usb_dataout;
output usb_dataoe;

//
// Registers...
//
reg query_id, next_query_id; 
reg query_metadata, next_query_metadata;
reg query_dataIn, next_query_dataIn; 


//
// Process special commands not handled by core decoder...
//
always @(posedge clock) 
begin
  query_id = next_query_id;
  query_metadata = next_query_metadata;
  query_dataIn = next_query_dataIn;
end

always @*
begin
  #1;
  next_query_id = 1'b0; 
  next_query_metadata = 1'b0;
  next_query_dataIn = 1'b0;
  if (execute)
    case (opcode)
      8'h02 : next_query_id = 1'b1;
      8'h04 : next_query_metadata = 1'b1; 
      8'h06 : next_query_dataIn = 1'b1;
    endcase
end


//
// Receiver
//
parameter 
  READOPCODE = 1'h0,
  READLONG = 1'h1;

parameter [1:0]
  RX_IDLE = 2'h0,
  RX_START = 2'h1,
  RX_RECEIVE = 2'h2,
  RX_STOP = 2'h3;

reg rx_state, next_rx_state;                        // receiver tx_state
reg [1:0] bytecount, next_bytecount;        // count rxed bytes of current command
reg [7:0] opcode, next_opcode;                // opcode byte
reg [31:0] databuf, next_databuf;        // data dword
reg execute, next_execute;

wire [7:0] rxByte;
wire byteready;

assign cmd = {databuf,opcode};

//
// Command tracking...
//
always @(posedge clock or posedge extReset) 
begin
  if (extReset) begin
    rx_state = READOPCODE;
    databuf = 32'd0;
    execute = 1'b0;
    bytecount = 2'd0;
    opcode = 8'd0;
  end else begin
    rx_state = next_rx_state;
    databuf = next_databuf;
    execute = next_execute;
    bytecount = next_bytecount;
    opcode = next_opcode;
  end
end

always @*
begin
  #1;
  next_rx_state = rx_state;
  next_bytecount = bytecount;
  next_opcode = opcode;
  next_databuf = databuf;
  next_execute = 1'b0;
  
  case (rx_state)
    READOPCODE : // receive byte
      begin
        next_bytecount = 0;
        if (byteready)
          begin
            next_opcode = rxByte;
            if (rxByte[7])
              next_rx_state = READLONG;
            else // short command
              begin
                next_execute = 1'b1;
                next_rx_state = READOPCODE;
              end
          end
      end

    READLONG : // receive 4 word parameter
      begin
        if (byteready)
          begin
            next_bytecount = bytecount + 1'b1;
            next_databuf = {rxByte,databuf[31:8]};
            if (&bytecount) // execute long command
              begin
                next_execute = 1'b1;
                next_rx_state = READOPCODE;
              end
          end
      end
  endcase
end




//
// Transmitter
//
parameter [1:0]
  TX_IDLE = 2'h0,
  TX_START = 2'h1,
  TX_SEND = 2'h2,
  TX_STOP = 2'h3;

reg [31:0] sampled_send_data, next_sampled_send_data;
reg [3:0] sampled_send_valid, next_sampled_send_valid;
reg [2:0] bits, next_bits;
reg [1:0] bytesel, next_bytesel;
reg [5:0] metasel, next_metasel;
reg busy, next_busy;
reg writeByte; 

reg [1:0] tx_uart_state, next_tx_uart_state;
reg [9:0] tx_count, next_tx_count;
reg [2:0] tx_bitcount, next_tx_bitcount;
reg [7:0] txByte, next_txByte;
reg tx, next_tx;
reg byteDone, next_byteDone;


`define ADDBYTE(cmd) meta_rom[i]=cmd; i=i+1
`define ADDSHORT(cmd,b0) meta_rom[i]=cmd; meta_rom[i+1]=b0; i=i+2
`define ADDLONG(cmd,b0,b1,b2,b3) meta_rom[i]=cmd; meta_rom[i+1]=b0; meta_rom[i+2]=b1; meta_rom[i+3]=b2; meta_rom[i+4]=b3; i=i+5

//
// Create meta data ROM...
//
reg [5:0] METADATA_LEN;
reg [7:0] meta_rom[63:0];
wire [7:0] meta_data = meta_rom[metasel];
initial
begin : meta
  integer i;

  i=0;

  `ADDLONG(8'h01, "P", "i", "p", "i"); // Device name string...
  `ADDLONG("s", "t", "r", "e", "l");
  `ADDLONG("l", "o", " ", "O", "L");
  `ADDLONG("S", " ", "6", "4", "M");
  `ADDLONG(" ", "v", "1", ".", "0");
  `ADDBYTE("0");
  `ADDBYTE(0);

  
  `ADDLONG(8'h02, "3", ".", "0", "8"); // FPGA firmware version string
  `ADDBYTE(0);

  `ADDLONG(8'h21,8'h04,8'h00,8'h00,8'h00); // Amount of sample memory (64M)
  `ADDLONG(8'h23,8'h0B,8'hEB,8'hC2,8'h00); // Max sample rate (200Mhz)

  `ADDSHORT(8'h40,8'h20); // Max # of probes
  `ADDSHORT(8'h41,8'h02); // Protocol version 

  `ADDBYTE(0); // End of data flag
  METADATA_LEN = i;

  for (i=i; i<64; i=i+1) meta_rom[i]=0; // Padding
end


//
// Control FSM for sending 32 bit words out serial interface...
//
parameter [2:0] INIT = 0, IDLE = 1, SEND = 2, POLL = 3, METASEND = 4, METAPOLL = 5;
reg [2:0] tx_state, next_tx_state;
wire space_avail;

always @(posedge clock or posedge extReset) 
begin
  if (extReset) 
    begin
      tx_state = INIT;
      sampled_send_data = 32'h0;
      sampled_send_valid = 4'h00;
      bytesel = 2'h0;
      metasel = 5'd0;
      busy = 1'b0;
    end 
  else 
    begin
      tx_state = next_tx_state;
      sampled_send_data = next_sampled_send_data;
      sampled_send_valid = next_sampled_send_valid;
      bytesel = next_bytesel;
      metasel = next_metasel;
      busy = next_busy;
    end
end

always @*
begin
  #1;
  next_tx_state = tx_state;
  next_sampled_send_data = sampled_send_data;
  next_sampled_send_valid = sampled_send_valid;
  next_bytesel = bytesel;
  next_metasel = metasel;

  next_busy = (tx_state != IDLE) || send;
  writeByte = 1'b0;

  case (tx_state) // when write is '1', data will be available with next cycle
    INIT :
      begin
        next_sampled_send_data = 32'h0;
        next_sampled_send_valid = 4'hF;
        next_bytesel = 2'h0;
        next_metasel = 5'd0;
        next_busy = 1'b0;
        next_tx_state = IDLE;
      end

    IDLE : 
      begin
        next_sampled_send_data = send_data;
        next_sampled_send_valid = send_valid;
        next_bytesel = 2'h0;
        next_metasel = 5'd0;
            
        if (send) 
          next_tx_state = SEND;
        else if (query_id) // output dword containing "SLA1" signature
          begin
            next_sampled_send_data = 32'h534c4131; // "SLA1"
            next_sampled_send_valid = 4'hF;
            next_tx_state = SEND;
          end
        else if (query_dataIn)
          begin
            next_sampled_send_data = dataIn;
            next_sampled_send_valid = 4'hF;
            next_tx_state = SEND;
          end
        else if (query_metadata)
          begin
            next_sampled_send_data = {24'd0, meta_data};
            next_sampled_send_valid = 4'hF;
            next_tx_state = METASEND;
          end
      end

    SEND : // output dword send by controller...
      if (space_avail) begin
        writeByte = 1'b1;
        next_bytesel = bytesel + 1'b1;
        next_tx_state = POLL;
      end

    POLL : 
      begin
        next_tx_state = (~|bytesel) ? IDLE : SEND;
      end
      
    METASEND :
      if (space_avail) begin
        writeByte = 1'b1;
        next_metasel = metasel+1'b1;
        next_tx_state = METAPOLL;
      end

    METAPOLL :
      begin
        next_sampled_send_data = {24'd0, meta_data};
        next_sampled_send_valid = 4'hF;
        next_tx_state = (metasel==METADATA_LEN) ? IDLE : METASEND;
      end

    default : next_tx_state = INIT;
  endcase
end


//
// Byte select mux...   Revised for better synth. - IED
//
reg [7:0] txbyte;
reg disabled;
always @*
begin
  #1;
  txbyte = 0;
  disabled = 0;
  case (bytesel)
    2'h0 : begin txbyte = sampled_send_data[7:0]; disabled = !sampled_send_valid[0]; end
    2'h1 : begin txbyte = sampled_send_data[15:8]; disabled = !sampled_send_valid[1]; end
    2'h2 : begin txbyte = sampled_send_data[23:16]; disabled = !sampled_send_valid[2]; end
    2'h3 : begin txbyte = sampled_send_data[31:24]; disabled = !sampled_send_valid[3]; end
  endcase
end


//
// FTDI Async FIFO interface
//

reg usb_wr, next_wr;
reg usb_rd, next_rd;
reg usb_dataoe, next_dataoe;
reg[7:0] usb_dataout;
reg[2:0] state, next_state;
reg wr_done;
reg usb_txed1, usb_txed2;
reg usb_rxfd1, usb_rxfd2;
reg usb_busy;

wire [7:0] fifo_dataout;
wire data_valid;
wire fifo_full, fifo_empty;
reg fifo_rd, fifo_wr;
reg usb_enable;
wire data_avail;
  
async_fifo #(.ASYNC_FIFO_MAXDATA(7)) tx_fifo (
  .wrclk(clock), .wrreset(extReset),
  .rdclk(clock), .rdreset(extReset),
  .space_avail(space_avail), .wrenb(writeByte & !disabled), .wrdata(txbyte),
  .read_req(fifo_rd), .data_avail(data_avail), 
  .data_valid(data_valid), .data_out(fifo_dataout));

async_fifo #(.ASYNC_FIFO_MAXDATA(7)) rx_fifo (
  .wrclk(clock), .wrreset(extReset),
  .rdclk(clock), .rdreset(extReset),
  .space_avail(), .wrenb(fifo_wr), .wrdata(usb_datain),
  .read_req(1'b1), .data_avail(), 
  .data_valid(byteready), .data_out(rxByte));

always @ (posedge clock or posedge extReset) begin
  if (extReset)
    fifo_rd <= 1'b0;
  else
    fifo_rd <= (data_avail & !usb_busy & !fifo_rd & !data_valid);
end

always @ (posedge clock) begin
  if (data_valid)
    usb_dataout <= fifo_dataout;
end

always @ (posedge clock or posedge extReset) begin
  if (extReset) begin
    usb_txed1 <= 1'b1;
    usb_txed2 <= 1'b1;
    usb_rxfd1 <= 1'b1;
    usb_rxfd2 <= 1'b1;
  end else begin
    usb_txed1 <= usb_txe;
    usb_txed2 <= usb_txed1 | usb_txe;
    usb_rxfd1 <= usb_rxf;
    usb_rxfd2 <= usb_rxfd1 | usb_rxf;
  end
end

always @ (posedge clock or posedge extReset) begin
  if (extReset) begin
    usb_wr <= 1'b1;
    usb_rd <= 1'b1;
    usb_dataoe <= 1'b1;
    state <= 3'h0;
    usb_busy <= 1'b0;
  end else begin
    usb_wr <= next_wr;
    usb_rd <= next_rd;
    usb_dataoe <= next_dataoe;
    state <= next_state;
    if (data_valid) begin
      usb_busy <= 1'b1;
    end else if (wr_done) begin
      usb_busy <= 1'b0;
    end
  end
end


always @ * begin
  next_state = state;
  next_wr = 1'b1;
  next_rd = 1'b1;
  next_dataoe = 1'b1;
  fifo_wr = 1'b0;
  wr_done = 1'b0;

  case(state)
    3'h0: begin
      if (!usb_rxfd2) begin
        next_state = 3'h4;
      end else if (usb_busy) begin
        next_state = 3'h1;
        next_dataoe = 1'b0;
      end
    end
    3'h1: begin
    next_dataoe = 1'b0;
      if (!usb_txed2) begin
        next_state = 3'h2;
        next_wr = 1'b0;
      end
    end
    3'h2: begin
      next_state = 3'h3;
      next_dataoe = 1'b0;
      next_wr = 1'b0;
      wr_done = 1'b1;
    end
    3'h3: begin
      next_state = 3'h0;
      next_dataoe = 1'b0;
      next_wr = 1'b0;
    end
    3'h4: begin
      next_state = 3'h5;
      next_rd = 1'b0;
    end
    3'h5: begin
      next_state = 3'h6;
      next_rd = 1'b0;
    end
    3'h6: begin
      next_state = 3'h7;
      next_rd = 1'b0;
      fifo_wr = 1'b1;
    end
    3'h7: begin
      next_state = usb_rxfd2 ? 3'h0 : 3'h7;
    end
    default: begin
      next_state = 3'h0;
    end
  endcase
end

endmodule
