//======================================================================
//
// api_extension.v
// ---------------
// Module that provides API access for all extensions to the
// baseline FPGA_NTP_SERVER. The module is connected to/integrated
// into the baseline design via an I/O port in network_path_shared.
//
// The module implements the high level address map. It also
// includes some internal test functionality (a 32-bit adder) that
// is mapped into the address space of the extension itself.
//
// The module implements a wait state mechanism to allow transport
// cycles. But the module will also wait for ready flags from
// the extension modules to allow things like SPI connected
// API endpoints to respond.
//
//
// Author: Joachim Strombergson
//
// Copyright (c) 2019, The Swedish Post and Telecom Authority (PTS)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

module api_extension(
                     input wire           clk,
                     input wire           reset,

                     // I/O port.
                     input wire [1 : 0]   command,
                     output wire [1 : 0]  status,
                     input wire [31 : 0]  address,
                     input wire [31 : 0]  write_data,
                     output wire [31 : 0] read_data,

                     // Access ports to extensions.
                     output wire          dp_cs,
                     output wire          dp_we,
                     output wire [23 : 0] dp_address,
                     output wire [31 : 0] dp_write_data,
                     input wire  [31 : 0] dp_read_data,
                     input wire           dp_ready,

                     output wire          nts_cs,
                     output wire          nts_we,
                     output wire [23 : 0] nts_address,
                     output wire [31 : 0] nts_write_data,
                     input wire  [31 : 0] nts_read_data,
                     input wire           nts_ready,

                     output wire          rosc_cs,
                     output wire          rosc_we,
                     output wire [7 : 0]  rosc_address,
                     output wire [31 : 0] rosc_write_data,
                     input wire  [31 : 0] rosc_read_data,
                     input wire           rosc_ready
                    );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  // Address space prefixes. We support 256 new top level modules.
  localparam API_PREFIX       = 8'h00;
  localparam DP_PREFIX        = 8'h10;
  localparam NTS_PREFIX       = 8'h20;
  localparam ROSC_PREFIX      = 8'hfe;

  // The API for the API extension itself.
  localparam API_ADDR_NAME0   = 8'h00;
  localparam API_ADDR_NAME1   = 8'h01;
  localparam API_ADDR_VERSION = 8'h02;

  localparam API_ADDR_OP_A    = 8'h10;
  localparam API_ADDR_OP_B    = 8'h11;
  localparam API_ADDR_SUM     = 8'h12;

  localparam CORE_NAME0       = 32'h6170692d; // "api-"
  localparam CORE_NAME1       = 32'h65787420; // "ext "
  localparam CORE_VERSION     = 32'h302e3230; // "0.20"

  localparam COMMAND_IDLE     = 2'h0;
  localparam COMMAND_READ     = 2'h1;
  localparam COMMAND_WRITE    = 2'h3;

  localparam STATUS_BUSY      = 2'h0;
  localparam STATUS_READY     = 2'h1;
  localparam STATUS_ERROR     = 2'h3;

  localparam API_CTRL_IDLE    = 2'h0;
  localparam API_CTRL_WAIT    = 2'h1;
  localparam API_CTRL_DONE    = 2'h2;

  localparam DEF_WAIT_CYCLES  = 3'h2;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [1 : 0]  command_reg;

  reg [1 : 0]  status_reg;
  reg [1 : 0]  status_new;
  reg          status_we;

  reg          ready_reg;
  reg          ready_new;

  reg          cs_reg;
  reg          cs_new;
  reg          cs_we;

  reg          we_reg;
  reg          we_new;
  reg          we_we;

  reg [31 : 0] address_reg;
  reg          address_we;

  reg [31 : 0] read_data_reg;
  reg [31 : 0] read_data_new;
  reg          read_data_we;

  reg [31 : 0] write_data_reg;
  reg          write_data_we;

  reg [31 : 0] op_a_reg;
  reg          op_a_we;

  reg [31 : 0] op_b_reg;
  reg          op_b_we;

  reg [31 : 0] sum_reg;

  reg [2 : 0]  wait_cycles_ctr_reg;
  reg [2 : 0]  wait_cycles_ctr_new;
  reg          wait_cycles_ctr_rst;
  reg          wait_cycles_ctr_inc;
  reg          wait_cycles_ctr_we;

  reg [1 : 0]  api_extension_ctrl_reg;
  reg [1 : 0]  api_extension_ctrl_new;
  reg          api_extension_ctrl_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg tmp_dp_cs;
  reg tmp_dp_we;
  reg tmp_nts_cs;
  reg tmp_nts_we;
  reg tmp_rosc_cs;
  reg tmp_rosc_we;
  reg address_error;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign status    = status_reg;
  assign read_data = read_data_reg;

  assign dp_cs         = tmp_nts_cs;
  assign dp_we         = tmp_nts_we;
  assign dp_address    = address_reg[23 : 0];
  assign dp_write_data = write_data_reg;

  assign nts_cs         = tmp_nts_cs;
  assign nts_we         = tmp_nts_we;
  assign nts_address    = address_reg[23 : 0];
  assign nts_write_data = write_data_reg;

  assign rosc_cs = tmp_rosc_cs;
  assign rosc_we = tmp_rosc_we;
  assign rosc_address    = address_reg[7 : 0];
  assign rosc_write_data = write_data_reg;


  //----------------------------------------------------------------
  // reg_update
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with asynchronous
  // active low reset.
  //----------------------------------------------------------------
  always @ (posedge clk or posedge reset)
    begin : reg_update
      if (reset)
        begin
          command_reg            <= 2'h0;
          status_reg             <= STATUS_READY;
          ready_reg              <= 1'h0;
          cs_reg                 <= 1'h0;
          we_reg                 <= 1'h0;
          address_reg            <= 32'h0;
          read_data_reg          <= 32'h0;
          op_a_reg               <= 32'h0;
          op_b_reg               <= 32'h0;
          sum_reg                <= 32'h0;
          write_data_reg         <= 32'h0;
          wait_cycles_ctr_reg    <= DEF_WAIT_CYCLES;
          api_extension_ctrl_reg <= API_CTRL_IDLE;
        end
      else
        begin
          command_reg <= command;
          ready_reg   <= ready_new;

          sum_reg     <= op_a_reg + op_b_reg;

          if (status_we)
            status_reg <= status_new;

          if (cs_we)
            cs_reg <= cs_new;

          if (we_we)
            we_reg <= we_new;

          if (address_we)
            address_reg <= address;

          if (read_data_we)
            read_data_reg <= read_data_new;

          if (write_data_we)
            write_data_reg <= write_data;

          if (op_a_we)
            op_a_reg <= write_data;

          if (op_b_we)
            op_b_reg <= write_data;

          if (wait_cycles_ctr_we)
            wait_cycles_ctr_reg <= wait_cycles_ctr_new;

          if (api_extension_ctrl_we)
            api_extension_ctrl_reg <= api_extension_ctrl_new;
        end
    end // reg_update


  //----------------------------------------------------------------
  // addr_mux
  //----------------------------------------------------------------
  always @*
    begin : addr_mux
      tmp_dp_cs     = 1'h0;
      tmp_dp_we     = 1'h0;
      tmp_nts_cs    = 1'h0;
      tmp_nts_we    = 1'h0;
      tmp_rosc_cs   = 1'h0;
      tmp_rosc_we   = 1'h0;
      ready_new     = 1'h0;
      read_data_new = 32'h0;
      op_a_we       = 1'h0;
      op_b_we       = 1'h0;
      address_error = 1'h0;

      case (address_reg[31 : 24])
        API_PREFIX:
          begin
            ready_new = 1'h1;

            if (cs_reg)
              begin
                if (we_reg)
                  begin
                    if (address_reg[7 : 0] == API_ADDR_OP_A)
                      op_a_we = 1'h1;

                    if (address_reg[7 : 0] == API_ADDR_OP_B)
                      op_b_we = 1'h1;
                  end
                else
                  begin
                    case (address_reg[7 : 0])
                      API_ADDR_NAME0:   read_data_new = CORE_NAME0;
                      API_ADDR_NAME1:   read_data_new = CORE_NAME1;
                      API_ADDR_VERSION: read_data_new = CORE_VERSION;
                      API_ADDR_OP_A:    read_data_new = op_a_reg;
                      API_ADDR_OP_B:    read_data_new = op_b_reg;
                      API_ADDR_SUM:     read_data_new = sum_reg;
                      default:
                        begin
                        end
                    endcase // case (address_reg[7 : 0])
                  end
              end
          end


        DP_PREFIX:
          begin
            tmp_dp_cs     = cs_reg;
            tmp_dp_we     = we_reg;
            ready_new     = dp_ready;
            read_data_new = dp_read_data;
          end


        NTS_PREFIX:
          begin
            tmp_nts_cs    = cs_reg;
            tmp_nts_we    = we_reg;
            ready_new     = nts_ready;
            read_data_new = nts_read_data;
          end


        ROSC_PREFIX:
          begin
            tmp_rosc_cs   = cs_reg;
            tmp_rosc_we   = we_reg;
            ready_new     = rosc_ready;
            read_data_new = rosc_read_data;
          end


        default:
          begin
            ready_new     = 1'h1;
            read_data_new = 32'hdeaddead;
            address_error = 1'h1;
          end
      endcase // case (address_reg[31 : 24])
    end // addr_mux


  //----------------------------------------------------------------
  // wait_cycles_ctr
  //----------------------------------------------------------------
  always @*
    begin : wait_cycles_ctr
      wait_cycles_ctr_new = 3'h0;
      wait_cycles_ctr_we  = 1'h0;

      if (wait_cycles_ctr_rst)
        wait_cycles_ctr_we = 1'h1;

      if (wait_cycles_ctr_inc)
        begin
          wait_cycles_ctr_new = wait_cycles_ctr_reg + 1'h1;
          wait_cycles_ctr_we  = 1'h1;
        end
    end


  //----------------------------------------------------------------
  // api_extension_ctrl
  //----------------------------------------------------------------
  always @*
    begin : api_extension_ctrl
      status_new             = STATUS_READY;
      status_we              = 1'h0;
      cs_new                 = 1'h0;
      cs_we                  = 1'h0;
      we_new                 = 1'h0;
      we_we                  = 1'h0;
      address_we             = 1'h0;
      read_data_we           = 1'h0;
      write_data_we          = 1'h0;
      wait_cycles_ctr_rst    = 1'h0;
      wait_cycles_ctr_inc    = 1'h0;
      api_extension_ctrl_new = API_CTRL_IDLE;
      api_extension_ctrl_we  = 1'h0;


      case (api_extension_ctrl_reg)
        API_CTRL_IDLE:
          begin
            if (command_reg != COMMAND_IDLE)
              begin
                if (command_reg == COMMAND_WRITE)
                  begin
                    write_data_we = 1'h1;
                    we_new        = 1'h1;
                    we_we         = 1'h1;
                  end

                status_new             = STATUS_BUSY;
                status_we              = 1'h1;
                cs_new                 = 1'h1;
                cs_we                  = 1'h1;
                address_we             = 1'h1;
                wait_cycles_ctr_rst    = 1'h1;
                api_extension_ctrl_new = API_CTRL_WAIT;
                api_extension_ctrl_we  = 1'h1;
              end
          end


        API_CTRL_WAIT:
          begin
            if (wait_cycles_ctr_reg == DEF_WAIT_CYCLES)
              begin
                if (ready_reg)
                  begin
                    if (address_error)
                      status_new = STATUS_ERROR;
                    else
                      status_new = STATUS_READY;

                    status_we              = 1'h1;
                    cs_new                 = 1'h0;
                    cs_we                  = 1'h1;
                    we_new                 = 1'h0;
                    we_we                  = 1'h1;
                    read_data_we           = 1'h1;
                    api_extension_ctrl_new = API_CTRL_DONE;
                    api_extension_ctrl_we  = 1'h1;
                  end
              end
            else
              wait_cycles_ctr_inc = 1'h1;
          end


        API_CTRL_DONE:
          begin
            if (command_reg == COMMAND_IDLE)
              begin
                status_new             = STATUS_READY;
                status_we              = 1'h1;
                api_extension_ctrl_new = API_CTRL_IDLE;
                api_extension_ctrl_we  = 1'h1;
              end
          end

        default:
          begin
          end
      endcase // case (api_extension_ctrl_reg)
    end // api_extension_ctrl
endmodule // api_extension

//======================================================================
// EOF api_extension.v
//======================================================================
