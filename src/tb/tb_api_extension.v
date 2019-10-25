//======================================================================
//
// tb_api_extension.v
// ------------------
// Testbench for the api_key_extension module.
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


//------------------------------------------------------------------
// Test module.
//------------------------------------------------------------------
module tb_api_key_extension();

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter DEBUG = 0;

  parameter CLK_HALF_PERIOD = 1;
  parameter CLK_PERIOD      = 2 * CLK_HALF_PERIOD;

  // API
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

  localparam COMMAND_IDLE     = 2'h0;
  localparam COMMAND_READ     = 2'h1;
  localparam COMMAND_WRITE    = 2'h3;

  localparam STATUS_BUSY      = 2'h0;
  localparam STATUS_READY     = 2'h1;
  localparam STATUS_ERROR     = 2'h3;


  //----------------------------------------------------------------
  // Register and Wire declarations.
  //----------------------------------------------------------------
  reg [31 : 0]  cycle_ctr;
  reg [31 : 0]  error_ctr;
  reg [31 : 0]  tc_ctr;

  reg [31 : 0]  read_data;
  reg [127 : 0] result_data;

  reg           tb_clk;
  reg           tb_reset;

  reg [1 : 0]   dut_command;
  wire [1 : 0]  dut_status;
  reg [31 : 0]  dut_address;
  reg [31 : 0]  dut_write_data;
  wire [31 : 0] dut_read_data;
  wire          dut_dp_cs;
  wire          dut_dp_we;
  wire [23 : 0] dut_dp_address;
  wire [31 : 0] dut_dp_write_data;
  reg  [31 : 0] dut_dp_read_data;
  reg           dut_dp_ready;
  wire          dut_nts_cs;
  wire          dut_nts_we;
  wire [23 : 0] dut_nts_address;
  wire [31 : 0] dut_nts_write_data;
  reg  [31 : 0] dut_nts_read_data;
  reg           dut_nts_ready;
  wire          dut_rosc_cs;
  wire          dut_rosc_we;
  wire [7 : 0]  dut_rosc_address;
  wire [31 : 0] dut_rosc_write_data;
  reg  [31 : 0] dut_rosc_read_data;
  reg           dut_rosc_ready;

  //----------------------------------------------------------------
  // Device Under Test.
  //----------------------------------------------------------------
  api_extension dut(
                    .clk(tb_clk),
                    .reset(tb_reset),

                    .command(dut_command),
                    .status(dut_status),
                    .address(dut_address),
                    .write_data(dut_write_data),
                    .read_data(dut_read_data),

                    .dp_cs(dut_dp_cs),
                    .dp_we(dut_dp_we),
                    .dp_address(dut_dp_address),
                    .dp_write_data(dut_dp_write_data),
                    .dp_read_data(dut_dp_read_data),
                    .dp_ready(dut_dp_ready),

                    .nts_cs(dut_nts_cs),
                    .nts_we(dut_nts_we),
                    .nts_address(dut_nts_address),
                    .nts_write_data(dut_nts_write_data),
                    .nts_read_data(dut_nts_read_data),
                    .nts_ready(dut_nts_ready),

                    .rosc_cs(dut_rosc_cs),
                    .rosc_we(dut_rosc_we),
                    .rosc_address(dut_rosc_address),
                    .rosc_write_data(dut_rosc_write_data),
                    .rosc_read_data(dut_rosc_read_data),
                    .rosc_ready(dut_rosc_ready)
                   );


  //----------------------------------------------------------------
  // clk_gen
  //
  // Always running clock generator process.
  //----------------------------------------------------------------
  always
    begin : clk_gen
      #CLK_HALF_PERIOD;
      tb_clk = !tb_clk;
    end // clk_gen


  //----------------------------------------------------------------
  // sys_monitor()
  //
  // An always running process that creates a cycle counter and
  // conditionally displays information about the DUT.
  //----------------------------------------------------------------
  always
    begin : sys_monitor
      cycle_ctr = cycle_ctr + 1;

      #(CLK_PERIOD);

      if (DEBUG)
        begin
          dump_dut_state();
        end
    end


  //----------------------------------------------------------------
  // dump_dut_state()
  //
  // Dump the state of the dump when needed.
  //----------------------------------------------------------------
  task dump_dut_state;
    begin
      $display("cycle: 0x%016x", cycle_ctr);
      $display("Inputs and outputs:");
      $display("-------------------");
      $display("command: 0x%02x, address: 0x%08x, write_data: 0x%08x\n",
               dut_command, dut_address, dut_write_data);
      $display("status:  0x%02x, read_data: 0x%08x\n",
               dut_status, dut_read_data);
      $display("");

      $display("Internal data fields:");
      $display("op_a_reg: 0x%08x, op_b_reg: 0x%08x, sum_reg: 0x%08x",
               dut.op_a_reg, dut.op_b_reg, dut.sum_reg);
      $display("");

      $display("Internal control:");
      $display("cs_reg: 0x%01x, we_reg: 0x%01x", dut.cs_reg, dut.we_reg);
      $display("address_reg: 0x%08x, read_data_reg: 0x%08x, write_data_reg: 0x%08x",
               dut.address_reg, dut.read_data_reg, dut.write_data_reg);
      $display("api_extension_ctrl_reg: 0x%02x, wait_cycles_ctr_reg: 0x%02x",
               dut.api_extension_ctrl_reg, dut.wait_cycles_ctr_reg);

      $display("\n");
    end
  endtask // dump_dut_state


  //----------------------------------------------------------------
  // inc_tc_ctr
  //----------------------------------------------------------------
  task inc_tc_ctr;
    tc_ctr = tc_ctr + 1;
  endtask // inc_tc_ctr


  //----------------------------------------------------------------
  // display_test_results()
  //
  // Display the accumulated test results.
  //----------------------------------------------------------------
  task display_test_results;
    begin
      if (error_ctr == 0)
        begin
          $display("*** All %02d test cases completed successfully", tc_ctr);
        end
      else
        begin
          $display("*** %02d tests completed - %02d test cases did not complete successfully.",
                   tc_ctr, error_ctr);
        end
    end
  endtask // display_test_results


  //----------------------------------------------------------------
  // wait_ready()
  //----------------------------------------------------------------
  task wait_ready;
    begin
      $display("wait_ready: called.");
      #(4 * CLK_PERIOD);
      while (dut_status == 1'h1)
        #(CLK_PERIOD);
      $display("wait_ready: ready deasserted.");

      while (dut_status == 1'h0)
        #(CLK_PERIOD);
      $display("wait_ready: ready asserted.");
    end
  endtask // wait_ready


  //----------------------------------------------------------------
  // init_sim()
  //
  // Initialize all counters and testbed functionality as well
  // as setting the DUT inputs to defined values.
  //----------------------------------------------------------------
  task init_sim;
    begin
      cycle_ctr           = 0;
      error_ctr           = 0;
      tc_ctr              = 0;

      tb_clk              = 0;
      tb_reset            = 0;

      dut_command        = 2'h0;
      dut_address        = 32'h0;
      dut_write_data     = 32'h0;
      dut_dp_read_data   = 32'h0;
      dut_dp_ready       = 1'h0;
      dut_nts_read_data  = 32'h0;
      dut_nts_ready      = 1'h0;
      dut_rosc_read_data = 32'h0;
      dut_rosc_ready     = 1'h0;

      $display("*** init_sim() completed.");
      $display("\n");
    end
  endtask // init_sim


  //----------------------------------------------------------------
  // tc_verify_reset
  //----------------------------------------------------------------
  task tc_verify_reset;
    begin: tc_verify_reset;
      inc_tc_ctr();
      $display("*** tc_verify_reset started.");

      #(2 * CLK_PERIOD);
      $display("*** tc_verify_reset: Asserting reset.");
      tb_reset = 1;
      #(2 * CLK_PERIOD);
      tb_reset = 0;
      $display("*** tc_verify_reset: Deasserting reset.");
      #(2 * CLK_PERIOD);

      $display("*** tc_verify_reset completed.");
      $display("\n");
    end
  endtask // tc_verify_reset


  //----------------------------------------------------------------
  // tc_read_api_name_version
  // Test that we can read out name and version fields.
  //----------------------------------------------------------------
  task tc_read_api_name_version;
    begin
      inc_tc_ctr();
      $display("tc_read_api_name_version started.");


      dut_command = COMMAND_READ;
      dut_address = 32'h00000000;
      wait_ready();
      dut_command = COMMAND_IDLE;

      if (dut_read_data == dut.CORE_NAME0)
        $display("tc_read_api_name_version: Correct NAME0 read.");
      else
        $display("tc_read_api_name_version: Incorrect NAME0 read.");
      #(4 * CLK_PERIOD);


      dut_command = COMMAND_READ;
      dut_address = 32'h00000001;
      wait_ready();
      dut_command = COMMAND_IDLE;

      if (dut_read_data == dut.CORE_NAME1)
        $display("tc_read_api_name_version: Correct NAME1 read.");
      else
        $display("tc_read_api_name_version: Incorrect NAME1 read.");
      #(4 * CLK_PERIOD);


      dut_command = COMMAND_READ;
      dut_address = 32'h00000002;
      wait_ready();
      dut_command = COMMAND_IDLE;

      if (dut_read_data == dut.CORE_VERSION)
        $display("tc_read_api_name_version: Correct VERSION read.");
      else
        $display("tc_read_api_name_version: Incorrect VERSION read.");
      #(4 * CLK_PERIOD);


      $display("tc_read_api_name_version completed.");
      $display("\n");
    end
  endtask // tc_read_api_name_version


  //----------------------------------------------------------------
  // tc_write_operands_read_sum
  //----------------------------------------------------------------
  task tc_write_operands_read_sum;
    begin
      inc_tc_ctr();
      $display("tc_write_operands_read_sum: started.");


      dut_command    = COMMAND_WRITE;
      dut_address    = 32'h00000010;
      dut_write_data = 32'hdead0000;
      wait_ready();
      dut_command = COMMAND_IDLE;
      #(4 * CLK_PERIOD);


      dut_command    = COMMAND_WRITE;
      dut_address    = 32'h00000011;
      dut_write_data = 32'h0000beef;
      wait_ready();
      dut_command = COMMAND_IDLE;
      #(4 * CLK_PERIOD);

      dut_command = COMMAND_READ;
      dut_address = 32'h00000012;
      wait_ready();
      dut_command = COMMAND_IDLE;
      #(4 * CLK_PERIOD);

      if (dut_read_data == 32'hdeadbeef)
        $display("tc_read_api_name_version: Correct sum read.");
      else
        $display("tc_read_api_name_version: Incorrect sum read.");

      $display("tc_write_operands_read_sum: completed.");
      $display("\n");
    end
  endtask // tc_write_operands_read_sum


  //----------------------------------------------------------------
  // tc_read_rosc
  //----------------------------------------------------------------
  task tc_read_rosc;
    begin
      inc_tc_ctr();
      $display("tc_read_rosc: started.");

      dut_rosc_read_data = 32'h55aa55aa;
      dut_rosc_ready     = 1'h1;

      dut_command = COMMAND_READ;
      dut_address = 32'hfe000000;
      wait_ready();
      dut_command = COMMAND_IDLE;
      #(4 * CLK_PERIOD);

      if (dut_read_data == 32'h55aa55aa)
        $display("tc_read_rosc: Correct rosc data read.");
      else
        $display("tc_read_rosc: Incorrect sum read: 0x%08x",
                 dut_read_data);

      $display("tc_read_rosc: completed.");
      $display("\n");
    end
  endtask // tc_read_rosc


  //----------------------------------------------------------------
  // tc_write_dp
  //----------------------------------------------------------------
  task tc_write_dp;
    begin
      inc_tc_ctr();
      $display("tc_write_dp: started.");

      dut_dp_ready = 1'h1;

      dut_command = COMMAND_WRITE;
      dut_address = 32'h10000002;
      dut_write_data = 32'hbeefbeef;
      wait_ready();
      #(4 * CLK_PERIOD);

      if ((dut_dp_write_data == 32'hbeefbeef) &&
          (dut_dp_address == 24'h000002))
        $display("tc_write_dp: Correct data and address written.");
      else
        $display("tc_write_dp: Incorrect data and address written:: 0x%08x to 0x%06x",
                 dut_dp_write_data, dut_dp_address);

      #(4 * CLK_PERIOD);
      dut_command = COMMAND_IDLE;
      #(4 * CLK_PERIOD);

      $display("tc_write_dp: completed.");
      $display("\n");
    end
  endtask // tc_write_dp


  //----------------------------------------------------------------
  // tc_write_nts
  //----------------------------------------------------------------
  task tc_write_nts;
    begin
      inc_tc_ctr();
      $display("tc_write_nts0: started.");

      dut_nts_ready = 1'h1;

      dut_command = COMMAND_WRITE;
      dut_address = 32'h20000002;
      dut_write_data = 32'hff00ff00;
      wait_ready();
      #(4 * CLK_PERIOD);

      if ((dut_nts_write_data == 32'hff00ff00) &&
          (dut_nts_address == 24'h000002))
        $display("tc_write_nts: Correct data and address written.");
      else
        $display("tc_write_nts: Incorrect data and address written:: 0x%08x to 0x%06x",
                 dut_nts_write_data, dut_nts_address);

      #(4 * CLK_PERIOD);
      dut_command = COMMAND_IDLE;
      #(4 * CLK_PERIOD);

      $display("tc_write_nts0: completed.");
      $display("\n");
    end
  endtask // tc_write_nts


  //----------------------------------------------------------------
  // main
  //
  // The main test functionality.
  //----------------------------------------------------------------
  initial
    begin : main
      $display("   -= Testbench for API Extension started =-");
      $display("    ========================================");
      $display("");

      init_sim();
      tc_verify_reset();
      tc_read_api_name_version();
      tc_write_operands_read_sum();
      tc_read_rosc();
      tc_write_dp();
      tc_write_nts();
      display_test_results();

      $display("");
      $display("*** API Extension simulation done. ***");
      $finish;
    end // main
endmodule // tb_api_extension

//======================================================================
// EOF tb_api_extension.v
//======================================================================
