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
  parameter DEBUG     = 0;

  parameter CLK_HALF_PERIOD = 1;
  parameter CLK_PERIOD      = 2 * CLK_HALF_PERIOD;

  // API
  // Address space prefixes. We support 256 new top level modules.
  localparam API_PREFIX       = 8'h00;
  localparam ROSC_PREFIX      = 8'hfe;
  localparam NTS0_PREFIX      = 8'h10;

  // The API for the API extension itself.
  localparam API_ADDR_NAME0   = 8'h00;
  localparam API_ADDR_NAME1   = 8'h01;
  localparam API_ADDR_VERSION = 8'h02;

  localparam API_ADDR_OP_A    = 8'h10;
  localparam API_ADDR_OP_B    = 8'h11;
  localparam API_ADDR_SUM     = 8'h12;


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
  wire          dut_nts0_cs;
  wire          dut_nts0_we;
  wire [23 : 0] dut_nts0_address;
  wire [31 : 0] dut_nts0_write_data;
  reg  [31 : 0] dut_nts0_read_data;
  reg           dut_nts0_ready;
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
                    .nts0_cs(dut_nts0_cs),
                    .nts0_we(dut_nts0_we),
                    .nts0_address(dut_nts0_address),
                    .nts0_write_data(dut_nts0_write_data),
                    .nts0_read_data(dut_nts0_read_data),
                    .nts0_ready(dut_nts0_ready),
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
      $display("\n");
    end
  endtask // dump_dut_state


  //----------------------------------------------------------------
  // reset_dut()
  //
  // Toggle reset to put the DUT into a well known state.
  //----------------------------------------------------------------
  task reset_dut;
    begin
      tb_reset = 1;
      #(2 * CLK_PERIOD);
      tb_reset = 0;
    end
  endtask // reset_dut


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
      dut_nts0_read_data = 32'h0;
      dut_nts0_ready     = 1'h0;
      dut_rosc_read_data = 32'h0;
      dut_rosc_ready     = 1'h0;

      $display("*** init_sim() completed.");
    end
  endtask // init_sim


  //----------------------------------------------------------------
  // tc01_verify_reset
  //----------------------------------------------------------------
  task tc01_verify_reset;
    begin: tc01_verify_reset;
        $display("*** TC01: Verify Reset completed");
        error_ctr = error_ctr + 1;
      end
  endtask // tc01_verify_reset


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
      display_test_results();

      $display("");
      $display("*** API Extension simulation done. ***");
      $finish;
    end // main
endmodule // tb_api_extension

//======================================================================
// EOF tb_api_extension.v
//======================================================================
