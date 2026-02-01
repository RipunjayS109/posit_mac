`default_nettype none
`timescale 1ns / 1ps

/* This testbench just instantiates the module and makes some convenient wires
   that can be driven / tested by the cocotb test.py.
*/
module tb ();

  // Dump the signals to a FST file. You can view it with gtkwave or surfer.
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // Replace tt_um_example with your module name:
  tt_um_posit_mac_stream u_user (
    .ui_in   (ui_in),    // Dedicated inputs
    .uo_out  (uo_out),   // Dedicated outputs

    .uio_in  (uio_in),   // Bidirectional IOs used as inputs
    .uio_out (uio_out),  // Must be connected (even if driven to 0 internally)
    .uio_oe  (uio_oe),   // Must be connected (even if driven to 0 internally)

    .ena     (ena),      // Enable (from TT harness)
    .clk     (clk),      // Clock
    .rst_n   (rst_n)     // Active-low reset
);


endmodule
