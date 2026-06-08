// ============================================================
// coral_act_buf.v
// Activation Buffer — SRAM Wrapper
//
// Dual-port behavioral stub: one write port (from cam_if),
// one read port (from ctrl). On SKY130, replace with a
// 1RW or 2-port OpenRAM macro.
//
// Capacity:  DEPTH × DATA_W bits
// Default:   256 × 8 = 2048 bits = 256 bytes
//            Enough for a 16×16 grayscale image or 64-pixel ROI strip.
//
// Read latency: 1 cycle registered
//
// SKY130 migration note:
//   If a true-dual-port SRAM macro is not available in SKY130
//   OpenRAM, use a 1RW macro and arbitrate wr/rd via time-slicing.
//   Cam_if writes during WAIT_FRAME; ctrl reads during COMPUTE.
//   These are naturally non-overlapping phases — single-port works.
// ============================================================

`default_nettype none
`timescale 1ns / 1ps

module coral_act_buf #(
    parameter DATA_W = 8,
    parameter DEPTH  = 256
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // Write port (cam_if)
    input  wire [$clog2(DEPTH)-1:0] wr_addr,
    input  wire                     wr_en,
    input  wire [DATA_W-1:0]        wr_data,

    // Read port (ctrl)
    input  wire [$clog2(DEPTH)-1:0] rd_addr,
    input  wire                     rd_en,
    output reg  [DATA_W-1:0]        rd_data
);

    reg [DATA_W-1:0] mem [0:DEPTH-1];

    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1)
            mem[i] = {DATA_W{1'b0}};
    end

    // Synchronous write
    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    // Synchronous read
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_data <= {DATA_W{1'b0}};
        else if (rd_en)
            rd_data <= mem[rd_addr];
    end

endmodule
`default_nettype wire
