// ============================================================
// coral_wgt_buf.v
// Weight Buffer — SRAM Wrapper
//
// Behavioral stub for a single-port SRAM.
// Intended to be replaced with a SKY130 OpenRAM macro instance
// (e.g., sky130_sram_1rw1r_32x256_8 or equivalent) in the next
// implementation phase.
//
// Capacity:  DEPTH × DATA_W bits
// Default:   512 × 8 = 4096 bits = 512 bytes (enough for an
//            8×8 filter bank with some headroom)
//
// Read latency: 1 cycle (registered output)
// Write priority: synchronous, write-before-read on same address
//
// SKY130 migration note:
//   Replace the reg array and always block below with a macro
//   instantiation. Preserve rd_data, wr_en, rd_en, addr ports.
//   The macro wrapper should absorb any timing/enable differences.
// ============================================================

`default_nettype none
`timescale 1ns / 1ps

module coral_wgt_buf #(
    parameter DATA_W = 8,
    parameter DEPTH  = 512
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // Read port
    input  wire [$clog2(DEPTH)-1:0] rd_addr,
    input  wire                     rd_en,
    output reg  [DATA_W-1:0]        rd_data,

    // Write port (host DMA fills weights before inference starts)
    input  wire [$clog2(DEPTH)-1:0] wr_addr,
    input  wire                     wr_en,
    input  wire [DATA_W-1:0]        wr_data
);

    // ----------------------------------------------------------------
    // Behavioral SRAM model (synthesizes to flops — replace with macro)
    // ----------------------------------------------------------------
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

    // Synchronous read (1-cycle latency)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_data <= {DATA_W{1'b0}};
        else if (rd_en)
            rd_data <= mem[rd_addr];
    end

endmodule
`default_nettype wire
