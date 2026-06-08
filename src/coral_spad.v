// ============================================================
// coral_spad.v
// Output Scratchpad — SRAM Wrapper
//
// Holds post-processed INT8 inference results before they are
// read by the trigger logic or the host via DMA.
//
// Capacity:  DEPTH × DATA_W bits
// Default:   256 × 8 = 256 bytes
//            Sufficient for 8×8×4 = 256 output values per tile pass.
//
// Write port: from post_proc (processed inference results)
// Read port:  from ctrl (trigger check) or host DMA (future)
//
// SKY130 migration note:
//   Same strategy as act_buf: single-port 1RW macro with time-sliced
//   access. post_proc writes after COMPUTE phase; trigger reads after
//   DONE phase. No overlap in steady-state operation.
// ============================================================

`default_nettype none
`timescale 1ns / 1ps

module coral_spad #(
    parameter DATA_W = 8,
    parameter DEPTH  = 256
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // Write port (post_proc)
    input  wire [$clog2(DEPTH)-1:0] wr_addr,
    input  wire                     wr_en,
    input  wire [DATA_W-1:0]        wr_data,

    // Read port (ctrl / trigger)
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

    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_data <= {DATA_W{1'b0}};
        else if (rd_en)
            rd_data <= mem[rd_addr];
    end

endmodule
`default_nettype wire
