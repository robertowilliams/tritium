// ============================================================
// coral_mac_array.v
// INT8 Systolic-Inspired MAC Array
//
// Implements an ARRAY_R × ARRAY_C grid of MAC processing elements.
// Dataflow: weight-stationary
//   - Weights are preloaded into each PE's weight register
//   - Activations stream in column-by-column
//   - Each PE accumulates: acc += weight * activation
//   - Accumulator is 32-bit to prevent overflow on long dot products
//
// Interface:
//   wgt_row    — one row of ARRAY_C weights, broadcast to one PE row
//   act_col    — one column of ARRAY_R activations, broadcast to one PE col
//   load_wgt   — strobe: latch wgt_row into PE weight registers for
//                current row (ctrl sequences rows externally)
//   compute    — strobe: one MAC step (acc += wgt * act)
//   clear      — synchronous clear all accumulators
//   accum_out  — flat packed output: [row][col] each ACCUM_W bits
//   accum_valid — pulses high the cycle after compute (1-cycle latency)
//
// Physical design note:
//   At 8×8 = 64 PEs with 32-bit accumulators, this is 2048 flop-bits
//   for state alone plus 64 multipliers. On SKY130 at modest util,
//   this is manageable. Scale down to 4×4 if synthesis is too heavy.
// ============================================================

`default_nettype none
`timescale 1ns / 1ps

module coral_mac_array #(
    parameter DATA_W  = 8,
    parameter ACCUM_W = 32,
    parameter ARRAY_R = 8,    // rows (output feature map rows per tile)
    parameter ARRAY_C = 8     // columns (output feature map cols per tile)
) (
    input  wire                               clk,
    input  wire                               rst_n,

    // Weight loading (one PE row at a time)
    input  wire [ARRAY_C*DATA_W-1:0]          wgt_row,   // ARRAY_C weights
    input  wire                               load_wgt,  // latch wgt_row

    // Activation input (one PE column at a time)
    input  wire [ARRAY_R*DATA_W-1:0]          act_col,   // ARRAY_R activations

    // Control
    input  wire                               compute,   // acc += wgt * act
    input  wire                               clear,     // zero all accumulators

    // Output
    output reg  [ARRAY_R*ARRAY_C*ACCUM_W-1:0] accum_out,
    output reg                                accum_valid
);

    // ----------------------------------------------------------------
    // PE weight registers: [row][col]
    // Flat: wgt_reg[r*ARRAY_C + c] = weight for PE(r,c)
    // ----------------------------------------------------------------
    reg signed [DATA_W-1:0]  wgt_reg  [0:ARRAY_R*ARRAY_C-1];
    reg signed [ACCUM_W-1:0] accum    [0:ARRAY_R*ARRAY_C-1];

    // Weight load row pointer — ctrl sequences this from outside
    // We use a shadow row register that ctrl fills, then load_wgt latches it
    // into the appropriate PE row. We track which row to write via a counter
    // that increments on each load_wgt.
    reg [7:0] wgt_load_row;

    // ----------------------------------------------------------------
    // Weight load logic
    // ----------------------------------------------------------------
    integer ci;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wgt_load_row <= 8'h0;
            for (ci = 0; ci < ARRAY_R * ARRAY_C; ci = ci + 1)
                wgt_reg[ci] <= {DATA_W{1'b0}};
        end else begin
            if (load_wgt) begin
                // Load one row of weights
                // wgt_row is packed: [col_0 | col_1 | ... | col_{C-1}]
                // col 0 at LSB
                for (ci = 0; ci < ARRAY_C; ci = ci + 1)
                    wgt_reg[wgt_load_row * ARRAY_C + ci] <=
                        $signed(wgt_row[(ci+1)*DATA_W-1 -: DATA_W]);
                // Advance row pointer, wrap around
                if (wgt_load_row == ARRAY_R - 1)
                    wgt_load_row <= 8'h0;
                else
                    wgt_load_row <= wgt_load_row + 8'h1;
            end
        end
    end

    // ----------------------------------------------------------------
    // Accumulator logic
    // ----------------------------------------------------------------
    integer ri, ci2, idx;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (idx = 0; idx < ARRAY_R * ARRAY_C; idx = idx + 1)
                accum[idx] <= {ACCUM_W{1'b0}};
            accum_valid <= 1'b0;
        end else begin
            accum_valid <= 1'b0;

            if (clear) begin
                for (idx = 0; idx < ARRAY_R * ARRAY_C; idx = idx + 1)
                    accum[idx] <= {ACCUM_W{1'b0}};
            end else if (compute) begin
                // Each PE(r,c): accum += wgt_reg[r][c] * act_col[r]
                for (ri = 0; ri < ARRAY_R; ri = ri + 1) begin
                    for (ci2 = 0; ci2 < ARRAY_C; ci2 = ci2 + 1) begin
                        accum[ri * ARRAY_C + ci2] <=
                            accum[ri * ARRAY_C + ci2]
                            + $signed(wgt_reg[ri * ARRAY_C + ci2])
                              * $signed(act_col[(ri+1)*DATA_W-1 -: DATA_W]);
                    end
                end
                accum_valid <= 1'b1;
            end
        end
    end

    // ----------------------------------------------------------------
    // Pack accumulators into output bus
    // Output packing: [row=0,col=0] at LSB, row-major
    // ----------------------------------------------------------------
    integer oi;
    always @(*) begin
        for (oi = 0; oi < ARRAY_R * ARRAY_C; oi = oi + 1)
            accum_out[(oi+1)*ACCUM_W-1 -: ACCUM_W] = accum[oi];
    end

endmodule
`default_nettype wire
