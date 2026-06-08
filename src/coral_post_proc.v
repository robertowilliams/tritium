// ============================================================
// coral_post_proc.v
// Post-Processing Unit — ReLU, Quantization Shift, Output Pack
//
// Takes the 32-bit accumulator outputs from the MAC array and
// produces INT8 outputs suitable for the scratchpad.
//
// Pipeline stages (2-cycle latency):
//   Stage 1: ReLU (clip negatives to zero) + arithmetic right-shift
//   Stage 2: Saturating clamp to [0, 255] and pack to INT8
//
// cfg inputs:
//   relu_en  — enable ReLU (disable for intermediate layers if needed)
//   shift    — right-shift amount (4-bit, supports 0–15 positions)
//              Used to re-quantize the 32-bit accumulator back to INT8
//              scale: output = clamp(ReLU(acc >> shift), 0, 255)
//
// Output:
//   out_data  — flat packed INT8 results, same order as accum_in
//   out_valid — high when out_data is valid (2 cycles after in_valid)
// ============================================================

`default_nettype none
`timescale 1ns / 1ps

module coral_post_proc #(
    parameter DATA_W  = 8,
    parameter ACCUM_W = 32,
    parameter ARRAY_R = 8,
    parameter ARRAY_C = 8
) (
    input  wire                                     clk,
    input  wire                                     rst_n,

    // Configuration
    input  wire                                     relu_en,
    input  wire [3:0]                               shift,

    // Input from MAC array
    input  wire [ARRAY_R*ARRAY_C*ACCUM_W-1:0]      accum_in,
    input  wire                                     in_valid,

    // Output to scratchpad
    output reg  [ARRAY_R*ARRAY_C*DATA_W-1:0]        out_data,
    output reg                                      out_valid
);

    localparam N_PE = ARRAY_R * ARRAY_C;

    // ----------------------------------------------------------------
    // Stage 1 registers: ReLU + shift
    // ----------------------------------------------------------------
    reg signed [ACCUM_W-1:0] stage1 [0:N_PE-1];
    reg                      stage1_valid;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stage1_valid <= 1'b0;
            for (i = 0; i < N_PE; i = i + 1)
                stage1[i] <= {ACCUM_W{1'b0}};
        end else begin
            stage1_valid <= in_valid;
            if (in_valid) begin
                for (i = 0; i < N_PE; i = i + 1) begin
                    // Extract accumulator value (signed)
                    stage1[i] <= $signed(accum_in[(i+1)*ACCUM_W-1 -: ACCUM_W]) >>> shift;
                end
            end
        end
    end

    // ----------------------------------------------------------------
    // Stage 2: ReLU clamp + saturate to [0, 255] + pack to INT8
    // ----------------------------------------------------------------
    reg [DATA_W-1:0] stage2 [0:N_PE-1];

    // Saturation function: clamp signed 32-bit to unsigned [0,255]
    // This is a combinational function used in the stage-2 always block
    function [DATA_W-1:0] saturate;
        input signed [ACCUM_W-1:0] val;
        input                       do_relu;
        reg signed [ACCUM_W-1:0]   clamped;
        begin
            if (do_relu && val[ACCUM_W-1]) begin
                // Negative after ReLU → zero
                saturate = {DATA_W{1'b0}};
            end else if (val > $signed({{(ACCUM_W-DATA_W){1'b0}}, {DATA_W{1'b1}}}) ) begin
                // Overflow → saturate high
                saturate = {DATA_W{1'b1}};
            end else if (val < 0) begin
                // Negative without ReLU → saturate low (zero for unsigned)
                saturate = {DATA_W{1'b0}};
            end else begin
                saturate = val[DATA_W-1:0];
            end
        end
    endfunction

    integer j;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            for (j = 0; j < N_PE; j = j + 1)
                stage2[j] <= {DATA_W{1'b0}};
        end else begin
            out_valid <= stage1_valid;
            if (stage1_valid) begin
                for (j = 0; j < N_PE; j = j + 1)
                    stage2[j] <= saturate(stage1[j], relu_en);
            end
        end
    end

    // ----------------------------------------------------------------
    // Pack stage2 array into flat output bus
    // ----------------------------------------------------------------
    integer k;
    always @(*) begin
        for (k = 0; k < N_PE; k = k + 1)
            out_data[(k+1)*DATA_W-1 -: DATA_W] = stage2[k];
    end

endmodule
`default_nettype wire
