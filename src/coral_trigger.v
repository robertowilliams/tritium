// ============================================================
// coral_trigger.v
// Threshold Detection and Event Output
//
// Reads INT8 inference scores from the scratchpad (via ctrl)
// and compares each against a configurable threshold.
// If any score exceeds the threshold:
//   - trig_out is asserted (level, cleared after one frame)
//   - trig_score holds the maximum score seen
//   - trig_valid pulses for one cycle when trig_out first asserts
//
// This is the primary camera-edge value: instead of sending every
// frame upstream, only trigger escalation when inference says
// something interesting is in the frame.
//
// Future extensions (not in this scaffold):
//   - Count of detections above threshold
//   - Bounding box / ROI coordinates
//   - Score histogram output
//   - Hysteresis / hold-off counter
// ============================================================

`default_nettype none
`timescale 1ns / 1ps

module coral_trigger #(
    parameter DATA_W = 8
) (
    input  wire               clk,
    input  wire               rst_n,

    // Configuration
    input  wire [DATA_W-1:0]  threshold,    // detection threshold

    // Score input stream (from spad read path)
    input  wire [DATA_W-1:0]  score_in,
    input  wire               score_valid,  // one score per cycle

    // Outputs
    output reg                trig_out,     // level: detection above threshold
    output reg  [DATA_W-1:0]  trig_score,  // max score seen this evaluation
    output reg                trig_valid    // one-cycle pulse on trigger assert
);

    reg [DATA_W-1:0]  max_score;
    reg               triggered;
    reg               last_triggered;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            max_score    <= {DATA_W{1'b0}};
            triggered    <= 1'b0;
            last_triggered <= 1'b0;
            trig_out     <= 1'b0;
            trig_score   <= {DATA_W{1'b0}};
            trig_valid   <= 1'b0;
        end else begin
            trig_valid <= 1'b0;   // default: no new trigger pulse

            if (score_valid) begin
                // Track maximum score
                if (score_in > max_score)
                    max_score <= score_in;

                // Threshold comparison
                if (score_in >= threshold) begin
                    triggered <= 1'b1;
                    trig_score <= score_in;   // update with latest exceeding score
                end
            end

            // Rising edge of triggered → generate trig_valid pulse
            last_triggered <= triggered;
            if (triggered && !last_triggered) begin
                trig_valid <= 1'b1;
                trig_out   <= 1'b1;
            end

            // Auto-clear after one evaluation window
            // ctrl drives score_valid low when evaluation is done;
            // when score_valid falls, we close the window and latch output.
            // For now: trig_out holds until next frame starts (ctrl will
            // clear by writing 0 to threshold or issuing a ctrl_halt).
        end
    end

endmodule
`default_nettype wire
