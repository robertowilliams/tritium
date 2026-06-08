// ============================================================
// coral_ctrl.v
// Control / Sequencer FSM
//
// Orchestrates the NPU compute pipeline:
//
//  IDLE ──start──► WAIT_FRAME ──frame_rdy──► LOAD_WEIGHTS
//       ──────────────────────────────────────────────────►
//  LOAD_WEIGHTS ──► COMPUTE_TILE ──► POST_PROC ──► CHECK_DONE
//  CHECK_DONE ──more_tiles──► COMPUTE_TILE
//  CHECK_DONE ──all_done──► TRIGGER_CHECK ──► DONE
//  DONE ──► IDLE (auto-clear after one cycle)
//
//  HALT input forces ──► IDLE from any state
//
// Tiling strategy:
//   The image is processed in tiles of (ARRAY_R x ARRAY_C) pixels.
//   Each tile computes one MAC-array output block.
//   tile_row, tile_col track position in the output feature map.
//
// Memory addressing:
//   Weight:     wgt_rd_addr = tile_row * ARRAY_C + mac_col
//   Activation: act_rd_addr = (tile_row*ARRAY_R + row) * img_w
//                            + (tile_col*ARRAY_C + mac_col)
// ============================================================

`default_nettype none
`timescale 1ns / 1ps

module coral_ctrl #(
    parameter DATA_W  = 8,
    parameter ARRAY_R = 8,
    parameter ARRAY_C = 8
) (
    input  wire         clk,
    input  wire         rst_n,

    // From host_if
    input  wire         ctrl_start,
    input  wire         ctrl_halt,
    input  wire [7:0]   cfg_img_w,
    input  wire [7:0]   cfg_img_h,
    input  wire [15:0]  cfg_wgt_base,
    input  wire [15:0]  cfg_act_base,

    // From cam_if
    input  wire         cam_frame_rdy,

    // Status outputs → host_if
    output reg          stat_busy,
    output reg          stat_done,
    output reg          stat_error,
    output reg  [3:0]   stat_phase,

    // Weight buffer interface
    output reg  [15:0]  wgt_rd_addr,
    output reg          wgt_rd_en,
    input  wire [DATA_W-1:0] wgt_rd_data,

    // Activation buffer interface
    output reg  [15:0]  act_rd_addr,
    output reg          act_rd_en,
    input  wire [DATA_W-1:0] act_rd_data,

    // MAC array control
    output reg  [ARRAY_C*DATA_W-1:0] mac_wgt_row,
    output reg  [ARRAY_R*DATA_W-1:0] mac_act_col,
    output reg          mac_load_wgt,
    output reg          mac_compute,
    output reg          mac_clear,

    // Scratchpad read (trigger reads from spad)
    output reg  [15:0]  spad_rd_addr,
    output reg          spad_rd_en
);

    // ----------------------------------------------------------------
    // State encoding
    // ----------------------------------------------------------------
    localparam [3:0]
        S_IDLE          = 4'd0,
        S_WAIT_FRAME    = 4'd1,
        S_LOAD_WGT      = 4'd2,
        S_LOAD_ACT      = 4'd3,
        S_COMPUTE       = 4'd4,
        S_POST_WAIT     = 4'd5,
        S_NEXT_TILE     = 4'd6,
        S_TRIG_CHECK    = 4'd7,
        S_DONE          = 4'd8,
        S_ERROR         = 4'd9;

    reg [3:0] state, next_state;

    // ----------------------------------------------------------------
    // Tile counters
    // ----------------------------------------------------------------
    reg [7:0]  tile_row;    // current tile row (in units of ARRAY_R pixels)
    reg [7:0]  tile_col;    // current tile column (in units of ARRAY_C pixels)
    reg [7:0]  mac_row;     // current row inside MAC array
    reg [7:0]  mac_col;     // current col inside MAC array
    reg [7:0]  load_step;   // generic step counter within a state

    // Derived: total tiles
    wire [7:0] n_tile_rows = (cfg_img_h + ARRAY_R - 1) / ARRAY_R;
    wire [7:0] n_tile_cols = (cfg_img_w + ARRAY_C - 1) / ARRAY_C;

    // ----------------------------------------------------------------
    // State register
    // ----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_IDLE;
        else if (ctrl_halt)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // Expose state as phase
    always @(*) stat_phase = state;

    // ----------------------------------------------------------------
    // Next-state logic
    // ----------------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE:
                if (ctrl_start) next_state = S_WAIT_FRAME;

            S_WAIT_FRAME:
                if (cam_frame_rdy) next_state = S_LOAD_WGT;

            S_LOAD_WGT:
                // load_step counts through ARRAY_R rows of weights
                if (load_step == ARRAY_R - 1) next_state = S_LOAD_ACT;

            S_LOAD_ACT:
                // load_step counts through ARRAY_C activation columns
                if (load_step == ARRAY_C - 1) next_state = S_COMPUTE;

            S_COMPUTE:
                // one cycle compute per column, ARRAY_C columns
                if (load_step == ARRAY_C - 1) next_state = S_POST_WAIT;

            S_POST_WAIT:
                // wait for post-processing pipeline to flush (2 cycles)
                if (load_step == 2) next_state = S_NEXT_TILE;

            S_NEXT_TILE: begin
                if (tile_col + 1 < n_tile_cols)
                    next_state = S_LOAD_WGT;
                else if (tile_row + 1 < n_tile_rows)
                    next_state = S_LOAD_WGT;
                else
                    next_state = S_TRIG_CHECK;
            end

            S_TRIG_CHECK:
                if (load_step == 4) next_state = S_DONE;

            S_DONE:
                next_state = S_IDLE;

            S_ERROR:
                next_state = S_IDLE;

            default:
                next_state = S_IDLE;
        endcase
    end

    // ----------------------------------------------------------------
    // Datapath and output logic
    // ----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stat_busy    <= 1'b0;
            stat_done    <= 1'b0;
            stat_error   <= 1'b0;
            wgt_rd_addr  <= 16'h0;
            wgt_rd_en    <= 1'b0;
            act_rd_addr  <= 16'h0;
            act_rd_en    <= 1'b0;
            mac_wgt_row  <= {(ARRAY_C*DATA_W){1'b0}};
            mac_act_col  <= {(ARRAY_R*DATA_W){1'b0}};
            mac_load_wgt <= 1'b0;
            mac_compute  <= 1'b0;
            mac_clear    <= 1'b0;
            spad_rd_addr <= 16'h0;
            spad_rd_en   <= 1'b0;
            tile_row     <= 8'h0;
            tile_col     <= 8'h0;
            mac_row      <= 8'h0;
            mac_col      <= 8'h0;
            load_step    <= 8'h0;
        end else begin
            // Default deassert
            wgt_rd_en    <= 1'b0;
            act_rd_en    <= 1'b0;
            mac_load_wgt <= 1'b0;
            mac_compute  <= 1'b0;
            mac_clear    <= 1'b0;
            spad_rd_en   <= 1'b0;
            stat_done    <= 1'b0;

            case (state)
                S_IDLE: begin
                    stat_busy  <= 1'b0;
                    stat_error <= 1'b0;
                    tile_row   <= 8'h0;
                    tile_col   <= 8'h0;
                    load_step  <= 8'h0;
                    mac_clear  <= 1'b1;   // ensure MAC is clear on idle
                end

                S_WAIT_FRAME: begin
                    stat_busy <= 1'b1;
                    mac_clear <= 1'b1;
                end

                S_LOAD_WGT: begin
                    // Issue weight read for current row
                    // Weight layout: row-major, each row = ARRAY_C weights
                    wgt_rd_addr <= cfg_wgt_base
                                 + tile_row * ARRAY_R * ARRAY_C
                                 + load_step * ARRAY_C;
                    wgt_rd_en   <= 1'b1;
                    // On the next cycle wgt_rd_data is valid (1-cycle SRAM latency)
                    // This is a simplified model — real impl needs pipeline delay
                    if (load_step > 0) begin
                        // Store previous cycle's read data into mac_wgt_row
                        // (placeholder — real impl needs a shift register here)
                        mac_wgt_row <= {mac_wgt_row[ARRAY_C*DATA_W-DATA_W-1:0], wgt_rd_data};
                        mac_load_wgt <= (load_step == ARRAY_R); // load when full row ready
                    end
                    if (load_step == ARRAY_R - 1)
                        load_step <= 8'h0;
                    else
                        load_step <= load_step + 8'h1;
                end

                S_LOAD_ACT: begin
                    // Issue activation read for current column
                    act_rd_addr <= cfg_act_base
                                 + (tile_row * ARRAY_R) * cfg_img_w
                                 + (tile_col * ARRAY_C)
                                 + load_step;
                    act_rd_en <= 1'b1;
                    if (load_step > 0)
                        mac_act_col <= {mac_act_col[ARRAY_R*DATA_W-DATA_W-1:0], act_rd_data};
                    if (load_step == ARRAY_C - 1)
                        load_step <= 8'h0;
                    else
                        load_step <= load_step + 8'h1;
                end

                S_COMPUTE: begin
                    mac_compute <= 1'b1;
                    if (load_step == ARRAY_C - 1)
                        load_step <= 8'h0;
                    else
                        load_step <= load_step + 8'h1;
                end

                S_POST_WAIT: begin
                    load_step <= load_step + 8'h1;
                end

                S_NEXT_TILE: begin
                    load_step <= 8'h0;
                    mac_clear <= 1'b1;
                    if (tile_col + 1 < n_tile_cols) begin
                        tile_col <= tile_col + 8'h1;
                    end else begin
                        tile_col <= 8'h0;
                        tile_row <= tile_row + 8'h1;
                    end
                end

                S_TRIG_CHECK: begin
                    // Read first few output values from spad for trigger eval
                    spad_rd_addr <= load_step[15:0];
                    spad_rd_en   <= 1'b1;
                    load_step    <= load_step + 8'h1;
                end

                S_DONE: begin
                    stat_busy <= 1'b0;
                    stat_done <= 1'b1;   // one-cycle pulse
                    tile_row  <= 8'h0;
                    tile_col  <= 8'h0;
                    load_step <= 8'h0;
                end

                S_ERROR: begin
                    stat_busy  <= 1'b0;
                    stat_error <= 1'b1;
                end

                default: ;
            endcase
        end
    end

endmodule
`default_nettype wire
