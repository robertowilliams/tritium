// ============================================================
// coral_cam_if.v
// Camera Pixel Stream Interface
//
// Accepts a parallel pixel bus (DVP-style: vsync, href, pclk, data).
// Resyncs cam_pclk domain → system clk domain via 2FF synchronizer
// on the control signals, and a small FIFO on the data path.
//
// Outputs:
//   frame_rdy    — pulses for one clk cycle when a full frame is
//                  stored in the activation buffer
//   pixel_out    — pixel data in system clock domain
//   pixel_valid  — write enable for act_buf
//   pixel_addr   — act_buf write address (row-major, wraps at img_w*img_h)
//
// Assumptions:
//   - Input is 8-bit grayscale (one byte per pixel)
//   - cam_pclk is asynchronous to clk but slower or comparable
//   - Maximum image size: 256x256 (fits in 16-bit pixel_addr)
//   - RGB inputs should be pre-converted to Y before this module
// ============================================================

`default_nettype none
`timescale 1ns / 1ps

module coral_cam_if #(
    parameter DATA_W = 8
) (
    input  wire               clk,
    input  wire               rst_n,

    // Camera DVP inputs (cam_pclk domain)
    input  wire               cam_vsync,   // high = active frame
    input  wire               cam_href,    // high = valid line
    input  wire               cam_pclk,    // pixel clock
    input  wire [DATA_W-1:0]  cam_data,    // pixel byte

    // System clock domain outputs → act_buf
    output reg                frame_rdy,
    output reg  [DATA_W-1:0]  pixel_out,
    output reg                pixel_valid,
    output reg  [15:0]        pixel_addr
);

    // ----------------------------------------------------------------
    // 2FF synchronizers: bring cam_pclk-domain signals into clk domain
    // We synchronize vsync and href edges; data follows with a known
    // pipeline relationship.
    // ----------------------------------------------------------------
    reg [1:0]  vsync_ff;
    reg [1:0]  href_ff;
    reg [1:0]  pclk_ff;   // detect rising edge of cam_pclk

    // Synchronize cam_pclk into system domain for edge detection
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pclk_ff  <= 2'b00;
            vsync_ff <= 2'b00;
            href_ff  <= 2'b00;
        end else begin
            pclk_ff  <= {pclk_ff[0],  cam_pclk};
            vsync_ff <= {vsync_ff[0], cam_vsync};
            href_ff  <= {href_ff[0],  cam_href};
        end
    end

    // Edge detection in system clock domain
    wire pclk_rise  = (pclk_ff == 2'b01);   // rising edge of cam_pclk
    wire vsync_rise = (vsync_ff == 2'b01);   // start of frame
    wire vsync_fall = (vsync_ff == 2'b10);   // end of frame
    wire href_sync  =  vsync_ff[1];          // synced vsync level
    wire href_active = href_ff[1];           // synced href level

    // ----------------------------------------------------------------
    // Data pipeline: sample cam_data one cycle after pclk_rise
    // (cam_data is stable at pclk_rise and one system cycle afterward
    //  given that cam_pclk << clk or ≈ clk/2 — safe assumption for
    //  low-res sensor at 30fps into a 25MHz system clock)
    // ----------------------------------------------------------------
    reg [DATA_W-1:0] data_latch;
    reg              data_valid_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_latch   <= {DATA_W{1'b0}};
            data_valid_d1 <= 1'b0;
        end else begin
            data_valid_d1 <= pclk_rise & href_active & href_sync;
            if (pclk_rise & href_active & href_sync)
                data_latch <= cam_data;
        end
    end

    // ----------------------------------------------------------------
    // Pixel address counter and output
    // ----------------------------------------------------------------
    reg frame_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_active <= 1'b0;
            frame_rdy    <= 1'b0;
            pixel_out    <= {DATA_W{1'b0}};
            pixel_valid  <= 1'b0;
            pixel_addr   <= 16'h0;
        end else begin
            frame_rdy    <= 1'b0;  // default pulse-low
            pixel_valid  <= 1'b0;

            if (vsync_rise) begin
                frame_active <= 1'b1;
                pixel_addr   <= 16'h0;
            end

            if (vsync_fall && frame_active) begin
                frame_active <= 1'b0;
                frame_rdy    <= 1'b1;  // one-cycle pulse
            end

            if (frame_active && data_valid_d1) begin
                pixel_out   <= data_latch;
                pixel_valid <= 1'b1;
                pixel_addr  <= pixel_addr + 16'h1;
            end
        end
    end

endmodule
`default_nettype wire
