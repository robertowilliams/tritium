// ============================================================
// coral_npu_top.v
// Smart Camera Accelerator — Top-Level Integration Wrapper
// Coral-derived NPU for SkyWater SKY130
//
// Hierarchy:
//   coral_npu_top
//   ├── coral_host_if    APB slave, register map
//   ├── coral_cam_if     Camera pixel stream input
//   ├── coral_ctrl       Control / sequencer FSM
//   ├── coral_mac_array  INT8 systolic-inspired MAC array
//   ├── coral_wgt_buf    Weight buffer SRAM wrapper
//   ├── coral_act_buf    Activation buffer SRAM wrapper
//   ├── coral_spad       Output scratchpad SRAM wrapper
//   ├── coral_post_proc  ReLU, quantization shift, pooling
//   └── coral_trigger    Threshold detection, event output
// ============================================================

`default_nettype none
`timescale 1ns / 1ps

module coral_npu_top #(
    parameter DATA_W    = 8,    // INT8 data width
    parameter ACCUM_W   = 32,   // Accumulator width
    parameter ARRAY_R   = 8,    // MAC array rows
    parameter ARRAY_C   = 8,    // MAC array columns
    parameter WGT_DEPTH = 512,  // Weight buffer depth (words)
    parameter ACT_DEPTH = 256,  // Activation buffer depth (words)
    parameter SPAD_DEPTH= 256   // Scratchpad depth (words)
) (
    // ---- Global ----
    input  wire                 clk,
    input  wire                 rst_n,          // active-low sync reset

    // ---- APB Host Interface ----
    input  wire [11:0]          apb_paddr,
    input  wire                 apb_psel,
    input  wire                 apb_penable,
    input  wire                 apb_pwrite,
    input  wire [31:0]          apb_pwdata,
    output wire [31:0]          apb_prdata,
    output wire                 apb_pready,
    output wire                 apb_pslverr,

    // ---- Camera Pixel Stream Input ----
    input  wire                 cam_vsync,      // frame start
    input  wire                 cam_href,       // line valid
    input  wire                 cam_pclk,       // pixel clock (async, resync inside)
    input  wire [DATA_W-1:0]    cam_data,       // pixel data (grayscale INT8)

    // ---- Trigger / Event Output ----
    output wire                 trig_out,       // high when detection threshold met
    output wire [7:0]           trig_score,     // detection confidence score
    output wire                 trig_valid,     // pulses with trig_out

    // ---- Interrupt ----
    output wire                 irq             // level interrupt to host
);

    // ----------------------------------------------------------------
    // Internal wiring: host_if → ctrl
    // ----------------------------------------------------------------
    wire        ctrl_start;
    wire        ctrl_halt;
    wire [7:0]  cfg_img_w;
    wire [7:0]  cfg_img_h;
    wire [15:0] cfg_wgt_base;
    wire [15:0] cfg_act_base;
    wire [7:0]  cfg_trig_thresh;
    wire        cfg_relu_en;
    wire [3:0]  cfg_shift;

    // ----------------------------------------------------------------
    // Internal wiring: ctrl → status
    // ----------------------------------------------------------------
    wire        stat_busy;
    wire        stat_done;
    wire        stat_error;
    wire [3:0]  stat_phase;

    // ----------------------------------------------------------------
    // Internal wiring: cam_if → ctrl / act_buf
    // ----------------------------------------------------------------
    wire                    cam_frame_rdy;
    wire [DATA_W-1:0]       cam_pixel_out;
    wire                    cam_pixel_valid;
    wire [15:0]             cam_pixel_addr;

    // ----------------------------------------------------------------
    // Internal wiring: ctrl → wgt_buf
    // ----------------------------------------------------------------
    wire [15:0]             wgt_rd_addr;
    wire                    wgt_rd_en;
    wire [DATA_W-1:0]       wgt_rd_data;

    // ----------------------------------------------------------------
    // Internal wiring: ctrl → act_buf
    // ----------------------------------------------------------------
    wire [15:0]             act_rd_addr;
    wire                    act_rd_en;
    wire [DATA_W-1:0]       act_rd_data;
    // write path from cam_if
    wire [15:0]             act_wr_addr;
    wire                    act_wr_en;
    wire [DATA_W-1:0]       act_wr_data;

    // ----------------------------------------------------------------
    // Internal wiring: ctrl → mac_array
    // ----------------------------------------------------------------
    wire [ARRAY_C*DATA_W-1:0] mac_wgt_row;    // one row of weights
    wire [ARRAY_R*DATA_W-1:0] mac_act_col;    // one column of activations
    wire                      mac_load_wgt;   // load weight register
    wire                      mac_compute;    // trigger accumulation
    wire                      mac_clear;      // clear accumulators
    wire [ARRAY_R*ARRAY_C*ACCUM_W-1:0] mac_accum_out; // all accumulators
    wire                      mac_accum_valid;

    // ----------------------------------------------------------------
    // Internal wiring: mac_array → post_proc
    // ----------------------------------------------------------------
    wire [ARRAY_R*ARRAY_C*ACCUM_W-1:0] pp_accum_in;
    wire                               pp_in_valid;
    wire [ARRAY_R*ARRAY_C*DATA_W-1:0]  pp_out_data;
    wire                               pp_out_valid;

    // ----------------------------------------------------------------
    // Internal wiring: post_proc → spad
    // ----------------------------------------------------------------
    wire [15:0]             spad_wr_addr;
    wire                    spad_wr_en;
    wire [DATA_W-1:0]       spad_wr_data;
    wire [15:0]             spad_rd_addr;
    wire                    spad_rd_en;
    wire [DATA_W-1:0]       spad_rd_data;

    // ----------------------------------------------------------------
    // Submodule: APB Host Interface
    // ----------------------------------------------------------------
    coral_host_if u_host_if (
        .clk            (clk),
        .rst_n          (rst_n),
        .apb_paddr      (apb_paddr),
        .apb_psel       (apb_psel),
        .apb_penable    (apb_penable),
        .apb_pwrite     (apb_pwrite),
        .apb_pwdata     (apb_pwdata),
        .apb_prdata     (apb_prdata),
        .apb_pready     (apb_pready),
        .apb_pslverr    (apb_pslverr),
        .ctrl_start     (ctrl_start),
        .ctrl_halt      (ctrl_halt),
        .cfg_img_w      (cfg_img_w),
        .cfg_img_h      (cfg_img_h),
        .cfg_wgt_base   (cfg_wgt_base),
        .cfg_act_base   (cfg_act_base),
        .cfg_trig_thresh(cfg_trig_thresh),
        .cfg_relu_en    (cfg_relu_en),
        .cfg_shift      (cfg_shift),
        .stat_busy      (stat_busy),
        .stat_done      (stat_done),
        .stat_error     (stat_error),
        .stat_phase     (stat_phase),
        .irq            (irq)
    );

    // ----------------------------------------------------------------
    // Submodule: Camera Pixel Interface
    // ----------------------------------------------------------------
    coral_cam_if #(.DATA_W(DATA_W)) u_cam_if (
        .clk            (clk),
        .rst_n          (rst_n),
        .cam_vsync      (cam_vsync),
        .cam_href       (cam_href),
        .cam_pclk       (cam_pclk),
        .cam_data       (cam_data),
        .frame_rdy      (cam_frame_rdy),
        .pixel_out      (cam_pixel_out),
        .pixel_valid    (cam_pixel_valid),
        .pixel_addr     (cam_pixel_addr)
    );

    // ----------------------------------------------------------------
    // Submodule: Control / Sequencer FSM
    // ----------------------------------------------------------------
    coral_ctrl #(.DATA_W(DATA_W), .ARRAY_R(ARRAY_R), .ARRAY_C(ARRAY_C)) u_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .ctrl_start     (ctrl_start),
        .ctrl_halt      (ctrl_halt),
        .cfg_img_w      (cfg_img_w),
        .cfg_img_h      (cfg_img_h),
        .cfg_wgt_base   (cfg_wgt_base),
        .cfg_act_base   (cfg_act_base),
        .cam_frame_rdy  (cam_frame_rdy),
        .stat_busy      (stat_busy),
        .stat_done      (stat_done),
        .stat_error     (stat_error),
        .stat_phase     (stat_phase),
        .wgt_rd_addr    (wgt_rd_addr),
        .wgt_rd_en      (wgt_rd_en),
        .act_rd_addr    (act_rd_addr),
        .act_rd_en      (act_rd_en),
        .mac_wgt_row    (mac_wgt_row),
        .mac_act_col    (mac_act_col),
        .mac_load_wgt   (mac_load_wgt),
        .mac_compute    (mac_compute),
        .mac_clear      (mac_clear),
        .spad_rd_addr   (spad_rd_addr),
        .spad_rd_en     (spad_rd_en),
        .wgt_rd_data    (wgt_rd_data),
        .act_rd_data    (act_rd_data)
    );

    // ----------------------------------------------------------------
    // Submodule: MAC Array
    // ----------------------------------------------------------------
    coral_mac_array #(.DATA_W(DATA_W), .ACCUM_W(ACCUM_W), .ARRAY_R(ARRAY_R), .ARRAY_C(ARRAY_C)) u_mac (
        .clk            (clk),
        .rst_n          (rst_n),
        .wgt_row        (mac_wgt_row),
        .act_col        (mac_act_col),
        .load_wgt       (mac_load_wgt),
        .compute        (mac_compute),
        .clear          (mac_clear),
        .accum_out      (mac_accum_out),
        .accum_valid    (mac_accum_valid)
    );

    // ----------------------------------------------------------------
    // Submodule: Weight Buffer
    // ----------------------------------------------------------------
    coral_wgt_buf #(.DATA_W(DATA_W), .DEPTH(WGT_DEPTH)) u_wgt_buf (
        .clk            (clk),
        .rst_n          (rst_n),
        .rd_addr        (wgt_rd_addr),
        .rd_en          (wgt_rd_en),
        .rd_data        (wgt_rd_data),
        .wr_addr        (16'b0),    // host DMA write path (future)
        .wr_en          (1'b0),
        .wr_data        ({DATA_W{1'b0}})
    );

    // ----------------------------------------------------------------
    // Submodule: Activation Buffer (cam_if writes, ctrl reads)
    // ----------------------------------------------------------------
    coral_act_buf #(.DATA_W(DATA_W), .DEPTH(ACT_DEPTH)) u_act_buf (
        .clk            (clk),
        .rst_n          (rst_n),
        .wr_addr        (cam_pixel_addr),
        .wr_en          (cam_pixel_valid),
        .wr_data        (cam_pixel_out),
        .rd_addr        (act_rd_addr),
        .rd_en          (act_rd_en),
        .rd_data        (act_rd_data)
    );

    // ----------------------------------------------------------------
    // Submodule: Post-Processing
    // ----------------------------------------------------------------
    assign pp_accum_in  = mac_accum_out;
    assign pp_in_valid  = mac_accum_valid;

    coral_post_proc #(.DATA_W(DATA_W), .ACCUM_W(ACCUM_W), .ARRAY_R(ARRAY_R), .ARRAY_C(ARRAY_C)) u_post (
        .clk            (clk),
        .rst_n          (rst_n),
        .relu_en        (cfg_relu_en),
        .shift          (cfg_shift),
        .accum_in       (pp_accum_in),
        .in_valid       (pp_in_valid),
        .out_data       (pp_out_data),
        .out_valid      (pp_out_valid)
    );

    // ----------------------------------------------------------------
    // Submodule: Scratchpad (post_proc writes, trigger reads)
    // ----------------------------------------------------------------
    coral_spad #(.DATA_W(DATA_W), .DEPTH(SPAD_DEPTH)) u_spad (
        .clk            (clk),
        .rst_n          (rst_n),
        .wr_addr        (spad_wr_addr),
        .wr_en          (spad_wr_en),
        .wr_data        (spad_wr_data),
        .rd_addr        (spad_rd_addr),
        .rd_en          (spad_rd_en),
        .rd_data        (spad_rd_data)
    );

    // ----------------------------------------------------------------
    // Submodule: Trigger / Event Detection
    // ----------------------------------------------------------------
    coral_trigger #(.DATA_W(DATA_W)) u_trigger (
        .clk            (clk),
        .rst_n          (rst_n),
        .threshold      (cfg_trig_thresh),
        .score_in       (spad_rd_data),
        .score_valid    (spad_rd_en),
        .trig_out       (trig_out),
        .trig_score     (trig_score),
        .trig_valid     (trig_valid)
    );

    // Post-proc → spad write path
    // (simple sequential address counter — ctrl will orchestrate properly)
    assign spad_wr_data = pp_out_data[DATA_W-1:0]; // first output word (placeholder)
    assign spad_wr_en   = pp_out_valid;
    // spad_wr_addr driven by ctrl (tied to 0 here; ctrl owns this in full impl)
    assign spad_wr_addr = 16'b0; // TODO: connect ctrl address counter

endmodule
`default_nettype wire
