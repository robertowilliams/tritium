// ============================================================
// coral_host_if.v
// APB Slave Host Interface — Register Map & Interrupt
//
// Register Map (word-addressed, 32-bit):
//   0x000  CTRL        [0]=start, [1]=halt, [2]=sw_reset
//   0x004  STATUS      [0]=busy, [1]=done, [2]=error, [5:4]=phase[1:0]
//   0x008  IMG_CFG     [7:0]=img_w, [15:8]=img_h
//   0x00C  WGT_BASE    [15:0]=weight buffer base address
//   0x010  ACT_BASE    [15:0]=activation buffer base address
//   0x014  TRIG_CFG    [7:0]=threshold, [8]=relu_en, [12:9]=shift
//   0x018  IRQ_CFG     [0]=irq_en_done, [1]=irq_en_trig, [8]=irq_clr
//   0x01C  SCRATCH     Read/write scratch register (test use)
// ============================================================

`default_nettype none
`timescale 1ns / 1ps

module coral_host_if (
    input  wire         clk,
    input  wire         rst_n,

    // APB slave
    input  wire [11:0]  apb_paddr,
    input  wire         apb_psel,
    input  wire         apb_penable,
    input  wire         apb_pwrite,
    input  wire [31:0]  apb_pwdata,
    output reg  [31:0]  apb_prdata,
    output wire         apb_pready,
    output wire         apb_pslverr,

    // Configuration outputs → ctrl
    output reg          ctrl_start,
    output reg          ctrl_halt,
    output reg  [7:0]   cfg_img_w,
    output reg  [7:0]   cfg_img_h,
    output reg  [15:0]  cfg_wgt_base,
    output reg  [15:0]  cfg_act_base,
    output reg  [7:0]   cfg_trig_thresh,
    output reg          cfg_relu_en,
    output reg  [3:0]   cfg_shift,

    // Status inputs ← ctrl
    input  wire         stat_busy,
    input  wire         stat_done,
    input  wire         stat_error,
    input  wire [3:0]   stat_phase,

    // Interrupt output
    output reg          irq
);

    // APB is always ready (single-cycle response, no wait states)
    assign apb_pready  = 1'b1;
    assign apb_pslverr = 1'b0;

    // ----------------------------------------------------------------
    // Internal registers
    // ----------------------------------------------------------------
    reg [31:0]  reg_scratch;
    reg         irq_en_done;
    reg         irq_en_trig;
    reg         irq_latch_done;     // latched done event

    // APB write enable: active during setup+enable with pwrite
    wire apb_wr = apb_psel & apb_penable & apb_pwrite;
    wire apb_rd = apb_psel & ~apb_pwrite;

    // ----------------------------------------------------------------
    // Register Write
    // ----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctrl_start      <= 1'b0;
            ctrl_halt       <= 1'b0;
            cfg_img_w       <= 8'd64;
            cfg_img_h       <= 8'd64;
            cfg_wgt_base    <= 16'h0000;
            cfg_act_base    <= 16'h0200;
            cfg_trig_thresh <= 8'd128;
            cfg_relu_en     <= 1'b1;
            cfg_shift       <= 4'd4;
            irq_en_done     <= 1'b0;
            irq_en_trig     <= 1'b0;
            reg_scratch     <= 32'hDEAD_BEEF;
        end else begin
            // start is a self-clearing pulse
            ctrl_start <= 1'b0;

            if (apb_wr) begin
                case (apb_paddr[7:0])
                    8'h00: begin
                        ctrl_start  <= apb_pwdata[0];
                        ctrl_halt   <= apb_pwdata[1];
                        // bit[2] = sw_reset handled separately if needed
                    end
                    8'h08: begin
                        cfg_img_w   <= apb_pwdata[7:0];
                        cfg_img_h   <= apb_pwdata[15:8];
                    end
                    8'h0C:  cfg_wgt_base    <= apb_pwdata[15:0];
                    8'h10:  cfg_act_base    <= apb_pwdata[15:0];
                    8'h14: begin
                        cfg_trig_thresh <= apb_pwdata[7:0];
                        cfg_relu_en     <= apb_pwdata[8];
                        cfg_shift       <= apb_pwdata[12:9];
                    end
                    8'h18: begin
                        irq_en_done <= apb_pwdata[0];
                        irq_en_trig <= apb_pwdata[1];
                        if (apb_pwdata[8]) irq_latch_done <= 1'b0; // IRQ clear
                    end
                    8'h1C:  reg_scratch     <= apb_pwdata;
                    default: ;
                endcase
            end

            // Latch done event for IRQ
            if (stat_done && !stat_busy)
                irq_latch_done <= 1'b1;
        end
    end

    // ----------------------------------------------------------------
    // Register Read
    // ----------------------------------------------------------------
    always @(*) begin
        apb_prdata = 32'h0;
        if (apb_rd) begin
            case (apb_paddr[7:0])
                8'h00:  apb_prdata = {29'b0, ctrl_halt, ctrl_start, 1'b0};
                8'h04:  apb_prdata = {24'b0, stat_phase, stat_error, stat_done, stat_busy};
                8'h08:  apb_prdata = {16'b0, cfg_img_h, cfg_img_w};
                8'h0C:  apb_prdata = {16'b0, cfg_wgt_base};
                8'h10:  apb_prdata = {16'b0, cfg_act_base};
                8'h14:  apb_prdata = {19'b0, cfg_shift, cfg_relu_en, cfg_trig_thresh};
                8'h18:  apb_prdata = {23'b0, irq_latch_done, 6'b0, irq_en_trig, irq_en_done};
                8'h1C:  apb_prdata = reg_scratch;
                default: apb_prdata = 32'hDEAD_BEEF;
            endcase
        end
    end

    // ----------------------------------------------------------------
    // Interrupt Generation
    // ----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            irq <= 1'b0;
        else
            irq <= (irq_en_done & irq_latch_done);
    end

endmodule
`default_nettype wire
