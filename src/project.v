/*
 * Copyright (c) 2024 Ripunjay Singh
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
`timescale 1ns / 1ps


module tt_um_posit_mac_stream (
    input  wire [7:0]  ui_in,    // A input
    output reg  [7:0]  uo_out,    // Result output
    input  wire [7:0]  uio_in,   // B input (used as input)
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire        ena,      // REQUIRED by Tiny Tapeout
    input  wire        clk,
    input  wire        rst_n  
);

    assign uio_oe = 0;
    assign uio_out = 0;

    
    reg [7:0] acc;
    wire [7:0] mac_out;

    posit_mac_8bit u_mac (
        .in_a(ui_in),
        .in_b(uio_in),
        .in_c(acc),
        .res(mac_out)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc    <= 8'b0;
            uo_out <= 8'b0;
        end else if (ena) begin
            acc    <= mac_out;   // accumulate only when enabled
            uo_out <= mac_out;
        end
        // else: hold state when ena == 0
    end

endmodule


// ============================================================================
// 1. HELPER MODULES
// ============================================================================
module lzc_16bit (
    input  wire [15:0] in_val,
    output reg  [3:0]  count
);
    always @(*) begin
        if (in_val[15]) count = 0;
        else if (in_val[14]) count = 1;
        else if (in_val[13]) count = 2;
        else if (in_val[12]) count = 3;
        else if (in_val[11]) count = 4;
        else if (in_val[10]) count = 5;
        else if (in_val[9])  count = 6;
        else if (in_val[8])  count = 7;
        else if (in_val[7])  count = 8;
        else if (in_val[6])  count = 9;
        else if (in_val[5])  count = 10;
        else if (in_val[4])  count = 11;
        else if (in_val[3])  count = 12;
        else if (in_val[2])  count = 13;
        else if (in_val[1])  count = 14;
        else if (in_val[0])  count = 15;
        else                 count = 16;
    end
endmodule

module lzoc_7bit (
    input  wire [6:0] in_val,
    input  wire       rc,
    output reg  [2:0] count
);
    wire [6:0] normalized_val;
    assign normalized_val = in_val ^ {7{rc}};

    always @(*) begin
        if (normalized_val[6] == 1'b1)      count = 3'd0;
        else if (normalized_val[5] == 1'b1) count = 3'd1;
        else if (normalized_val[4] == 1'b1) count = 3'd2;
        else if (normalized_val[3] == 1'b1) count = 3'd3;
        else if (normalized_val[2] == 1'b1) count = 3'd4;
        else if (normalized_val[1] == 1'b1) count = 3'd5;
        else if (normalized_val[0] == 1'b1) count = 3'd6;
        else                                count = 3'd7;
    end
endmodule

// ============================================================================
// 2. DECODER (FIXED: ROBUST 2's COMPLEMENT)
// ============================================================================
module posit_decoder_8bit (
    input  wire [7:0] in_posit,
    output wire       sign,
    output wire signed [5:0] reg_k,
    output wire [6:0] frac,
    output wire       z,
    output wire       inf
);
    wire [6:0] payload;
    wire       nzero;
    wire [6:0] twos_payload;
    wire       rc;
    wire [2:0] lzoc_count;
    wire [6:0] shifted_payload;
    wire [3:0] shift_amount;
    wire signed [5:0] raw_k;
    wire [6:0] raw_frac;

    assign sign = in_posit[7];
    assign payload = in_posit[6:0];
    assign nzero = |payload; 
    assign z = ~sign & ~nzero;
    assign inf = sign & ~nzero;

    // --- ROBUST FIX: Explicit 8-bit Math for 2's Comp ---
    wire [7:0] temp_twos;
    assign temp_twos = (sign) ? ({1'b0, ~payload} + 8'b1) : {1'b0, payload};
    assign twos_payload = temp_twos[6:0];

    assign rc = twos_payload[6];
    
    lzoc_7bit lzoc_inst (
        .in_val(twos_payload), .rc(rc), .count(lzoc_count)
    );

    assign shift_amount = {1'b0, lzoc_count} + 4'd1;
    assign shifted_payload = twos_payload << shift_amount;
    assign raw_k = (rc) ? ({3'b000, lzoc_count} - 6'sd1) : (-{3'b000, lzoc_count});
    assign raw_frac = {nzero, shifted_payload[6:1]};
    assign reg_k = (z | inf) ? 6'sd0 : raw_k;
    assign frac  = (z | inf) ? 7'b0  : raw_frac;
endmodule

// ============================================================================
// 3. ENCODER
// ============================================================================
module posit_encoder_8bit (
    input  wire       sign,
    input  wire signed [5:0] sf,
    input  wire [9:0] norm_f,
    input  wire       z,
    input  wire       inf,
    output wire [7:0] result
);
    localparam MAX_REG = 6;
    wire rc;
    wire signed [5:0] reg_f_signed;
    wire [3:0] reg_f;
    wire [3:0] offset;
    wire [11:0] in_shift;
    wire [23:0] padded_vec;
    wire [23:0] ans_shf;
    wire [6:0] payload_trunc;
    wire G, R, S_bit;
    wire round_up;
    wire [6:0] payload_rounded;
    wire [7:0] final_posit_pos;
    wire [7:0] final_posit_neg;

    assign rc = sf[5];
    assign reg_f_signed = (rc) ? (-sf) : sf;
    assign reg_f = (reg_f_signed > MAX_REG) ? 4'd6 : reg_f_signed[3:0];
    assign in_shift = (rc) ? {2'b01, norm_f} : {2'b10, norm_f};
    assign offset = (rc) ? (reg_f - 1'b1) : reg_f;
    assign padded_vec = { {12{~rc}}, in_shift }; 
    assign ans_shf = padded_vec >> offset;
    assign payload_trunc = ans_shf[11:5];
    assign G = ans_shf[4];
    assign R = ans_shf[3];
    assign S_bit = |ans_shf[2:0]; 
    
    wire LSB = payload_trunc[0];
    assign round_up = G & (LSB | R | S_bit);
    assign payload_rounded = payload_trunc + {6'b0, round_up};
    assign final_posit_pos = {1'b0, payload_rounded};
    assign final_posit_neg = -final_posit_pos;
    
    wire [7:0] result_computed = (sign) ? final_posit_neg : final_posit_pos;
    assign result = (inf) ? 8'b10000000 : (z) ? 8'b00000000 : result_computed;
endmodule

// ============================================================================
// 4. MULTIPLIER
// ============================================================================
module posit_multiplier_core_8bit (
    input  wire       sign_a,
    input  wire signed [5:0] sf_a,
    input  wire [6:0] frac_a,
    input  wire       z_a,
    input  wire       inf_a,
    input  wire       sign_b,
    input  wire signed [5:0] sf_b,
    input  wire [6:0] frac_b,
    input  wire       z_b,
    input  wire       inf_b,
    output wire       sign_out,
    output wire signed [5:0] sf_out,
    output wire [9:0] frac_out,
    output wire       z_out,
    output wire       inf_out
);
    assign sign_out = sign_a ^ sign_b;
    assign inf_out = inf_a | inf_b;
    assign z_out   = (z_a | z_b) & ~inf_out;

    wire [13:0] raw_mult;
    assign raw_mult = frac_a * frac_b;
    wire mult_overflow;
    assign mult_overflow = raw_mult[13]; 
    assign sf_out = sf_a + sf_b + mult_overflow;
    assign frac_out = (mult_overflow) ? raw_mult[12:3] : raw_mult[11:2];
endmodule

module posit_mult_8bit (
    input  wire [7:0] in_a,
    input  wire [7:0] in_b,
    output wire [7:0] res
);
    wire sign_a, z_a, inf_a;
    wire signed [5:0] sf_a;
    wire [6:0] frac_a;
    wire sign_b, z_b, inf_b;
    wire signed [5:0] sf_b;
    wire [6:0] frac_b;
    wire sign_core, z_core, inf_core;
    wire signed [5:0] sf_core;
    wire [9:0] frac_core;

    posit_decoder_8bit u_dec_a ( .in_posit(in_a), .sign(sign_a), .reg_k(sf_a), .frac(frac_a), .z(z_a), .inf(inf_a));
    posit_decoder_8bit u_dec_b ( .in_posit(in_b), .sign(sign_b), .reg_k(sf_b), .frac(frac_b), .z(z_b), .inf(inf_b));
    
    posit_multiplier_core_8bit u_core (
        .sign_a(sign_a), .sf_a(sf_a), .frac_a(frac_a), .z_a(z_a), .inf_a(inf_a),
        .sign_b(sign_b), .sf_b(sf_b), .frac_b(frac_b), .z_b(z_b), .inf_b(inf_b),
        .sign_out(sign_core), .sf_out(sf_core), .frac_out(frac_core), .z_out(z_core), .inf_out(inf_core)
    );
    
    posit_encoder_8bit u_enc ( .sign(sign_core), .sf(sf_core), .norm_f(frac_core), .z(z_core), .inf(inf_core), .result(res));
endmodule

// ============================================================================
// 5. ADDER (FIXED: ZERO BYPASS ADDED)
// ============================================================================
module posit_adder_8bit (
    input  wire [7:0] in_a,
    input  wire [7:0] in_b,
    output wire [7:0] res
);
    wire sign_a, z_a, inf_a;
    wire signed [5:0] sf_a;
    wire [6:0] frac_a;
    wire sign_b, z_b, inf_b;
    wire signed [5:0] sf_b;
    wire [6:0] frac_b;

    posit_decoder_8bit u_dec_a ( .in_posit(in_a), .sign(sign_a), .reg_k(sf_a), .frac(frac_a), .z(z_a), .inf(inf_a));
    posit_decoder_8bit u_dec_b ( .in_posit(in_b), .sign(sign_b), .reg_k(sf_b), .frac(frac_b), .z(z_b), .inf(inf_b));

    reg is_a_larger;
    always @(*) begin
        if (sf_a > sf_b) is_a_larger = 1'b1;
        else if (sf_b > sf_a) is_a_larger = 1'b0;
        else is_a_larger = (frac_a >= frac_b); 
    end

    wire sign_L, sign_S;
    wire signed [5:0] sf_L, sf_S;
    wire [6:0] frac_L, frac_S;

    assign sign_L = (is_a_larger) ? sign_a : sign_b;
    assign sf_L   = (is_a_larger) ? sf_a   : sf_b;
    assign frac_L = (is_a_larger) ? frac_a : frac_b;
    assign sign_S = (is_a_larger) ? sign_b : sign_a;
    assign sf_S   = (is_a_larger) ? sf_b   : sf_a;
    assign frac_S = (is_a_larger) ? frac_b : frac_a;

    wire [5:0] offset = sf_L - sf_S;
    wire [15:0] f_L_ext = {frac_L, 9'b0};
    wire [15:0] f_S_ext = {frac_S, 9'b0};
    wire [3:0] shift_amt = (offset > 15) ? 4'd15 : offset[3:0];
    wire [15:0] f_S_shifted = f_S_ext >> shift_amt;

    wire op_sub = sign_L ^ sign_S;
    reg [16:0] f_sum; 

    always @(*) begin
        if (op_sub) f_sum = {1'b0, f_L_ext} - {1'b0, f_S_shifted};
        else        f_sum = {1'b0, f_L_ext} + {1'b0, f_S_shifted};
    end

    wire ovf_f = f_sum[16];
    wire [3:0] lzc_count;
    lzc_16bit u_lzc ( .in_val(f_sum[15:0]), .count(lzc_count));

    reg signed [5:0] sf_final;
    reg [15:0] norm_f_accum;

    always @(*) begin
        if (ovf_f) begin
            sf_final = sf_L + 6'd1;
            norm_f_accum = f_sum[16:1]; 
        end else if (|f_sum == 0) begin
            sf_final = -6'd32; 
            norm_f_accum = 0;
        end else begin
            sf_final = sf_L - {2'b0, lzc_count};
            norm_f_accum = f_sum[15:0] << lzc_count;
        end
    end

    wire [9:0] frac_to_enc = norm_f_accum[14:5];
    wire res_inf = inf_a | inf_b;
    wire res_zero = (f_sum == 0) && !res_inf; 

    wire [7:0] calc_res;
    posit_encoder_8bit u_enc ( .sign(sign_L), .sf(sf_final), .norm_f(frac_to_enc), .z(res_zero), .inf(res_inf), .result(calc_res));

    // --- CRITICAL FIX: Zero Bypass ---
    // If one input is zero, return the other.
    // z_a means "A is Zero", so result is B.
    assign res = (z_a) ? in_b : (z_b) ? in_a : calc_res;

endmodule

// ============================================================================
// 6. TOP LEVEL
// ============================================================================
module posit_mac_8bit (
    input  wire [7:0] in_a,
    input  wire [7:0] in_b,
    input  wire [7:0] in_c,
    output wire [7:0] res
);
    wire [7:0] mult_result;
    posit_mult_8bit u_multiplier ( .in_a(in_a), .in_b(in_b), .res(mult_result));
    posit_adder_8bit u_adder ( .in_a(mult_result), .in_b(in_c), .res(res));
endmodule
