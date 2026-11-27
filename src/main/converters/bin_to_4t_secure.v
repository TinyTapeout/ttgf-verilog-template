module bin_to_4t_secure(
    input wire [7:0] binary_bits,
    inout [3:0] ternary_bits,
    output wire error,
    inout VDD_1V65 
);

//it's this big because i do NOT want to figure out how to even call the placement operations


assign error = trit0_decode_err | trit1_decode_err | trit2_decode_err 
| trit3_decode_err;


wire trit0_decode_err;
native_b2t_secure  trit0_decoder(
    .VDD_1V65(VDD_1V65),
    .binary_input(binary_bits[1:0]),
    .ternary_result(ternary_bits[0]),
    .decode_error(trit0_decode_err)

);
wire trit1_decode_err;
native_b2t_secure  trit1_decoder(
    .VDD_1V65(VDD_1V65),
    .binary_input(binary_bits[3:2]),
    .ternary_result(ternary_bits[1]),
    .decode_error(trit1_decode_err)
);
wire trit2_decode_err;
native_b2t_secure  trit2_decoder(
    .VDD_1V65(VDD_1V65),
    .binary_input(binary_bits[5:4]),
    .ternary_result(ternary_bits[2]),
    .decode_error(trit2_decode_err)
);
wire trit3_decode_err;
native_b2t_secure  trit3_decoder(
    .VDD_1V65(VDD_1V65),
    .binary_input(binary_bits[7:6]),
    .ternary_result(ternary_bits[3]),
    .decode_error(trit3_decode_err)
);
endmodule