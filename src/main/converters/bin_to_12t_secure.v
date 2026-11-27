module bin_to_12t_secure(
    input wire [23:0] binary_bits,
    inout [11:0] ternary_bits,
    output wire error,
    inout VDD_1V65 
);

//it's this big because i do NOT want to figure out how to even call the placement operations


assign error = trit0_decode_err | trit1_decode_err | trit2_decode_err 
| trit3_decode_err | trit4_decode_err | trit5_decode_err
| trit6_decode_err | trit7_decode_err | trit8_decode_err
| trit9_decode_err | trit10_decode_err| trit11_decode_err;


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
wire trit4_decode_err;
native_b2t_secure  trit4_decoder(
    .VDD_1V65(VDD_1V65),
    .binary_input(binary_bits[9:8]),
    .ternary_result(ternary_bits[4]),
    .decode_error(trit4_decode_err)
);
wire trit5_decode_err;
native_b2t_secure  trit3_decoder(
    .VDD_1V65(VDD_1V65),
    .binary_input(binary_bits[11:10]),
    .ternary_result(ternary_bits[5]),
    .decode_error(trit5_decode_err)
);
wire trit6_decode_err;
native_b2t_secure  trit6_decoder(
    .VDD_1V65(VDD_1V65),
    .binary_input(binary_bits[13:12]),
    .ternary_result(ternary_bits[6]),
    .decode_error(trit6_decode_err)
);
wire trit7_decode_err;
native_b2t_secure  trit7_decoder(
    .VDD_1V65(VDD_1V65),
    .binary_input(binary_bits[15:14]),
    .ternary_result(ternary_bits[7]),
    .decode_error(trit7_decode_err)
);
wire trit8_decode_err;
native_b2t_secure  trit8_decoder(
    .VDD_1V65(VDD_1V65),
    .binary_input(binary_bits[17:16]),
    .ternary_result(ternary_bits[8]),
    .decode_error(trit8_decode_err)
);
wire trit9_decode_err;
native_b2t_secure  trit9_decoder(
    .VDD_1V65(VDD_1V65),
    .binary_input(binary_bits[19:18]),
    .ternary_result(ternary_bits[9]),
    .decode_error(trit9_decode_err)
);
wire trit10_decode_err;
native_b2t_secure  trit10_decoder(
    .VDD_1V65(VDD_1V65),
    .binary_input(binary_bits[21:20]),
    .ternary_result(ternary_bits[10]),
    .decode_error(trit10_decode_err)
);
wire trit11_decode_err;
native_b2t_secure  trit11_decoder(
    .VDD_1V65(VDD_1V65),
    .binary_input(binary_bits[23:22]),
    .ternary_result(ternary_bits[11]),
    .decode_error(trit11_decode_err)
);
endmodule