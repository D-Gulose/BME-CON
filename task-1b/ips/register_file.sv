module register_file(
  input logic         clk_i,
  input logic         reset_i,
  // Read Reg Port 1
  input logic [4:0]   read_reg_1_i, // address to read
  output logic [31:0] read_data_1_o, // content of address
  // Read Reg Port 2
  input logic [4:0]   read_reg_2_i, // address to read
  output logic [31:0] read_data_2_o, // content of address
  // Write Reg Port
  input logic         write_i, // check if to write
  input logic [4:0]   write_reg_i, // writing address
  input logic [31:0]  write_data_i // content to write
);
  localparam NREGS = 32;
  integer i;

  // Register file and register read
  logic [31:0] X_p[NREGS-1:0];  // Registers x0 - x31

  always_ff @(posedge clk_i or posedge reset_i) begin
    if (reset_i) begin
      for (i = 0; i < NREGS; i = i + 1) begin
        X_p[i] <= 32'b0;
      end
    end else begin
      // x0 is always 0
      X_p[0]  <= 32'b0;

      for (i = 1; i < NREGS; i = i + 1) begin
        if(write_reg_i == i && write_i)
          X_p[i] <= write_data_i;
      end
    end
  end

  // Register read logic
  assign read_data_1_o = X_p[read_reg_1_i];
  assign read_data_2_o = X_p[read_reg_2_i];
endmodule