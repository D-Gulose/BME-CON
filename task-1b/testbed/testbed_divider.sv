
module testbench_divider ();
  logic        clk;
  logic        reset;
  logic        start;
  logic        busy;
  logic [7:0]  dividend;
  logic [7:0]  divisor;
  logic        finish;
  logic [8:0]  quotient;

  integer infile, outfile, read;
  logic [8:0] expected_quotient;

  divider div(
    .clk_i      (clk),
    .reset_i    (reset),
    .dividend_i (dividend),
    .divisor_i  (divisor),
    .start_i    (start),
    .busy_o     (busy),
    .finish_o   (finish),
    .quotient_o (quotient)
  );


  always begin
    #50 clk = ~clk;
  end

  initial
  begin
    $dumpfile("_sim/divider.vcd");
    $dumpvars(0, testbench_divider);
  end

  initial begin
    infile = $fopen("./testcases/testcases_divider.txt", "r");
    outfile = $fopen("_sim/output_divider.txt", "w");

    clk   = 0;
    start = 0;
    reset = 1;
    #100
    reset = 0;
    #100
    dividend = 0;
    divisor = 0;

    while (1 == 1) begin
      read = $fscanf(infile, "%d %d %d", dividend, divisor, expected_quotient);
      if (read == -1) begin
        $fdisplay(outfile, "All tests completed successfully!");
        $display("All tests completed successfully!");
        #200
        $finish();
      end

      #120
      start = 1;
      #120
      start = 0;
      wait(finish);
      $display("%d / %d = %d (expected %d)", dividend, divisor, quotient, expected_quotient);
      $fdisplay(outfile, "%d / %d = %d (expected %d)", dividend, divisor, quotient, expected_quotient);

      if (expected_quotient != quotient) begin
        $fdisplay(outfile, "Unexpected result on last computation!");
        $display("Unexpected result on last computation!");
        #200
        $finish();
      end
    end
  end

endmodule
