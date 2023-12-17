module divider(input logic         clk_i,
                input logic         reset_i,
                input logic [7:0]   dividend_i,
                input logic [7:0]   divisor_i,
                input logic         start_i,
                output logic        busy_o,
                output logic        finish_o,
                output logic [8:0] quotient_o);
    
    // State Registers
    enum logic [1:0] {INIT, BUSY, FINISH} state_p, state_n; 
    
    // Registers
    logic [7:0] subtrahend;
    logic [8:0] quotient;

    //  FSM
    // * Sequential circuit: use '<=', 'always_ff@(..)'
    // * Store current state, load next state at clock edge
    always_ff@(posedge clk_i or posedge reset_i) begin
        if(reset_i == 1) // Async reset
            state_p <= INIT;
        else
            state_p <= state_n;
    end

    // Data path + Data registers    
    always_ff@(posedge clk_i) begin //no posedge reset_i !
        case(state_p)
            
            INIT: begin
                subtrahend <= dividend_i;
                if(state_n == FINISH)
                    quotient <= 9'h1FF;
                else // INIT, BUSY
                    quotient <= 9'h0;
            end

            BUSY: begin
                subtrahend <= subtrahend - divisor_i;
                if(state_n == BUSY)
                    quotient <= quotient + 9'h1;
                // else FINISH quotient remains 
            end

            FINISH: begin
                quotient <= quotient;
                subtrahend <= subtrahend;
            end

        endcase
    end 

    // Combinational Next State Logic + Output Logic
    // * Combinational logic: use '=', 'always@(*) -> preferred or always_comb(..)'
    // * never edit state_p here, but in state register
    always@(*) begin

        // Default assign to avoid latches
        state_n = state_p; 
        quotient_o = quotient;

        case(state_p)

            INIT: begin
                busy_o=0;
                finish_o=0;
                if(start_i == 1) begin
                    if(divisor_i == 0)    
                        state_n = FINISH;
                    else 
                        state_n = BUSY;
                end 
                else
                    state_n = INIT;
            end

            BUSY: begin
                busy_o = 1;
                finish_o = 0;
                if(subtrahend >= divisor_i)
                    state_n = BUSY;
                else
                    state_n = FINISH;
            end

            FINISH: begin
                busy_o = 0;
                finish_o = 1;
                state_n = INIT;
            end

            default: begin // default assign error case
                busy_o = 0;
                finish_o = 0;
            end
            
        endcase
    end
endmodule