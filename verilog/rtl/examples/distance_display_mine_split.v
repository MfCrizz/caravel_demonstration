`default_nettype none

/*
* Clock is set to 100 mHz -> 10 nS clock period
* Every 100ms starts a new measurement. -> every 100_000_000 nS / 10 ns = 10_000_000 cycles. 
* At a clock frequnency of 100mHz every 10_000_000 cycles a new measurement starts.
*
* Sensor has 4 m max distance -> max 23200 micro seconds till echo should arrive. (distance in cm = microseconds passed from trigger to echo / 58)
* 23_200 uS -> 23_200_00 nS / 10 ns = 2_320_000 cycles 
*
* Sensor has 2 cm min distance -> min 116 micro seconds till echo should arrive. (distance in cm = microseconds passed from trigger to echo / 58)
* 116 uS -> 116_000 nS / 10 ns = 11_600 cycles 
*


* Clock is set to 1 mHz -> 1 uS clock period
* Every 100ms starts a new measurement. -> every 100_000 uS = 100_000 cycles. 
* At a clock frequnency of 1mHz every 100_000 cycles a new measurement starts.
*
* Sensor has 4 m max distance -> max 23200 micro seconds till echo should arrive. (distance in cm = microseconds passed from trigger to echo / 58)
* 23_200 uS = 23_200 cycles 
*
* Sensor has 2 cm min distance -> min 116 micro seconds till echo should arrive. (distance in cm = microseconds passed from trigger to echo / 58)
* 116 uS = 116 cycles 

* Every digit of the Display is lit for 500uS
* Wait 20 ms after every display cycle
*/


module distance_display(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif
    input wire          i_clk,
    input wire          i_reset,
    input wire          i_sensor_echo,
    //
    output wire         o_sensor_trigger,
    output wire [3:0]   o_display_anode,
    output wire [7:0]   o_display_cathode,
    output wire [12:0]  o_io_oeb 
    );

    assign o_io_oeb = {13{1'b0}};

    wire [8:0]   w_distance_binary;  // 9 bit - 512; sensor has 400cm max distance
    wire [11:0]  w_distance_bcd;     // 3 digit BCD
    wire         w_start_bcd_conversion;
    wire         w_finish;


    distance_measurement d_m (
        .i_clk(i_clk), 
        .i_reset(i_reset), 
        .i_sensor_echo(i_sensor_echo), 
        .o_sensor_trigger(o_sensor_trigger), 
        .o_distance(w_distance_binary),
        .o_rdy(w_start_bcd_conversion)
        );

    binary_to_bcd b_t_btc (
        .i_clk(i_clk), 
        .i_reset(i_reset), 
        .i_binary(w_distance_binary), 
        .i_start(w_start_bcd_conversion), 
        .o_bcd(w_distance_bcd),
        .o_finished(w_finish)
        );

    four_digit_seven_segment f_d_s_s (
        .i_clk(i_clk), 
        .i_reset(i_reset), 
        .i_distance_bcd(w_distance_bcd), 
        .o_display_anode(o_display_anode),
        .o_display_cathode(o_display_cathode)
        );    
          


endmodule

module distance_measurement(
	  input wire        i_clk,
    input wire        i_reset,
    input wire        i_sensor_echo,
    output reg        o_sensor_trigger,
    output reg [8:0]  o_distance,  // 9 bit - 512; sensor has 400cm max distance
    output reg        o_rdy  // measurment finished
);

    localparam CYCLES_TILL_100_MS = 100_000;
    localparam CYCLES_TILL_DISABLE_TRIGGER = 10; // 10 uS = 10_000 nS / 10 nS = 1_000 cycles
    localparam MAX_DISTANCE_CYCLES = 23_200;
    localparam MIN_DISTANCE_CYCLES = 116;

    reg [16:0]  r_measurment_counter_uS; //17 bit - 131_072
    reg         r_echo_received;

    always @(posedge i_clk) begin
        // if i_reset, set counter to 0
        if (i_reset) begin
            r_measurment_counter_uS <= {17{1'b0}};
            r_echo_received <= 1'b0;
            o_distance <= {9{1'b0}};
            o_sensor_trigger <= 1'b0;
            o_rdy <= 1'b0;

        // not i_reset
        end else begin

            // if up to 100ms -> i_reset to 0
            if (r_measurment_counter_uS == CYCLES_TILL_100_MS - 1) begin
                // i_reset
                r_measurment_counter_uS <= {17{1'b0}};
                r_echo_received <= 1'b0;
                o_distance <= {9{1'b0}};
                o_rdy <= 1'b0;
                // enabel trigger
                o_sensor_trigger <= 1'b1;

            // increment counter
            end else begin
                r_measurment_counter_uS <= r_measurment_counter_uS + 1'b1;
            end

            // disable trigger
            if (r_measurment_counter_uS == CYCLES_TILL_DISABLE_TRIGGER - 1) begin
            o_sensor_trigger <= 1'b0;
            end

            // echo is back, calculate distance in cm
            if (i_sensor_echo && !r_echo_received) begin
                r_echo_received <= 1'b1;

                // if out of range. set infalid
                if ((r_measurment_counter_uS > MAX_DISTANCE_CYCLES - 1) || (r_measurment_counter_uS < MIN_DISTANCE_CYCLES - 1)) begin
                    o_distance <= {9{1'b0}};
                    o_rdy <= 1'b1;
                    
                // valid
                end else 
                o_distance <= (r_measurment_counter_uS - 10) / 58;
                o_rdy <= 1'b1;

            end

        end

    end
 
endmodule


// Binary_to_BCD
// double dabble algorithm
module binary_to_bcd
  #(parameter INPUT_WIDTH = 9,
    parameter DECIMAL_DIGITS = 3)
  (
   input wire                           i_clk,
   input wire                           i_reset,
   input wire [INPUT_WIDTH-1:0]         i_binary,
   input wire                           i_start,
   //
   output wire [DECIMAL_DIGITS*4-1:0]   o_bcd,
   output wire                          o_finished
   );
   
  // states
  localparam s_IDLE              = 3'b000;
  localparam s_SHIFT             = 3'b001;
  localparam s_CHECK_SHIFT_INDEX = 3'b010;
  localparam s_ADD               = 3'b011;
  localparam s_CHECK_DIGIT_INDEX = 3'b100;
  localparam s_BCD_DONE          = 3'b101;

  // current state register
  reg [2:0] r_current_state;
   
  // vector that contains the output BCD
  reg [DECIMAL_DIGITS*4-1:0] r_bcd;
    
  // vector that contains the input binary value being shifted.
  reg [INPUT_WIDTH-1:0]      r_Binary;
      
  // keeps track of which Decimal Digit we are indexing
  reg [DECIMAL_DIGITS-1:0]   r_Digit_Index;
    
  // keeps track of which loop iteration we are on.
  // number of loops performed = INPUT_WIDTH
  reg [7:0]                  r_Loop_Count;
 
  wire [3:0]                 w_BCD_Digit;
  reg                        r_DV;                       
    
  always @(posedge i_clk)
    begin

      if (i_reset) begin
        r_current_state <= s_IDLE;
        r_bcd <= {DECIMAL_DIGITS*4-1{1'b0}};
        r_Binary <= {INPUT_WIDTH-1{1'b0}};
        r_Digit_Index <= {DECIMAL_DIGITS-1{1'b0}};
        r_Loop_Count <= {8{1'b0}};
        r_DV <= 0;

        // not i_reset
      end else begin
 
      case (r_current_state) 
  
        // Stay in this state until i_start comes along
        s_IDLE :
          begin
            r_DV <= 1'b0;
            
             
            if (i_start == 1'b1)
              begin
                r_Binary  <= i_binary;
                r_current_state <= s_SHIFT;
                r_bcd     <= 0;
              end
            else
              r_current_state <= s_IDLE;
          end
                 
  
        // Always shift the BCD Vector until we have shifted all bits through
        // Shift the most significant bit of r_Binary into r_bcd lowest bit.
        s_SHIFT :
          begin
            r_bcd     <= r_bcd << 1;
            r_bcd[0]  <= r_Binary[INPUT_WIDTH-1];
            r_Binary  <= r_Binary << 1;
            r_current_state <= s_CHECK_SHIFT_INDEX;
          end          
         
  
        // Check if we are done with shifting in r_Binary vector
        s_CHECK_SHIFT_INDEX :
          begin
            if (r_Loop_Count == INPUT_WIDTH-1)
              begin
                r_Loop_Count <= 0;
                r_current_state    <= s_BCD_DONE;
              end
            else
              begin
                r_Loop_Count <= r_Loop_Count + 1;
                r_current_state    <= s_ADD;
              end
          end
                 
  
        // Break down each BCD Digit individually.  Check them one-by-one to
        // see if they are greater than 4.  If they are, increment by 3.
        // Put the result back into r_bcd Vector.  
        s_ADD :
          begin
            if (w_BCD_Digit > 4)
              begin                                     
                r_bcd[(r_Digit_Index*4)+:4] <= w_BCD_Digit + 3;  
              end
             
            r_current_state <= s_CHECK_DIGIT_INDEX; 
          end       
         
         
        // Check if we are done incrementing all of the BCD Digits
        s_CHECK_DIGIT_INDEX :
          begin
            if (r_Digit_Index == DECIMAL_DIGITS-1)
              begin
                r_Digit_Index <= 0;
                r_current_state     <= s_SHIFT;
              end
            else
              begin
                r_Digit_Index <= r_Digit_Index + 1;
                r_current_state     <= s_ADD;
              end
          end
         
  
  
        s_BCD_DONE :
          begin
            r_DV      <= 1'b1;
            r_current_state <= s_IDLE;
          end
         
         
        default :
          r_current_state <= s_IDLE;
            
      endcase
    end // always @ (posedge i_clk)  
  end
   
  assign w_BCD_Digit = r_bcd[r_Digit_Index*4 +: 4];
       
  assign o_bcd = r_bcd;
  assign o_finished  = r_DV;
      
endmodule



/*
Select   3              2              1              0

      -- 1 --        -- 1 --        -- 1 --        -- 1 --
     |       |      |       |      |       |      |       |
     6       2      6       2      6       2      6       2
     |       |      |       |      |       |      |       |
      -- 7 --        -- 7 --        -- 7 --        -- 7 --
     |       |      |       |      |       |      |       |
     5       3      5       3      5       3      5       3
     |       |      |       |      |       |      |       |
      -- 4 --  (8)   -- 4 --  (8)   -- 4 --  (8)   -- 4 --  (8)
*/

module four_digit_seven_segment (
    input wire        i_clk,
    input wire        i_reset,
    input wire [11:0] i_distance_bcd, // 12 bit - 3 digits; sensor has 400cm max distance
    //
    output wire [3:0]   o_display_anode,
    output wire [7:0]   o_display_cathode
);

    localparam CYCLES_TILL_22000_US = 22_000;

    reg [14:0]  r_display_counter_uS;   // 15 bit - 32_768
    reg [11:0]  r_distance_bcd;         // to save distance for one cycle
    reg [3:0]   r_current_bcd_digit;    // holds bcd digit for current to be outputted value
    reg [3:0]   r_display_anode;              // holds output signals for display 
    reg [7:0]   r_display_cathode;              // holds output signals for display 

      // current state register
    reg [2:0] r_current_state;


    // states
    localparam s_IDLE              = 3'b000;
    localparam s_FIRST_DIGIT             = 3'b001;
    localparam s_SECOND_DIGIT = 3'b010;
    localparam s_THIRD_DIGIT               = 3'b011;
    localparam s_FOURTH_DIGIT = 3'b100;
    localparam s_BCD_DONE          = 3'b101;


    always @(posedge i_clk) begin
        // if i_reset, set counter to 0
        if (i_reset) begin
            r_current_state <= s_IDLE;
            r_display_counter_uS <= {15{1'b0}};
            r_distance_bcd <= {12{1'b0}};
            r_display_anode <= {4{1'b0}};
            r_display_cathode <= {8{1'b0}};
            r_current_bcd_digit <= {4{1'b0}};

        // not i_reset
        end else begin

            // if up to 22ms -> i_reset to 0
            if (r_display_counter_uS == CYCLES_TILL_22000_US - 1) begin
                // i_reset
                r_display_counter_uS <= {15{1'b0}};
                r_current_state <= s_FIRST_DIGIT;

            // increment counter
            end else begin
                r_display_counter_uS <= r_display_counter_uS + 1'b1;
            end


            if (r_display_counter_uS == 500 - 1) begin
                  r_current_state <= s_SECOND_DIGIT;

            end else if (r_display_counter_uS == 1000 - 1) begin
                r_current_state <= s_THIRD_DIGIT;

            end else if (r_display_counter_uS == 1500 - 1) begin
                r_current_state <= s_FOURTH_DIGIT;

            end else if (r_display_counter_uS == 2000 - 1) begin
                r_current_state <= s_IDLE;

            end





            case (r_current_state)

            s_IDLE:
              begin
                r_display_anode <= 4'b0000;

              end

            s_FIRST_DIGIT:
              begin

                // load distance
                r_distance_bcd <= i_distance_bcd;

                // extract first place out of distance
                r_current_bcd_digit <= r_distance_bcd[3:0];
                r_display_cathode <= seg8_decode(r_distance_bcd[3:0]);

                // enable first digit
                r_display_anode[0] <= 1;

              end
              
            s_SECOND_DIGIT:
              begin

                // disable first digit
                r_display_anode[0] <= 0;

                // extract second place out of distance
                r_current_bcd_digit <= r_distance_bcd[7:4];
                r_display_cathode <= seg8_decode(r_distance_bcd[7:4]);

                // enable second digit
                r_display_anode[1] <= 1;

              end
              
            s_THIRD_DIGIT:
              begin
                // disable second digit
                r_display_anode[1] <= 0;

                // extract third place out of distance
                r_current_bcd_digit <= r_distance_bcd[11:8];
                r_display_cathode <= seg8_decode(r_distance_bcd[11:8]);

                // enable third digit
                r_display_anode[2] <= 1;

              end

              
            s_FOURTH_DIGIT:
              begin
                 // disable third digit
                r_display_anode[2] <= 0;

                // set fourth digit to zero
                r_current_bcd_digit <= {4{1'b0}};
                //r_display[7:0] <= seg8_decode({4{1'b0}});
                r_display_cathode <= seg8_decode({4{1'b0}});

                // enable fourth digit
                r_display_anode[3] <= 1;

              end

            default :
              r_current_state <= s_IDLE;

            endcase

        end

    end


  // instantiate segment display
  //seg8 seg8(.i_bcd_digit(r_current_bcd_digit), .o_segments(o_display[7:0]));

  assign o_display_anode = r_display_anode;
  assign o_display_cathode = r_display_cathode;



function [7:0] seg8_decode(input [3:0] i_bcd_digit);
  begin
            case(i_bcd_digit)
            //        segments  87654321
            0:  seg8_decode = 8'b00111111;
            1:  seg8_decode = 8'b00000110;
            2:  seg8_decode = 8'b01011011;
            3:  seg8_decode = 8'b01001111;
            4:  seg8_decode = 8'b01100110;
            5:  seg8_decode = 8'b01101101;
            6:  seg8_decode = 8'b01111100;
            7:  seg8_decode = 8'b00000111;
            8:  seg8_decode = 8'b01111111;
            9:  seg8_decode = 8'b01100111;
           16:  seg8_decode = 8'b01000000;
            default:    
                seg8_decode = 8'b00000000;
        endcase
  end
  
endfunction

endmodule




/*
      -- 1 --
     |       |
     6       2
     |       |
      -- 7 --
     |       |
     5       3
     |       |
      -- 4 --  (8)
*/

module seg8 (
    input wire [3:0] i_bcd_digit,
    output reg [7:0] o_segments
);

	always @(*) begin
        case(i_bcd_digit)
            //        segments  87654321
            0:  o_segments = 8'b00111111;
            1:  o_segments = 8'b00000110;
            2:  o_segments = 8'b01011011;
            3:  o_segments = 8'b01001111;
            4:  o_segments = 8'b01100110;
            5:  o_segments = 8'b01101101;
            6:  o_segments = 8'b01111100;
            7:  o_segments = 8'b00000111;
            8:  o_segments = 8'b01111111;
            9:  o_segments = 8'b01100111;
           16:  o_segments = 8'b01000000;
            default:    
                o_segments = 8'b00000000;
        endcase
    end

endmodule



`default_nettype wire