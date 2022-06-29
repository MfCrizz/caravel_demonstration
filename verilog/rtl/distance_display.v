`default_nettype none

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


    four_digit_seven_segment_controler f_d_s_s_c (
      .i_clk(i_clk),
      .i_bcd_value(w_distance_bcd),
      .o_anode(o_display_anode),
      .o_cathode(o_display_cathode)
      );
     


endmodule



module distance_measurement(
	  input wire        i_clk,
    input wire        i_reset,
    input wire        i_sensor_echo,
    output reg        o_sensor_trigger = 0,
    output reg [8:0]  o_distance = 0,  // 9 bit - 512; sensor has 400cm max distance
    output reg        o_rdy = 0  // measurment finished
);


    localparam MAX_DISTANCE_CYCLES = 232_0; // 23_200 uS


    integer  r_measurment_counter_10_uS = 0; 
    reg      r_echo_received = 0;

    // current state register
    reg [1:0] r_current_state;

      // states
    localparam s_IDLE             = 2'b00;
    localparam s_TRIGGER          = 2'b01;
    localparam s_COUNT_ECHO       = 2'b10;
    localparam s_WAIT             = 2'b11;


    // every 100mS -> 10 Hz
    wire w_div_clk_100_mS;

    clock_divider #(.DIV_VAL(49_999)) c_d(
      .i_clk(i_clk),
      .o_div_clk(w_div_clk_100_mS)
    );

    always@(posedge i_clk or posedge i_reset) begin

      if(i_reset) begin
        r_current_state <= s_IDLE;
        r_measurment_counter_10_uS <= 0;
        o_rdy <= 1'b0;
        o_distance <= 0;
        o_sensor_trigger <= 1'b0;

      end else begin
        case (r_current_state)

          s_IDLE:
            begin
              o_rdy <= 1'b0;
              // start new measurement
              if(w_div_clk_100_mS == 1'b1) begin
                r_measurment_counter_10_uS <= 0;
                o_sensor_trigger <= 1'b1;
                r_current_state <= s_TRIGGER;
              end else begin
                r_current_state <= s_IDLE;
              end
            end

          s_TRIGGER:
          begin
            o_sensor_trigger <= 1'b0;
            r_current_state <= s_COUNT_ECHO;
          end

          s_COUNT_ECHO:
          begin
            if(!s_COUNT_ECHO || r_measurment_counter_10_uS > MAX_DISTANCE_CYCLES - 1) begin
              r_measurment_counter_10_uS <= r_measurment_counter_10_uS + 1;
              r_current_state <= s_COUNT_ECHO;
            end else begin
              r_current_state <= s_WAIT;
            end
          end

          s_WAIT:
          begin
            o_distance <= ((r_measurment_counter_10_uS / 10)) / 58;
            o_rdy <= 1'b1;
          end
        endcase
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


module four_digit_seven_segment_controler(
  input wire i_clk,
  input wire [11:0] i_bcd_value,
  output wire [3:0] o_anode,
  output wire [7:0] o_cathode
);
wire w_refresh_clk;
wire [1:0] w_refresh_counter;
wire [3:0] w_single_digit;

clock_divider refresh_clk_generator(
  .i_clk(i_clk),
  .o_div_clk(w_refresh_clk) // 10 kHz
);

refresh_counter refresh_counter_wrapper(
  .i_refresh_clk(w_refresh_clk),
  .o_refresh_counter(w_refresh_counter)
);

anode_control anode_control_wrapper(
  .i_refresh_counter(w_refresh_counter),
  .o_anode(o_anode)
);

bcd_control bcd_control_wrapper(
  .i_digit_1(i_bcd_value[3:0]),
  .i_digit_2(i_bcd_value[7:4]),
  .i_digit_3(i_bcd_value[11:8]),
  .i_digit_4(4'b0000),
  .i_refresh_counter(w_refresh_counter),
  .o_single_digit(w_single_digit)
);

bcd_to_cathodes bcd_to_cathodes_wrapper(
  .i_digit(w_single_digit),
  .o_cathode(o_cathode)
);
endmodule


module refresh_counter(
  input wire i_refresh_clk,
  output reg [1:0] o_refresh_counter = 0
);

always@(posedge i_refresh_clk)
begin
  o_refresh_counter <= o_refresh_counter + 1;

end
endmodule


module anode_control(
  input wire [1:0] i_refresh_counter,
  output reg [3:0] o_anode = 0
  );

always@(i_refresh_counter)
begin
  case (i_refresh_counter)
  2'b00: o_anode = 4'b0001; // digit 1 ON (right digit)
  2'b01: o_anode = 4'b0010; // digit 2 ON
  2'b10: o_anode = 4'b0100; // digit 3 ON
  2'b11: o_anode = 4'b1000; // digit 4 ON (left digit)
  endcase

end
endmodule


module bcd_control(
  input wire [3:0] i_digit_1, // right digit // ones
  input wire [3:0] i_digit_2, // tens
  input wire [3:0] i_digit_3, // hundreds
  input wire [3:0] i_digit_4, // left digit // thousands
  input wire [1:0] i_refresh_counter, // left digit // thousands
  //
  output reg [3:0] o_single_digit = 0 // choose wich input digit is displayed
);

always@(i_refresh_counter)
begin
  case (i_refresh_counter)
  2'd0: o_single_digit = i_digit_1; // digit 1 value (right digit)
  2'd1: o_single_digit = i_digit_2; // digit 2 value
  2'd2: o_single_digit = i_digit_3; // digit 3 value
  2'd3: o_single_digit = i_digit_4; // digit 4 value (left digit)
  endcase

end
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

    cathodes needs to be driven low to be ON
*/

module bcd_to_cathodes(
  input [3:0] i_digit,
  output reg [7:0] o_cathode
);

always@(i_digit)
begin

  case(i_digit)
    //        segments  87654321
  4'd0: o_cathode <= 8'b11000000;
  4'd1: o_cathode <= 8'b11111001;
  4'd2: o_cathode <= 8'b10100100;
  4'd3: o_cathode <= 8'b10110000;
  4'd4: o_cathode <= 8'b10011001;
  4'd5: o_cathode <= 8'b10010010;
  4'd6: o_cathode <= 8'b10000010;
  4'd7: o_cathode <= 8'b11111000;
  4'd8: o_cathode <= 8'b10000000;
  4'd9: o_cathode <= 8'b10010000;
  default:
    o_cathode <= 8'b11000000;
 endcase

end
endmodule


module clock_divider
  // DIV_VAL = ( i_clk / (2 * desired frequency) ) - 1
  #(parameter DIV_VAL = 49) // 10kHz @ 1MHz
  (
    input wire i_clk, // 1MHz
    output reg o_div_clk = 0
  );


// counter
integer counter_value = 0;

always@(posedge i_clk)
  begin
    // reset
    if (counter_value == DIV_VAL) begin
      counter_value <= 0;
    // increase
    end else begin
      counter_value <= counter_value + 1;
    end
  end

// divide clock
always@(posedge i_clk)
  begin
    // flip clock
    if (counter_value == DIV_VAL) begin
      o_div_clk <= ~o_div_clk;
    // store value
    end else begin
      o_div_clk <= o_div_clk;
    end
  end
endmodule


`default_nettype wire