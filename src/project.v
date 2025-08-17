/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_ev_motor_control (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // Set uio_oe to control bidirectional pins (1=output, 0=input)
    // This must be a constant assignment for gate-level simulation
    assign uio_oe = 8'b11110000;  // uio[7:4] as outputs, uio[3:0] as inputs

    // Pin mapping - using wire assignments for better gate-level compatibility
    wire [2:0] operation_select;
    wire power_on_plc, power_on_hmi, mode_select;
    wire headlight_plc, headlight_hmi;
    wire horn_plc, horn_hmi, right_ind_plc, right_ind_hmi;
    wire [3:0] accelerator_brake_data;

    // Explicit wire assignments to avoid any potential X propagation
    assign operation_select = ui_in[2:0];
    assign power_on_plc = ui_in[3];
    assign power_on_hmi = ui_in[4];
    assign mode_select = ui_in[5];
    assign headlight_plc = ui_in[6];
    assign headlight_hmi = ui_in[7];
    
    assign horn_plc = uio_in[0];
    assign horn_hmi = uio_in[1];
    assign right_ind_plc = uio_in[2];
    assign right_ind_hmi = uio_in[3];
    assign accelerator_brake_data = uio_in[7:4];

    // Internal registers - All explicitly sized and initialized
    reg [3:0] accelerator_value;
    reg [3:0] brake_value;
    reg [7:0] motor_speed;
    reg [7:0] pwm_counter;
    reg [7:0] pwm_duty_cycle;
    reg system_enabled;
    reg temperature_fault;
    reg [6:0] internal_temperature;
    
    // Output control registers
    reg headlight_active;
    reg horn_active;
    reg indicator_active;
    reg motor_active;
    reg pwm_active;

    // PWM timing control
    reg [15:0] pwm_clk_div;

    // State machine for proper initialization
    reg [2:0] init_state;
    reg init_complete;

    // =============================================================================
    // INITIALIZATION STATE MACHINE - Critical for gate-level simulation
    // =============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            init_state <= 3'b000;
            init_complete <= 1'b0;
            accelerator_value <= 4'b1000;      // 8
            brake_value <= 4'b0011;            // 3  
        end else begin
            case (init_state)
                3'b000: begin
                    init_state <= 3'b001;
                end
                3'b001: begin
                    init_state <= 3'b010;
                end
                3'b010: begin
                    init_state <= 3'b011;
                end
                3'b011: begin
                    init_complete <= 1'b1;
                    init_state <= 3'b100;
                end
                default: begin
                    init_complete <= 1'b1;
                    // Update input values only when initialized and in motor control mode
                    if (operation_select == 3'b100 && ena) begin
                        accelerator_value <= uio_in[7:4];
                        brake_value <= uio_in[3:0];
                    end
                end
            endcase
        end
    end

    // =============================================================================
    // PWM CLOCK GENERATION
    // =============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_clk_div <= 16'b0;
        end else if (init_complete && ena) begin
            pwm_clk_div <= pwm_clk_div + 1'b1;
        end
    end

    // =============================================================================
    // TEMPERATURE MONITORING
    // =============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            internal_temperature <= 7'd25; // Room temperature
            temperature_fault <= 1'b0;
        end else if (init_complete && ena) begin
            // Temperature rises with motor activity
            if (system_enabled && (motor_speed > 8'd50)) begin
                if ((internal_temperature < 7'd100) && (pwm_clk_div[9:0] == 10'b0)) begin
                    internal_temperature <= internal_temperature + 1'b1;
                end
            end else if ((internal_temperature > 7'd25) && (pwm_clk_div[9:0] == 10'b0)) begin
                internal_temperature <= internal_temperature - 1'b1;
            end

            // Temperature fault detection with hysteresis
            if (internal_temperature >= 7'd85) begin
                temperature_fault <= 1'b1;
            end else if (internal_temperature <= 7'd75) begin
                temperature_fault <= 1'b0;
            end
        end
    end

    // =============================================================================
    // MAIN CONTROL LOGIC - Completely rewritten for gate-level compatibility
    // =============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Explicit initialization of all registers
            system_enabled <= 1'b0;
            motor_speed <= 8'b0;
            headlight_active <= 1'b0;
            horn_active <= 1'b0;
            indicator_active <= 1'b0;
            motor_active <= 1'b0;
            pwm_active <= 1'b0;
            pwm_duty_cycle <= 8'b0;
        end else if (init_complete && ena) begin
            
            // Power control - always evaluated
            system_enabled <= power_on_plc | power_on_hmi;
            
            // Only process when system is enabled
            if (power_on_plc | power_on_hmi) begin
                case (operation_select)
                    3'b000: begin // Power control
                        // Handled above
                    end
                    
                    3'b001: begin // Headlight control
                        headlight_active <= headlight_plc ^ headlight_hmi;
                    end
                    
                    3'b010: begin // Horn control
                        horn_active <= horn_plc ^ horn_hmi;
                    end
                    
                    3'b011: begin // Right indicator control
                        indicator_active <= right_ind_plc ^ right_ind_hmi;
                    end
                    
                    3'b100: begin // Motor speed calculation - FIXED
                        if (!temperature_fault) begin
                            // Explicit comparison and calculation
                            if (accelerator_value > brake_value) begin
                                // Manual multiplication by 16 using concatenation
                                motor_speed <= {(accelerator_value - brake_value), 4'b0000};
                            end else begin
                                motor_speed <= 8'b0;
                            end
                            motor_active <= 1'b1;
                        end else begin
                            // During overheating, reduce speed
                            motor_speed <= {1'b0, motor_speed[7:1]};
                            motor_active <= 1'b1;
                        end
                    end
                    
                    3'b101: begin // PWM generation
                        if (!temperature_fault) begin
                            pwm_duty_cycle <= motor_speed;
                            pwm_active <= |motor_speed; // OR reduction - 1 if any bit is 1
                        end else begin
                            pwm_duty_cycle <= {1'b0, motor_speed[7:1]};
                            pwm_active <= |motor_speed;
                        end
                    end
                    
                    3'b110: begin // Temperature monitoring
                        // Handled in separate always block
                    end
                    
                    3'b111: begin // System reset
                        motor_speed <= 8'b0;
                        pwm_duty_cycle <= 8'b0;
                        headlight_active <= 1'b0;
                        horn_active <= 1'b0;
                        indicator_active <= 1'b0;
                        motor_active <= 1'b0;
                        pwm_active <= 1'b0;
                    end
                    
                    default: begin
                        // Maintain current state
                    end
                endcase
            end else begin
                // Power off - reset all outputs
                headlight_active <= 1'b0;
                horn_active <= 1'b0;
                indicator_active <= 1'b0;
                motor_active <= 1'b0;
                pwm_active <= 1'b0;
                motor_speed <= 8'b0;
                pwm_duty_cycle <= 8'b0;
            end
        end
    end

    // =============================================================================
    // PWM COUNTER - Separate block for timing
    // =============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_counter <= 8'b0;
        end else if (init_complete && ena && system_enabled) begin
            pwm_counter <= pwm_counter + 1'b1;
        end else begin
            pwm_counter <= 8'b0;
        end
    end

    // =============================================================================
    // OUTPUT LOGIC - Explicit wire assignments
    // =============================================================================
    wire power_status;
    wire headlight_out;
    wire horn_out;
    wire right_indicator;
    wire motor_pwm;
    wire overheat_warning;
    wire [1:0] status_led;

    // Explicit assignments to prevent X propagation
    assign power_status = system_enabled;
    assign headlight_out = headlight_active & system_enabled;
    assign horn_out = horn_active & system_enabled;
    assign right_indicator = indicator_active & system_enabled;
    assign overheat_warning = temperature_fault;
    assign status_led = {temperature_fault, system_enabled};
    
    // PWM generation with explicit conditions
    assign motor_pwm = (system_enabled & pwm_active & (|pwm_duty_cycle)) ? 
                       (pwm_counter < pwm_duty_cycle) : 1'b0;

    // Final output assignments
    assign uo_out = {status_led, overheat_warning, motor_pwm, 
                     right_indicator, horn_out, headlight_out, power_status};
    
    assign uio_out = motor_speed;

    // Prevent optimization warnings
    wire _unused = &{mode_select, motor_active, 1'b0};

endmodule
