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

    // Pin mapping for ui_in[7:0] (Dedicated inputs)
    wire [2:0] operation_select = ui_in[2:0];  // 3-bit operation selector
    wire power_on_plc = ui_in[3];              // Power control from PLC
    wire power_on_hmi = ui_in[4];              // Power control from HMI
    wire mode_select = ui_in[5];               // 0: PLC mode, 1: HMI mode
    wire headlight_plc = ui_in[6];             // Headlight control from PLC
    wire headlight_hmi = ui_in[7];             // Headlight control from HMI

    // Pin mapping for uio_in[7:0] (Bidirectional inputs)
    wire horn_plc = uio_in[0];                 // Horn control from PLC
    wire horn_hmi = uio_in[1];                 // Horn control from HMI
    wire right_ind_plc = uio_in[2];            // Right indicator from PLC
    wire right_ind_hmi = uio_in[3];            // Right indicator from HMI
    wire [3:0] accelerator_brake_data = uio_in[7:4]; // 4-bit data for accel/brake

    // Set uio_oe to control bidirectional pins (1=output, 0=input)
    assign uio_oe = 8'b11110000;  // uio[7:4] as outputs, uio[3:0] as inputs

    // Internal registers and wires
    reg [3:0] accelerator_value;
    reg [3:0] brake_value;
    reg [7:0] motor_speed;
    reg [7:0] pwm_counter;
    reg [7:0] pwm_duty_cycle;
    reg system_enabled;
    reg temperature_fault;
    reg [6:0] internal_temperature;
    
    // Output control registers for ALL cases
    reg headlight_active;
    reg horn_active;
    reg indicator_active;
    reg motor_active;
    reg pwm_active;

    // PWM and timing control
    reg [15:0] pwm_clk_div;
    wire pwm_clk;

    // FIXED: Simplified data input control
    reg [7:0] data_counter;

    // =============================================================================
    // DATA INPUT HANDLING - COMPLETELY SIMPLIFIED
    // =============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accelerator_value <= 4'd8;      // Default accelerator
            brake_value <= 4'd3;            // Default brake  
            data_counter <= 8'b0;
        end else begin
            data_counter <= data_counter + 1;
            
            // FIXED: Direct assignment - take upper 4 bits as accelerator for now
            // In your test, you're setting both accel and brake simultaneously
            // So let's extract them properly from the 8-bit input
            accelerator_value <= accelerator_brake_data; // This will be the upper 4 bits
            
            // For brake, we'll use the lower 4 bits of uio_in when operation is motor control
            if (operation_select == 3'b100) begin
                brake_value <= uio_in[3:0]; // Lower 4 bits contain brake value
            end
        end
    end

    // Generate PWM clock
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_clk_div <= 16'b0;
        end else begin
            pwm_clk_div <= pwm_clk_div + 1;
        end
    end
    assign pwm_clk = pwm_clk_div[4]; // Fast PWM for visible results

    // =============================================================================
    // TEMPERATURE MONITORING - Always Active
    // =============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            internal_temperature <= 7'd25; // Room temperature
            temperature_fault <= 1'b0;
        end else begin
            // Temperature rises with motor activity
            if (system_enabled && motor_speed > 8'd50) begin
                if (internal_temperature < 7'd100 && pwm_clk_div[9:0] == 10'h000)
                    internal_temperature <= internal_temperature + 1;
            end else if (internal_temperature > 7'd25 && pwm_clk_div[9:0] == 10'h000) begin
                internal_temperature <= internal_temperature - 1;
            end

            // Temperature fault detection
            if (internal_temperature >= 7'd85) begin
                temperature_fault <= 1'b1;
            end else if (internal_temperature <= 7'd75) begin
                temperature_fault <= 1'b0;
            end
        end
    end

    // =============================================================================
    // MAIN CONTROL LOGIC - FIXED MOTOR SPEED CALCULATION
    // =============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize ALL registers
            system_enabled <= 1'b0;
            motor_speed <= 8'b0;
            headlight_active <= 1'b0;
            horn_active <= 1'b0;
            indicator_active <= 1'b0;
            motor_active <= 1'b0;
            pwm_active <= 1'b0;
            pwm_duty_cycle <= 8'b0;
        end else if (ena) begin
            
            // Power control is always evaluated regardless of operation_select
            system_enabled <= (power_on_plc | power_on_hmi);
            
            // Reset all outputs when power is off
            if (!(power_on_plc | power_on_hmi)) begin
                headlight_active <= 1'b0;
                horn_active <= 1'b0;
                indicator_active <= 1'b0;
                motor_active <= 1'b0;
                pwm_active <= 1'b0;
                motor_speed <= 8'b0;
                pwm_duty_cycle <= 8'b0;
            end else begin
                // Only execute operations when system is powered
                case (operation_select)
                    // =================================================================
                    // CASE 0: POWER CONTROL (operation_select = 3'b000)
                    // =================================================================
                    3'b000: begin
                        // Power control is handled above - just maintain state
                    end
                    
                    // =================================================================
                    // CASE 1: HEADLIGHT CONTROL (operation_select = 3'b001)
                    // =================================================================
                    3'b001: begin
                        // XOR logic: only one source should control
                        headlight_active <= (headlight_plc ^ headlight_hmi);
                    end
                    
                    // =================================================================
                    // CASE 2: HORN CONTROL (operation_select = 3'b010)
                    // =================================================================
                    3'b010: begin
                        // XOR logic for horn control
                        horn_active <= (horn_plc ^ horn_hmi);
                    end
                    
                    // =================================================================
                    // CASE 3: RIGHT INDICATOR CONTROL (operation_select = 3'b011)
                    // =================================================================
                    3'b011: begin
                        // XOR logic for indicator control
                        indicator_active <= (right_ind_plc ^ right_ind_hmi);
                    end
                    
                    // =================================================================
                    // CASE 4: MOTOR SPEED CALCULATION - COMPLETELY FIXED
                    // =================================================================
                    3'b100: begin
                        if (!temperature_fault) begin
                            // FIXED: Proper motor speed calculation
                            if (accelerator_value > brake_value) begin
                                // Scale by 16 for good range (4-bit difference -> 8-bit speed)
                                motor_speed <= (accelerator_value - brake_value) << 4;
                            end else begin
                                motor_speed <= 8'b0;
                            end
                            motor_active <= 1'b1;
                        end else begin
                            // Reduce speed by 50% during overheating
                            motor_speed <= motor_speed >> 1;
                            motor_active <= 1'b1;
                        end
                    end
                    
                    // =================================================================
                    // CASE 5: PWM GENERATION (operation_select = 3'b101)
                    // =================================================================
                    3'b101: begin
                        if (!temperature_fault) begin
                            // FIXED: Ensure PWM duty cycle is properly set
                            pwm_duty_cycle <= motor_speed;
                            pwm_active <= (motor_speed > 0) ? 1'b1 : 1'b0;
                        end else begin
                            // Reduced PWM during fault
                            pwm_duty_cycle <= motor_speed >> 1;
                            pwm_active <= (motor_speed > 0) ? 1'b1 : 1'b0;
                        end
                    end
                    
                    // =================================================================
                    // CASE 6: TEMPERATURE MONITORING (operation_select = 3'b110)
                    // =================================================================
                    3'b110: begin
                        // Temperature monitoring is handled in separate always block
                        // This case maintains current state and allows temperature readout
                    end
                    
                    // =================================================================
                    // CASE 7: SYSTEM STATUS/RESET (operation_select = 3'b111)
                    // =================================================================
                    3'b111: begin
                        // Reset all active states
                        motor_speed <= 8'b0;
                        pwm_duty_cycle <= 8'b0;
                        headlight_active <= 1'b0;
                        horn_active <= 1'b0;
                        indicator_active <= 1'b0;
                        motor_active <= 1'b0;
                        pwm_active <= 1'b0;
                    end
                    
                    // Default case: maintain current state
                    default: begin
                        // Maintain current state for any undefined operations
                    end
                endcase
            end
        end
    end

    // =============================================================================
    // PWM GENERATION HARDWARE - COMPLETELY FIXED
    // =============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_counter <= 8'b0;
        end else if (system_enabled) begin
            // Increment PWM counter every clock cycle for high frequency PWM
            pwm_counter <= pwm_counter + 1;
        end else begin
            pwm_counter <= 8'b0;
        end
    end

    // =============================================================================
    // OUTPUT ASSIGNMENTS - ALL OUTPUTS PROPERLY DEFINED
    // =============================================================================
    wire power_status = system_enabled;
    wire headlight_out = headlight_active & system_enabled;
    wire horn_out = horn_active & system_enabled;
    wire right_indicator = indicator_active & system_enabled;
    
    // FIXED: PWM output with better duty cycle control
    wire motor_pwm = (system_enabled && pwm_active && pwm_duty_cycle > 0) ? 
                     (pwm_counter < pwm_duty_cycle) : 1'b0;
                     
    wire overheat_warning = temperature_fault;
    wire [1:0] status_led = {temperature_fault, system_enabled};

    // Final output assignments
    assign uo_out = {status_led[1:0], overheat_warning, motor_pwm, 
                     right_indicator, horn_out, headlight_out, power_status};
    
    // FIXED: Output full motor speed value
    assign uio_out = motor_speed; // Full 8-bit motor speed on output pins

    // Tie off unused input to prevent warnings
    wire _unused = &{ena, mode_select, motor_active, 1'b0};

endmodule
