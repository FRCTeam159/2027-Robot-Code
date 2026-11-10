// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.hardware.bus.CANPort;

/** Add your docs here. */
public class Constants {
    // Systemcore CAN connector the drive motors are plugged into. Set this to match the wiring.
    public static final CANPort can_port = CANPort.CAN_S0;

    public static final int left_lead_id = 1;
    public static final int left_follow_id = 2;
    public static final int right_lead_id = 3;
    public static final int right_follow_id = 4;

    // Starting simulation assumptions: a 6-inch-wheel kit drivetrain with two CIMs
    // per side, a 10.71:1 reduction, and 0.60 m track width. Update these to match
    // the actual robot before using simulation results for tuning.
    public static final double sim_track_width_meters = 0.60;
    public static final double sim_gear_ratio = 10.71;
    public static final double sim_wheel_diameter_meters = 0.1524;
    public static final double sim_battery_voltage = 12.0;
    public static final double sim_loop_period_seconds = 0.02;
}
