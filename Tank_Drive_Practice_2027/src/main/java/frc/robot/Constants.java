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
}
