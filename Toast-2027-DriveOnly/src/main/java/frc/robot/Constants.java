// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.hardware.bus.CANPort;

/** Add your docs here. */
public class Constants {

    // SystemCore CAN connector used by SPARK MAXes and the four Phoenix CANcoders.
    public static final CANPort can_port = CANPort.CAN_S0;

    public static final double kBackRightOffset = -0.169434;
    public static final double kBackLeftOffset = -0.215332;
    public static final double kFrontRightOffset = -0.400635;
    public static final double kFrontLeftOffset = 0.039062;
    // can ids for drive train
    public static final int kFl_Drive = 3;
    public static final int kFl_Turn = 8;
    public static final int kFl_Encoder = 12;

    public static final int kFr_Drive = 7;
    public static final int kFr_Turn = 6;
    public static final int kFr_Encoder = 10;

    public static final int kBl_Drive = 2;
    public static final int kBl_Turn = 4;
    public static final int kBl_Encoder = 9;

    public static final int kBr_Drive = 5;
    public static final int kBr_Turn = 1;
    public static final int kBr_Encoder = 11;

}
