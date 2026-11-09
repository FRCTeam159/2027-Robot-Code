// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveWithGamepad;
import frc.robot.subsystems.Drivetrain;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.XboxController;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final XboxController driver = new XboxController(0);

  public RobotContainer() {
    drivetrain.init();
    drivetrain.setDefaultCommand(new DriveWithGamepad(drivetrain, driver));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
