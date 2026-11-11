// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.driverstation.XboxController;
import org.wpilib.command2.Command;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.DriveStraight;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private final XboxController controller = new XboxController(0);
  private final Drivetrain drivetrain = new Drivetrain();

  public RobotContainer() {
    drivetrain.setDefaultCommand(new DriveWithGamepad(drivetrain, controller));
  }

  public void robotInit() {
    drivetrain.init();
    drivetrain.reset();
  }

  public void teleopInit() {
    drivetrain.resetOdometry();
    drivetrain.enable();
  }

  public void disabledInit() {
    drivetrain.disable();
  }

  public void simulationPeriodic() {
    drivetrain.updateSimulation();
  }

  public void autonomousInit() {
    drivetrain.resetOdometry();
    drivetrain.enable();
  }

  public Command getAutonomousCommand() {
    // Keep the original default autonomous behavior, without the removed chooser/vision stack.
    return new DriveStraight(drivetrain, 1.5);
  }
}
