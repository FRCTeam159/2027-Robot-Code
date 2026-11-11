// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command2.Command;
import frc.robot.subsystems.Drivetrain;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveStraight extends Command {
  /** Creates a new DriveStraight. 
 * @param m_drivetrain */
  Drivetrain m_drive;
  PIDController m_PID;
  double m_target=1;
  public DriveStraight(Drivetrain drive, double t) {
    // Use addRequirements() here to declare subsystem dependencies
    m_PID = new PIDController(0.1, 0, 0);
    m_drive = drive;
    m_target = t;
    addRequirements(drive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // System.out.println("Drive straight target = " + m_target);
    m_PID.setSetpoint(m_target);
    m_PID.setTolerance(0.02);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d p = m_drive.getPose();
    double s = p.getTranslation().getX();
    double d = m_PID.calculate(s, m_target);
    System.out.println("distance = " + s + " correction = " + d);
    m_drive.drive(d, 0, 0, false);
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0, 0.0, 0.0, false);
    System.out.println("DriveStraight.end " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atTarget = m_PID.atSetpoint();
    if(atTarget)
      System.out.println("DriveStraight Target Reached");
    return atTarget;
  }
}
