package frc.robot.commands;

import org.wpilib.driverstation.XboxController;
import org.wpilib.command2.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveWithGamepad extends Command {
    private static final double STICK_DEADBAND = 0.2;

    private final Drivetrain drivetrain;
    private final XboxController controller;

    public DriveWithGamepad(Drivetrain drive, XboxController xbox){
        drivetrain = drive;
        controller = xbox;

        // Since 2027 the controller applies the deadband itself (default 0.1).
        controller.setLeftYDeadband(STICK_DEADBAND);
        controller.setRightXDeadband(STICK_DEADBAND);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double drive_speed = controller.getLeftY();
        double turn_speed = controller.getRightX();

        drivetrain.drive(drive_speed, turn_speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DriveWithGamepad cancelled");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
