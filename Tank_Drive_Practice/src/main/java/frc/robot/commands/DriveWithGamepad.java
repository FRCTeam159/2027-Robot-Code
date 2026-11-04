package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveWithGamepad extends Command {
    private final Drivetrain drivetrain;
    private final XboxController controller;

    public DriveWithGamepad(Drivetrain drive, XboxController xbox){
        drivetrain = drive;
        controller = xbox;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double vx = controller.getLeftY();
        double vr = controller.getRightX();

        double drive_speed = MathUtil.applyDeadband(vx, 0.2);
        double turn_speed = MathUtil.applyDeadband(vr, 0.2);

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
