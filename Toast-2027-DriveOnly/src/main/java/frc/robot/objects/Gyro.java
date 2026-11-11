package frc.robot.objects;

/**
 * Software yaw estimate used while migrating away from the RoboRIO-only ADXRS450 SPI gyro.
 * Drivetrain integrates yaw from swerve wheel deltas. Wheel slip can make this estimate drift.
 */
public class Gyro {
    private double angleDegrees;

    public void reset() {
        angleDegrees = 0.0;
    }

    public double getAngle() {
        return angleDegrees;
    }

    public void addAngle(double deltaDegrees) {
        angleDegrees += deltaDegrees;
    }
}
