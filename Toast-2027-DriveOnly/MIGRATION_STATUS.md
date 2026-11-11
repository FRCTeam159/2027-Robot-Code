# Toast-2027-DriveOnly migration status

This separate copy is a drivetrain-focused SystemCore/2027-alpha project. The full Toast-2027 project and Toast-2026 are unchanged. Toolchain: WPILib `2027_alpha7`, GradleRIO `2027.0.0-alpha-8`, Java 25, Gradle 9.4.1.

## Included

- Four-module swerve drive, SPARK MAX drive/turn motors, Phoenix CANcoder steering encoders, field-relative toggle, and wheel-odometry pose estimate.
- Xbox controller default command.
- A basic 1.5 m drive-straight autonomous command; path-following and tag-based autonomous were removed.
- NetworkTables telemetry/settings via `RobotDashboard`.

## Removed

Arm and roller subsystem/commands, climber subsystem/commands, navX/arm sensor code, AprilTag detector/vision, drive-to-tag, old trajectory/path command, and plotting/dashboard files.

## Hardware caveat

The RoboRIO ADXRS450 gyro is not included. Heading is estimated from swerve wheel deltas and may drift with slip. This is a compile-ready drivetrain scaffold, not yet verified on a SystemCore robot. The project targets `CAN_S0`; confirm wiring and device IDs. Phoenix 6 is `26.70.0-alpha-2`, which requires matching 26.70.x firmware. Build success does not prove deployment or hardware behavior; test with the robot secured and outputs limited.
