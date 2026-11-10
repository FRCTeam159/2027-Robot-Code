package frc.robot.subsystems;

import static frc.robot.Constants.*;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.simulation.DifferentialDrivetrainSim;
import org.wpilib.simulation.DifferentialDrivetrainSim.KitbotGearing;
import org.wpilib.simulation.DifferentialDrivetrainSim.KitbotMotor;
import org.wpilib.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.SubsystemBase;
import frc.robot.objects.Motor;

public class Drivetrain extends SubsystemBase {
    private final Motor left_lead_motor = new Motor(left_lead_id, true);
    private final Motor right_lead_motor = new Motor(right_lead_id, true);
    private final Motor left_follow_motor = new Motor(left_follow_id, true);
    private final Motor right_follow_motor = new Motor(right_follow_id, true);

    private final DifferentialDrivetrainSim drivetrain_sim = DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.DUAL_CIM_PER_SIDE,
            KitbotGearing.RATIO_10P71,
            KitbotWheelSize.SIX_INCH,
            sim_track_width_meters,
            VecBuilder.fill(0, 0, 0, 0, 0, 0, 0));
    private final Field2d field = new Field2d();
    private double left_command;
    private double right_command;

    double drive_scale = 1;
    double turn_scale = 1;

    public void setConfig(){
        left_follow_motor.setConfig(true, false, 1, left_lead_id);
        right_follow_motor.setConfig(false, false, 1, right_lead_id);

        left_lead_motor.setConfig(true, 1);
        right_lead_motor.setConfig(false, 1);
    }

    public void init(){
        setConfig();

        left_lead_motor.enable();
        right_lead_motor.enable();
        left_follow_motor.enable();
        right_follow_motor.enable();
    }

    public void drive(double drive_speed, double turn_speed){
        double left_speed = drive_speed * drive_scale + turn_speed * turn_scale;
        double right_speed = drive_speed * drive_scale - turn_speed * turn_scale;

        double max_speed = Math.max(Math.abs(left_speed), Math.abs(right_speed));
        if (max_speed > 1) {
            left_speed /= max_speed;
            right_speed /= max_speed;
        }

        left_command = left_speed;
        right_command = right_speed;
        left_lead_motor.set(left_speed);
        right_lead_motor.set(right_speed);
    }

    @Override
    public void simulationPeriodic() {
        if (!RobotBase.isSimulation()) {
            return;
        }

        drivetrain_sim.setInputs(
            left_lead_motor.getSimAppliedOutput() * sim_battery_voltage,
            right_lead_motor.getSimAppliedOutput() * sim_battery_voltage);
        drivetrain_sim.update(sim_loop_period_seconds);

        double wheel_circumference = Math.PI * sim_wheel_diameter_meters;
        double metersToMotorRotations = sim_gear_ratio / wheel_circumference;
        double leftPositionRotations = drivetrain_sim.getLeftPosition() * metersToMotorRotations;
        double leftVelocityRpm = drivetrain_sim.getLeftVelocity() * metersToMotorRotations * 60.0;
        double rightPositionRotations = drivetrain_sim.getRightPosition() * metersToMotorRotations;
        double rightVelocityRpm = drivetrain_sim.getRightVelocity() * metersToMotorRotations * 60.0;
        left_lead_motor.updateSimState(
            leftPositionRotations, leftVelocityRpm, sim_battery_voltage, sim_loop_period_seconds);
        left_follow_motor.updateSimState(
            leftPositionRotations, leftVelocityRpm, sim_battery_voltage, sim_loop_period_seconds);
        right_lead_motor.updateSimState(
            rightPositionRotations, rightVelocityRpm, sim_battery_voltage, sim_loop_period_seconds);
        right_follow_motor.updateSimState(
            rightPositionRotations, rightVelocityRpm, sim_battery_voltage, sim_loop_period_seconds);

        field.setRobotPose(drivetrain_sim.getPose());
        var fieldTelemetry = Telemetry.getTable("Drivetrain");
        fieldTelemetry.setType(field.getTelemetryType());
        field.logTo(fieldTelemetry);

        var debugTelemetry = Telemetry.getTable("DriveDebug");
        debugTelemetry.log("enabled", RobotBase.isEnabled());
        debugTelemetry.log("leftCommand", left_command);
        debugTelemetry.log("rightCommand", right_command);
        debugTelemetry.log("leftSparkSetpoint", left_lead_motor.getSimSetpoint());
        debugTelemetry.log("rightSparkSetpoint", right_lead_motor.getSimSetpoint());
        debugTelemetry.log("leftSparkOutput", left_lead_motor.getSimAppliedOutput());
        debugTelemetry.log("rightSparkOutput", right_lead_motor.getSimAppliedOutput());
        debugTelemetry.log("leftPoseMeters", drivetrain_sim.getPose().getX());
        debugTelemetry.log("yPoseMeters", drivetrain_sim.getPose().getY());
    }

}
