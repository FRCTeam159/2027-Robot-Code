package frc.robot.subsystems;

import static frc.robot.Constants.*;

import org.wpilib.command2.SubsystemBase;
import frc.robot.objects.Motor;

public class Drivetrain extends SubsystemBase{
    private Motor left_lead_motor = new Motor(left_lead_id, true);
    private Motor right_lead_motor = new Motor(right_lead_id, true);
    private Motor left_follow_motor = new Motor(left_follow_id, true);
    private Motor right_follow_motor = new Motor(right_follow_id, true);

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

        left_lead_motor.set(left_speed);
        right_lead_motor.set(right_speed);
    }

}
