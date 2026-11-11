// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import frc.robot.Constants;
import org.wpilib.framework.RobotBase;


/** Add your docs here. */
public class Encoder {
    private static final CANBus CAN_BUS = new CANBus(Constants.can_port);
    CANcoder can_encoder;
    double offset=0;
    private double simPositionRotations;
    private boolean simPositionSet;
    CANcoderConfiguration config = new CANcoderConfiguration();

   
    public Encoder(int id) {
        can_encoder = new CANcoder(id, CAN_BUS);
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // -0.5 to 0.5 in revolutions (Signed_PlusMinusHalf) 
        can_encoder.getConfigurator().apply(config);
        //can_encoder.setPosition(0);
        offset=0;
 
        //setOffset(0);
    }
    
    public void setOffset(double d){
        offset=d;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // -0.5 to 0.5 in revolutions (Signed_PlusMinusHalf) 
        config.MagnetSensor.MagnetOffset = -offset; // in rotations   
        can_encoder.getConfigurator().apply(config);
        //can_encoder.setPosition(0);
        System.out.println("Offset = " + offset);
    }
   
    public double getPosition() {// return rotations
        if (RobotBase.isSimulation() && simPositionSet) {
            return simPositionRotations;
        }
        StatusSignal<Angle> val = can_encoder.getPosition(true);
        double pos =val.getValueAsDouble();
        System.out.println("pos = " + getAbsolutePosition());
        return pos;
    }

    /** Set the steering position returned by this encoder in desktop simulation. */
    public void setSimPosition(double rotations) {
        simPositionRotations = rotations;
        simPositionSet = true;
    }
    public double getAbsolutePosition() {// return rotations
        StatusSignal<Angle> val = can_encoder.getAbsolutePosition(true);
        return val.getValueAsDouble()-offset; 
    }
    public AngularVelocity getRate() { // return rotations/s
        StatusSignal<AngularVelocity> val = can_encoder.getVelocity();
        return val.getValue(); // return radians for rotations
     }

}
