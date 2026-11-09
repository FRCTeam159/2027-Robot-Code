package frc.robot.objects;

import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import org.wpilib.hardware.bus.CANPort;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Motor {

    private static final CANPort CAN_PORT = Constants.can_port;

    boolean upperLimitNormalyClosed = false;
    boolean lowerLimitNormalyClosed = false;
    SparkMax rev_motor = null;
    RelativeEncoder rev_encoder;
    int m_chnl;
    boolean m_inverted = false;
    boolean m_enabled = true;
    SparkLimitSwitch m_upperLimit;
    SparkLimitSwitch m_lowerLimit;
    double m_dpr = 1;

   public Motor(int id, boolean isBrushed) {
        rev_motor = new SparkMax(CAN_PORT, id, isBrushed ? MotorType.kBrushed : MotorType.kBrushless);
        rev_encoder = rev_motor.getEncoder();
        m_chnl = id;
    }

    public Motor(int id) {
        rev_motor = new SparkMax(CAN_PORT, id, MotorType.kBrushless);
        rev_encoder = rev_motor.getEncoder();
        m_chnl = id;
    }

    public void setUpperLimit(boolean b) {
        upperLimitNormalyClosed = b;
        m_upperLimit = rev_motor.getForwardLimitSwitch();
    }

    public void setLowerLimit(boolean b) {
        lowerLimitNormalyClosed = b;
        m_lowerLimit = rev_motor.getReverseLimitSwitch();
    }

    public void setUpperLimit() {
        setUpperLimit(false);
    }

    public void setLowerLimit() {
        setLowerLimit(false);
    }

    public boolean atUpperLimit() {
        return m_upperLimit == null ? false : m_upperLimit.isPressed().get(false);
    }

    public boolean atLowerLimit() {
        return m_lowerLimit == null ? false : m_lowerLimit.isPressed().get(false);
    }

    public void setFollow(int lead){
        SparkMaxConfig config = new SparkMaxConfig();
        config.follow(lead);
        rev_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

     public void setConfig(boolean isInverted, boolean isBreak, double d, int lead) {
        m_dpr = d;
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(isInverted)
                .idleMode(isBreak ? IdleMode.kBrake : IdleMode.kCoast);
        if (m_upperLimit != null){
            LimitSwitchConfig.Type upperType = upperLimitNormalyClosed?LimitSwitchConfig.Type.kNormallyClosed:LimitSwitchConfig.Type.kNormallyOpen;
            config.limitSwitch.forwardLimitSwitchType(upperType);
        }
        if (m_lowerLimit != null){
            LimitSwitchConfig.Type lowerType = lowerLimitNormalyClosed?LimitSwitchConfig.Type.kNormallyClosed:LimitSwitchConfig.Type.kNormallyOpen;
            config.limitSwitch.reverseLimitSwitchType(lowerType);
        }

        if (lead != -1) {
            config.follow(lead);
        }
        // config.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .pid(1.0, 0.0, 0.0);

        rev_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setConfig(boolean isInverted, double d) {
        setConfig(isInverted, false, d, -1);
    }

    public void setConfig(double d) {
        setConfig(false, false, d, -1);
    }
    
    public void enable() {
        m_enabled = true;
    }

    public void disable() {
        m_enabled = false;
        rev_motor.disable();
    }

    public double getPosition() {
        return rev_encoder.getPosition().get() * m_dpr;
    }

    public void setPosition(double d) {
        rev_encoder.setPosition(d / m_dpr);
    }

    public double getRotations() {
        return getPosition() / m_dpr;
    }

    public void reset() {
        rev_encoder.setPosition(0.0);
    }

    public double getVelocity() {
        return rev_encoder.getVelocity().get() * m_dpr / 60 / 60; // same result as 2026 code
    }

    public void set(double speed) {
        rev_motor.setThrottle(speed);
    }

    public void setVoltage(double v) {
        rev_motor.setVoltage(v);
    }
}

