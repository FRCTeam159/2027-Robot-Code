// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.trajectory.TrapezoidProfile;
import frc.robot.utils.RobotDashboard;
import frc.robot.objects.Motor;
import frc.robot.objects.Encoder;

public class SwerveModule {

  private static final double kSimBusVoltage = 12.0;
  private static final double kSimDriveAcceleration = 2.0;
  private static final double kSimTurnRateRadiansPerSecond = Math.toRadians(720.0);

  private Motor m_driveMotor=null;
	private Motor m_turnMotor=null;
  private Encoder m_turnEncoder=null;
  double m_valueOfPID=0;
  double m_targetAngle=0;
  
  int cnt=0;


  public static boolean swerveEncoders=true; // false:use REV encoders (ala Waffle) true: use CANcoders

  public static String chnlnames[] = { "FL", "FR", "BL", "BR" };

  String name;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0.0, 0);
  //private final PIDController m_turningPIDController = new PIDController(3, 0.05, 0);
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(0.5,0,0,
          new TrapezoidProfile.Constraints(
              Drivetrain.kMaxAngularVelocity, Drivetrain.kMaxAngularAcceleration));

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.01, 0.25);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
  //private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.01, 0.25 /*Old: 0.25*/);
 
  public int m_drive_chnl;
  public int m_turn_chnl;
  
  boolean m_enabled=false;

  private double m_simDrivePositionMeters;
  private double m_simDriveVelocityMetersPerSecond;
  private double m_simTurnPositionRotations;
  private double m_simTargetVelocityMetersPerSecond;
  private double m_simTargetAngleRadians;

  static boolean debug=false;
  
  public SwerveModule(int driveMotorChannel,int turningMotorChannel,int turningEncoderChannel, int i) {
    this(driveMotorChannel,turningMotorChannel,i);
    if(swerveEncoders)
      m_turnEncoder=new Encoder(turningEncoderChannel);
  }
  public SwerveModule(int driveMotorChannel,int turningMotorChannel,int  i) {
    m_drive_chnl=driveMotorChannel;
    m_turn_chnl=turningMotorChannel;

    m_driveMotor=new Motor(driveMotorChannel);
    m_turnMotor=new Motor(turningMotorChannel);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    name = chnlnames[i - 1];
   
  }

  public void setConfig(boolean isInverted, double distancePerRotation) {
    m_driveMotor.setConfig(isInverted, distancePerRotation);
  }

  public void setOffset(double offset){
      m_turnEncoder.setOffset(offset);
  }
  public void enable(){
    m_enabled=true;
    m_driveMotor.enable();
    m_turnMotor.enable();
  }
  public void disable(){
     m_enabled=false;
    m_driveMotor.disable();
    m_turnMotor.disable();
  }
  public void reset(){
     m_drivePIDController.reset();
    m_turningPIDController.reset(0.0);
    m_enabled=false;
    m_driveMotor.reset();
    m_turnMotor.reset();
    m_simDrivePositionMeters = 0.0;
    m_simDriveVelocityMetersPerSecond = 0.0;
    m_simTurnPositionRotations = 0.0;
    m_simTargetVelocityMetersPerSecond = 0.0;
    m_simTargetAngleRadians = 0.0;
  }

  public double heading(){
    return 2*Math.PI*getRotations();
  }
  public double getRotations(){
    return m_turnEncoder.getPosition();
  }
  
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(heading());
  }
 
  public SwerveModuleVelocity getState() {
    return new SwerveModuleVelocity(m_driveMotor.getVelocity(),  getRotation2d());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveMotor.getPosition(),  getRotation2d());
  }

  public double getDistance(){
    return m_driveMotor.getPosition();
  }

  public double getVelocity() {
      return m_driveMotor.getVelocity();
  }
  
  public void setDesiredState(SwerveModuleVelocity desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    // SwerveModuleState state = desiredState;  // don't optimize
    SwerveModuleVelocity state = desiredState.optimize(getRotation2d());
    m_simTargetVelocityMetersPerSecond = state.velocity;
    m_simTargetAngleRadians = state.angle.getRadians();

    double velocity=getVelocity();
    
    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getVelocity(), state.velocity);
    final double driveFeedforward = m_driveFeedforward.calculate(state.velocity);
    double turn_angle=getRotation2d().getRadians(); // rotations in radians
    m_targetAngle = state.angle.getRadians();

    final double turnOutput = -m_turningPIDController.calculate(turn_angle,state.angle.getRadians());
    final double turnFeedforward = 0;//m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    //final double turnFeedforward = m_turnFeedforward.calculate(state.angle.getRadians());
    m_valueOfPID = turnOutput;

    double set_drive=driveOutput+driveFeedforward;
    double set_turn=turnOutput;//+turnFeedforward;

    if(debug){
      String s = String.format("Vel %-2.2f(%-2.2f) -> %-2.2f Angle %-3.3f(%-2.3f) -> %-2.3f\n", 
      velocity,state.velocity,set_drive,Math.toDegrees(turn_angle), state.angle.getDegrees(), set_turn); 
      RobotDashboard.putString(name, s);
    }
    m_driveMotor.set(set_drive);
    m_turnMotor.set(set_turn);
  }

  public void turnAndMove(double m, double t) {
    move(m);
    turn(t);
  }

  // just apply a voltage to the turn motor
  public void turn(double value){
    m_turnMotor.set(value);
  }

  // just apply a voltage to the wheel motor
  public void move(double value){
    m_driveMotor.set(value);
  }
  // use a PID controller to set an explicit turn angle
  public void setAngle(double a, double d){
    double current=m_turnMotor.getPosition();
    current=current%360;
    double turnOutput = m_turningPIDController.calculate(Math.toRadians(current),Math.toRadians(a) );
    //System.out.println(m_drive_chnl/2+" "+a+" "+current+" "+turnOutput+" "+d);
    //m_driveMotor.set(d);
    //m_turnMotor.set(turnOutput); 
  }
  
  public boolean wheelReset() {
    return m_turningPIDController.atSetpoint();
  }
  public void alignWheel(){
    setAngle(0,0);
 }
  public void resetWheel(){
    alignWheel();
  }
  public double getAngle(){
    return  m_turnMotor.getPosition();
  }
  public double getDegrees(){
    return  Math.toDegrees(getAngle());
  }
  public double getMoveRate(){
    return  m_driveMotor.getVelocity();
  }
  public double getTurnRate(){
    return  m_turnMotor.getVelocity();
  }

  /**
   * Advance a lightweight swerve-module model for desktop simulation. Drive and
   * turn motors use their simulated SPARK outputs; sensor values are then fed
   * back through the same encoder paths used by odometry and control.
   */
  public void simulationPeriodic(double deltaSeconds) {
    double maxDriveDelta = kSimDriveAcceleration * deltaSeconds;
    double driveDelta = m_simTargetVelocityMetersPerSecond - m_simDriveVelocityMetersPerSecond;
    driveDelta = Math.max(-maxDriveDelta, Math.min(maxDriveDelta, driveDelta));
    m_simDriveVelocityMetersPerSecond += driveDelta;
    m_simDrivePositionMeters += m_simDriveVelocityMetersPerSecond * deltaSeconds;

    double currentAngle = m_simTurnPositionRotations * 2.0 * Math.PI;
    double angleDelta = Math.atan2(
      Math.sin(m_simTargetAngleRadians - currentAngle),
      Math.cos(m_simTargetAngleRadians - currentAngle));
    double maxTurnDelta = kSimTurnRateRadiansPerSecond * deltaSeconds;
    angleDelta = Math.max(-maxTurnDelta, Math.min(maxTurnDelta, angleDelta));
    m_simTurnPositionRotations += angleDelta / (2.0 * Math.PI);

    double drivePositionRotations = m_simDrivePositionMeters / Drivetrain.kDistPerRot;
    double driveVelocityRpm = m_simDriveVelocityMetersPerSecond / Drivetrain.kDistPerRot * 60.0;
    m_driveMotor.updateSimState(
      drivePositionRotations, driveVelocityRpm, kSimBusVoltage, deltaSeconds);

    double turnVelocityRpm = angleDelta / deltaSeconds
      * Drivetrain.kTurnGearRatio * 60.0 / (2.0 * Math.PI);
    if (m_turnEncoder != null) {
      m_turnEncoder.setSimPosition(m_simTurnPositionRotations);
    }
    m_turnMotor.updateSimState(
        m_simTurnPositionRotations * Drivetrain.kTurnGearRatio,
      turnVelocityRpm,
        kSimBusVoltage,
        deltaSeconds);
  }

  public void log (){
    if(debug){
      String s = String.format("angle %g PID %g Target Angle %g\n", 
      getRotations(),
      m_valueOfPID,
      m_targetAngle);
      RobotDashboard.putString(name, s);
      //if(name=="FR" && (cnt%10)==0)
      //System.out.println(s);
      cnt++;
    }
  }
}
