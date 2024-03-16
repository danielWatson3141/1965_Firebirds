package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;

public class MecanumDrivetrain extends SubsystemBase {

  public final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // public double gyroAngle = m_gyro.getAngle();

  private SlewRateLimiter throttleLimiterX;
  private SlewRateLimiter throttleLimiterY;
  private SlewRateLimiter rotationLimiter;

  private final double TRANSLATION_DEADZONE = 0.08;
  private final double ROTATION_DEADZONE = 0.2;

  private final double ROTATION_TOLERANCE = 3; // degrees
  // private double locationX = 0.2794;
  // private double locationY = 0.3048;

  Joystick m_stick;

  Rotation2d gyroAngle;
  Rotation2d POVvalue;

  Rotation2d testDriveAngle;
  double currentTestDriveAngle;
  int testDriveState;

  CANSparkMax m_frontLeft = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_rearLeft = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax m_frontRight = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax m_rearRight = new CANSparkMax(4, MotorType.kBrushless);

  RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();
  RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();
  RelativeEncoder m_frontRightEncoder = m_frontRight.getEncoder();
  RelativeEncoder m_rearRightEncoder = m_rearRight.getEncoder();

  double initialFLEncoder;
  double initialFREncoder;
  double initialRLEncoder;
  double initialRREncoder;

  double encoderDistanceMean;

  private double rSetpoint = 0;
  private double rError = 0;
  private double KpSlider = 3;
  private PIDController rotationPID = new PIDController(2, 0, 0);

  private double tXError = 0;
  private double tYError = 0;
  private double xTranslation = 0;
  private double yTranslation = 0;
  private final double SHOOT_DISTANCE = 0;
  private PIDController translationPID = new PIDController(0.6, 0, 0);

  private final double ROTATION_RATE = 4;
  private final double ROTATION_CAP = .6;
  private final double TRANSLATION_RATE = 4;

  private boolean ROTATION_LOCK = false;
  private boolean rotationFeedback = true;

  private final long DRIVE_AUTO_WAIT = 500;// fast speed for initial testing
  private final double DRIVE_AUTO_SPEED = 0.2;

  public boolean fieldRelative = false;

  MecanumDrive m_robotDrive = new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);

  public double setPoint;

  private final PhotonVision m_photonvision = new PhotonVision();
  private final Vision m_vision = new Vision();

  private final Translation2d m_frontLeftLocation = new Translation2d(0, 0);
  private final Translation2d m_frontRightLocation = new Translation2d(0, -0);
  private final Translation2d m_backLeftLocation = new Translation2d(-0, 0);
  private final Translation2d m_backRightLocation = new Translation2d(-0, -0);

  MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(m_frontRightLocation, m_frontLeftLocation, m_backRightLocation, m_backLeftLocation);

  private final MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, m_gyro.getRotation2d(), getCurrentDistances());


  public MecanumDrivetrain(Joystick input_stick) {

    m_gyro.calibrate();
    resetEncoder();

    currentTestDriveAngle = 0;
    testDriveState = 0;

    m_frontLeft.setIdleMode(IdleMode.kBrake);
    m_rearLeft.setIdleMode(IdleMode.kBrake);
    m_frontRight.setIdleMode(IdleMode.kBrake);
    m_rearRight.setIdleMode(IdleMode.kBrake);

    m_stick = input_stick;

    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);

    throttleLimiterX = new SlewRateLimiter(TRANSLATION_RATE);
    throttleLimiterY = new SlewRateLimiter(TRANSLATION_RATE);
    rotationLimiter = new SlewRateLimiter(ROTATION_RATE);

    SmartDashboard.putNumber("Throttle max%", 100);
    SmartDashboard.putNumber("Kp value", KpSlider);

    rotationPID.setTolerance(ROTATION_TOLERANCE);

  }

  // multipliers for values
  final double SPEED_CAP = 1;
  public double driveSpeed;

  public void setSpeed() {
    // get percentage from the 4th axis and converts it from 0% - 100%
    double throttle_value = ((-m_stick.getRawAxis(3) + 1) / 2) * (SmartDashboard.getNumber("Throttle max%", 100) / 100);
    // sets the sped based on the cap and percentage
    driveSpeed = SPEED_CAP * throttle_value;
    // documents the current percentage of the motors for driver
    // drivePercentEntry.setDouble(throttle_value * 100);
  }

  public double deadzone(double input, double deadzone) {
    double absInput = Math.abs(input);
    if (absInput <= deadzone) {
      return 0;
    } else {
      double value = (absInput - deadzone) / (1 - deadzone);
      if (input < 0) {
        value = -value;
      }
      return value;
    }
  }

  private final double ENCODER_CONVERSION_FACTOR = 25;

  public MecanumDriveWheelPositions getCurrentDistances(){
    return new MecanumDriveWheelPositions(
      (m_frontLeftEncoder.getPosition() / ENCODER_CONVERSION_FACTOR),
      (m_frontRightEncoder.getPosition() / ENCODER_CONVERSION_FACTOR),
      (m_rearLeftEncoder.getPosition() / ENCODER_CONVERSION_FACTOR),
      (m_rearRightEncoder.getPosition() / ENCODER_CONVERSION_FACTOR)
    );

  }

  public double getTotalDistanceTravelled() {
    double frontLeftEncoderDistance = (m_frontLeftEncoder.getPosition() - initialFLEncoder) / ENCODER_CONVERSION_FACTOR;
    double frontRightEncoderDistance = (m_frontRightEncoder.getPosition() - initialFREncoder) / ENCODER_CONVERSION_FACTOR;
    double rearLeftEncoderDistance = (m_rearLeftEncoder.getPosition() - initialRLEncoder) / ENCODER_CONVERSION_FACTOR;
    double rearRightEncoderDistance = (m_rearRightEncoder.getPosition() - initialRREncoder) / ENCODER_CONVERSION_FACTOR;

    encoderDistanceMean = (frontLeftEncoderDistance + frontRightEncoderDistance + rearLeftEncoderDistance + rearRightEncoderDistance) / 4;

    return encoderDistanceMean;
  }

  public void resetEncoder() {
    initialFLEncoder = m_frontLeftEncoder.getPosition();
    initialFREncoder = m_frontRightEncoder.getPosition();
    initialRLEncoder = m_rearLeftEncoder.getPosition();
    initialRREncoder = m_rearRightEncoder.getPosition();
  }

  public Rotation2d gyroAngle() {
    gyroAngle = m_gyro.getRotation2d().times(-1);
    return gyroAngle;
  }

  public void gyroReset() {
    m_gyro.reset();
    rSetpoint = 0;
    Logging.log(getSubsystem(), "reset gyroscope");
  }

  public void stopTestDrive(){
    currentTestDriveAngle = 0;
    testDriveState = 0;
    driveAuto(0);
  }

  public void testDrive(double speed){
    if (testDriveState <= 1) {
      testDriveState = testDriveState + 1;
    }
    else {
      currentTestDriveAngle = currentTestDriveAngle + 45;
      testDriveState = 0;
    }

    testDriveAngle = Rotation2d.fromDegrees(currentTestDriveAngle);

    m_robotDrive.drivePolar(speed, testDriveAngle, 0);
  }

  public void switchDriveRelativity() {
    fieldRelative = !fieldRelative;
  }

  public void lifterModeToggle(){
    if (rotationFeedback){
      rotationFeedback = false;
    }
    else{
      gyroReset();
      rotationFeedback = true;
    }

  }

  public void driveAuto(double autoSpeed) {
    m_frontLeft.set(autoSpeed);
    m_rearLeft.set(autoSpeed);
    m_frontRight.set(autoSpeed);
    m_rearRight.set(autoSpeed);
  }

  public void drive() {

    gyroAngle();

    double drive_x = 0;
    double drive_y = 0;
    double drive_z = 0;

    // **** POV drive mode
    // If the POV stick is active, put raw POV input into the drivetrain.
    // TODO: This will not have directional stability. Remove / Update?
    if (m_stick.getPOV() != -1) {
      POVvalue = Rotation2d.fromDegrees(m_stick.getPOV());
      m_robotDrive.drivePolar(driveSpeed, POVvalue, 0);

      // **** April Tag Tracking mode
      // If the vision system has a detection and button 3 is held, track the april
      // tag and position the robot directly in front of it.
    } else if (m_vision.myPosition != null && m_stick.getRawButton(3)) {

      if (m_vision.VISION_WORKING) {

        if (m_photonvision.hasTargets){
          
        }
        
      }

      // **** Joystick Control Mode
      // If neither POV mode nor tracking mode is active, then take stick input.
    } else {
      // Rotation is locked by default, unlock when button is held
      if (!ROTATION_LOCK || m_stick.getRawButton(2)) {
        drive_z = rotationLimiter.calculate((deadzone(m_stick.getZ(), ROTATION_DEADZONE)) * ROTATION_CAP);
      } else {
        drive_z = 0;
      }

      // Update the setpoint by the drive_z input, this moves the desired heading.
      rSetpoint = (rSetpoint + drive_z);

      // Calculate the current error, we will pass this to the PID loop for corrective
      // input
      rError = (m_gyro.getAngle() - rSetpoint);

      // Pull the translation input from the stick, apply deadzone and slew rateP
      drive_x = throttleLimiterX.calculate(deadzone(m_stick.getX(), TRANSLATION_DEADZONE)) * driveSpeed;
      drive_y = throttleLimiterY.calculate(deadzone(-m_stick.getY(), TRANSLATION_DEADZONE)) * driveSpeed;

    }

    // Mecanum seems to consider 'X' the forward direction, so we're passing y, x, z
    // to driveCartesian on purpose.
    m_robotDrive.driveCartesian(
        drive_y,
        drive_x,
        rotationFeedback ? rotationPID.calculate(rError, 0) / 180 : drive_z,
        fieldRelative ? gyroAngle : Rotation2d.fromDegrees(0));

    // stickXEntry.setDouble(drive_x);
    // stickYEntry.setDouble(drive_y);
    // stickZEntry.setDouble(drive_z);
  }

  public void drivetrainTestLogging(){

    SmartDashboard.putNumber("rotation setpoint", rSetpoint);
    SmartDashboard.putNumber("rotation error", rError);

    SmartDashboard.putNumber("gyro angle graph", m_gyro.getAngle());

    SmartDashboard.putNumber("encoder value", getTotalDistanceTravelled());

    SmartDashboard.putNumber("FL_SPEED", m_frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("RL_SPEED", m_rearLeftEncoder.getVelocity());
    SmartDashboard.putNumber("FR_SPEED", m_frontRightEncoder.getVelocity());
    SmartDashboard.putNumber("RR_SPEED", m_rearRightEncoder.getVelocity());
  }

  @Override
  public void periodic() {

    //part of drive mode
    SmartDashboard.putBoolean("lifter mode", rotationFeedback);

    setSpeed();

    rotationPID.setP(KpSlider);
  }
}