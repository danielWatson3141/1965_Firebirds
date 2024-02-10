package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Logging;

public class MecanumDrivetrain extends SubsystemBase {

  public final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // public double gyroAngle = m_gyro.getAngle();

  private SlewRateLimiter rotationLimiter;
  private SlewRateLimiter throttleLimiterX;
  private SlewRateLimiter throttleLimiterY;

  private double initialRotationValue;
  private final double STICK_DEADZONE = 0.08;
  private final double ROTATION_TOLERANCE = 3; //degrees
  // private double locationX = 0.2794;
  // private double locationY = 0.3048;

  Joystick m_stick;

  Rotation2d gyroAngle;
  Rotation2d POVvalue;

  CANSparkMax m_frontLeft =  new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_rearLeft = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax m_frontRight =  new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax m_rearRight =  new CANSparkMax(4, MotorType.kBrushless);

  RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();
  RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();
  RelativeEncoder m_frontRightEncoder = m_frontRight.getEncoder();
  RelativeEncoder m_rearRightEncoder = m_rearRight.getEncoder();
  private double rSetpoint;
  private double rSetpointTracker;
  private double rError;
  private double KpSlider = 1;
  PIDController rotationPID = new PIDController(3, 0, 0);

  private double rotationRate = 0.5;
  private double throttleRate = 0.5;
  private long driveAutoWait = 3000;

  private double drive_x;
  private double drive_y;
  private double drive_z;

  public boolean fieldOrientation;

  MecanumDrive m_robotDrive;

  public double setPoint;



  /*
   * ChassisSpeeds chassisSpeed = new ChassisSpeeds(m_stick.getY(),
   * m_stick.getX(), 0);
   * 
   * Translation2d m_frontLeftLocation = new Translation2d(locationX, locationY);
   * Translation2d m_frontRightLocation = new Translation2d(locationX,
   * -locationY);
   * Translation2d m_rearLeftLocation = new Translation2d(-locationX, locationY);
   * Translation2d m_rearRightLocation = new Translation2d(-locationX,
   * -locationY);
   * 
   * MecanumDriveKinematics m_kinematics = new
   * MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
   * m_rearLeftLocation, m_rearRightLocation);
   * 
   * 
   * ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);
   * MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
   * 
   * double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
   * double frontRight = wheelSpeeds.frontRightMetersPerSecond;
   * double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
   * double backRight = wheelSpeeds.rearRightMetersPerSecond;
   */

  public MecanumDrivetrain(Joystick input_stick) {

    m_frontLeft.setIdleMode(IdleMode.kBrake);
    m_rearLeft.setIdleMode(IdleMode.kBrake);
    m_frontRight.setIdleMode(IdleMode.kBrake);
    m_rearRight.setIdleMode(IdleMode.kBrake);

    m_robotDrive = new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);

    m_stick = input_stick;

    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);

    rotationLimiter = new SlewRateLimiter(rotationRate);
    throttleLimiterX = new SlewRateLimiter(throttleRate);
    throttleLimiterY = new SlewRateLimiter(throttleRate);

    initialRotationValue = 0;

    rSetpoint = 0;
    rSetpointTracker = 0;
    rError = 0;

    SmartDashboard.putBoolean("Feild/Robot", true);
    SmartDashboard.putNumber("Throttle max%", 100);

    rotationPID.setTolerance(3);

    //for testing
    SmartDashboard.putNumber("Kp value", KpSlider);
  }

  // multipliers for values
  final double SPEED_CAP = .6;
  public double driveSpeed;

  public void setSpeed() {
    // get percentage from the 4th axis and converts it from 0% - 100%
    double throttle_value = ((-m_stick.getRawAxis(3) + 1) / 2) * (SmartDashboard.getNumber("Throttle max%", 100) / 100);
    // sets the sped based on the cap and percentage
    driveSpeed = SPEED_CAP * throttle_value;
    // documents the current percentage of the motors for driver
    SmartDashboard.putNumber("Drive %", throttle_value * 100);
  }

  public double deadzone(double input) {
    if (Math.abs(input) < STICK_DEADZONE) {
      return 0;
    } else {
      return input;
    }
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

  public void driveAuto(double autoSpeed) {
    m_frontLeft.set(autoSpeed);
    m_rearLeft.set(autoSpeed);
    m_frontRight.set(autoSpeed);
    m_rearRight.set(autoSpeed);
  }

  public Command driveAutoCommand() {
    Command r_command = Commands.sequence(
        new InstantCommand(() -> driveAuto(0.3)),
        Commands.waitSeconds(driveAutoWait),
        new InstantCommand(() -> driveAuto(0)));

    r_command.addRequirements(this);
    return r_command;
  }

  public void drive() {

    gyroAngle();


    if (m_stick.getPOV() != -1) {
      POVvalue = Rotation2d.fromDegrees(m_stick.getPOV());
      m_robotDrive.drivePolar(driveSpeed, POVvalue, 0);
    } else {
      drive_x = throttleLimiterX.calculate(deadzone(m_stick.getX())) * driveSpeed;
      drive_y = throttleLimiterY.calculate(deadzone(-m_stick.getY())) * driveSpeed;
      drive_z = rotationLimiter.calculate(deadzone(m_stick.getZ())) * driveSpeed;

      rSetpoint = (rSetpoint + drive_z) % 360;
      rError = (m_gyro.getAngle() - rSetpoint); //* (1/180);
      //Mecanum seems to consider 'X' the forward direction, so we're passing y, x, z to driveCartesian on purpose.
      m_robotDrive.driveCartesian(
        drive_y,
        drive_x,
        rotationPID.calculate(rError, 0) / 180,
        gyroAngle
        );

    }

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("stickX", drive_x);
    SmartDashboard.putNumber("stickY", drive_y);
    SmartDashboard.putNumber("stickZ", drive_z);

    SmartDashboard.putNumber("rotation setpoint", rSetpoint);
    SmartDashboard.putNumber("rotation error", rError);
    SmartDashboard.putNumber("rotation PID", rotationPID.calculate(rError, 0));

    setSpeed();

    SmartDashboard.putNumber("gyroAngle", m_gyro.getRotation2d().getDegrees() * -1);
    fieldOrientation = SmartDashboard.getBoolean("Feild/Robot", true);

    SmartDashboard.putNumber("FL_SPEED", m_frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("RL_SPEED", m_rearLeftEncoder.getVelocity());
    SmartDashboard.putNumber("FR_SPEED", m_frontRightEncoder.getVelocity());
    SmartDashboard.putNumber("RR_SPEED", m_rearRightEncoder.getVelocity());

    //for testng
    KpSlider = SmartDashboard.getNumber("Kp value", KpSlider);
    SmartDashboard.putNumber("final Kp value", KpSlider);
    rotationPID.setP(KpSlider);
}
}