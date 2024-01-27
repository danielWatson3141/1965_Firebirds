package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

  private SlewRateLimiter rotationLimiter;
  private SlewRateLimiter throttleLimiterX;
  private SlewRateLimiter throttleLimiterY;

  // private double locationX = 0.2794;
  // private double locationY = 0.3048;

  Joystick m_stick;

  Rotation2d gyroAngle;

  CANSparkMax m_frontLeft;
  CANSparkMax m_rearLeft;
  CANSparkMax m_frontRight;
  CANSparkMax m_rearRight;

  private double rotationRate = 1;
  private double throttleRate = 1;
  private long driveAutoWait = 3000;

  MecanumDrive m_robotDrive;

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
    m_frontLeft = new CANSparkMax(1, MotorType.kBrushless);
    m_rearLeft =  new CANSparkMax(2, MotorType.kBrushless);
    m_frontRight =  new CANSparkMax(3, MotorType.kBrushless);
    m_rearRight =  new CANSparkMax(4, MotorType.kBrushless);

    m_robotDrive = new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);

    m_stick = input_stick;

    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);

    rotationLimiter = new SlewRateLimiter(rotationRate);
    throttleLimiterX = new SlewRateLimiter(throttleRate);
    throttleLimiterY = new SlewRateLimiter(throttleRate);
  }

  // multipliers for values
  final double SPEED_CAP = .6;
  private double driveSpeed;

  public void setSpeed() {
    // get percentage from the 4th axis and converts it from 0% - 100%
    double throttle_value = (-m_stick.getRawAxis(3) + 1) / 2;
    // sets the sped based on the cap and percentage
    driveSpeed = SPEED_CAP * throttle_value;
    // documents the current percentage of the motors for driver
    SmartDashboard.putNumber("Drive %", throttle_value * 100);
  }

  public Rotation2d gyroAngle() {
    return m_gyro.getRotation2d();
  }

  public void gyroReset() {
    m_gyro.reset();
    Logging.log(getSubsystem(), "reset gyroscope");
  }

  double autoSpeed = 0.5;

  public void driveAutoGo() {
    m_frontLeft.set(autoSpeed);
    m_rearLeft.set(autoSpeed);
    m_frontRight.set(autoSpeed);
    m_rearRight.set(autoSpeed);
  }

  public void driveAutoStop() {
    m_frontLeft.set(0);
    m_rearLeft.set(0);
    m_frontRight.set(0);
    m_rearRight.set(0);

  }

  public Command driveAutoCommand() {
    Command r_command = Commands.sequence(
        new InstantCommand(() -> driveAutoGo()),
        Commands.waitSeconds(driveAutoWait),
        new InstantCommand(() -> driveAutoStop()));

    r_command.addRequirements(this);
    return r_command;
  }

  public void drive() {

    double in_x = throttleLimiterX.calculate(m_stick.getX()) * driveSpeed;
    double in_y = throttleLimiterY.calculate(-m_stick.getY()) * driveSpeed;
    double in_z = rotationLimiter.calculate(m_stick.getZ()) * driveSpeed;
    SmartDashboard.putNumber("in_x", in_x);
    SmartDashboard.putNumber("in_y", in_y);
    SmartDashboard.putNumber("in_z", in_z);


    //Mecanum seems to consider 'X' the forward direction, so we're passing y, x, z on purpose.
    m_robotDrive.driveCartesian(
        in_y,
        in_x,
        in_z,
        m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("stickX", m_stick.getX());
    SmartDashboard.putNumber("stickY", m_stick.getY());
    SmartDashboard.putNumber("stickZ", m_stick.getZ());
    setSpeed();
    SmartDashboard.putNumber("gyroAngle", m_gyro.getRotation2d().getDegrees());
  }
}