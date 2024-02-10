// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/** Represents a mecanum drive style drivetrain. */
public class PIDMecanum extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final CANSparkMax m_frontLeftMotor = new CANSparkMax(1,MotorType.kBrushless);
  private final CANSparkMax m_frontRightMotor = new CANSparkMax(3,MotorType.kBrushless);
  private final CANSparkMax m_backLeftMotor = new CANSparkMax(2,MotorType.kBrushless);
  private final CANSparkMax m_backRightMotor = new CANSparkMax(4,MotorType.kBrushless);

  private final RelativeEncoder m_frontLeftEncoder = m_frontLeftMotor.getEncoder();
  private final RelativeEncoder m_frontRightEncoder = m_frontRightMotor.getEncoder();
  private final RelativeEncoder m_rearLeftEncoder = m_backLeftMotor.getEncoder();
  private final RelativeEncoder m_rearRightEncoder = m_backRightMotor.getEncoder();


  private final Translation2d m_frontLeftLocation = new Translation2d(0.3, 0.26);
  private final Translation2d m_frontRightLocation = new Translation2d(0.3, -0.26);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.3, 0.26);
  private final Translation2d m_backRightLocation = new Translation2d(-0.3, -0.26);

  private final PIDController m_frontLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_frontRightPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backRightPIDController = new PIDController(1, 0, 0);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  public double driveSpeed;

  private final MecanumDriveKinematics m_kinematics =
      new MecanumDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(m_kinematics, m_gyro.getRotation2d(), getCurrentDistances());

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  Joystick m_stick;

  private final double VELOCITY_CONVERSION_FACTOR = 1/150;

  /** Constructs a MecanumDrive and resets the gyro. */
  public PIDMecanum(Joystick input_stick) {

    m_stick = input_stick;
    m_gyro.reset();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRightMotor.setInverted(true);
    m_backRightMotor.setInverted(true);

    m_frontLeftEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
    m_rearLeftEncoder.setPositionConversionFactor(VELOCITY_CONVERSION_FACTOR);
    m_frontRightEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
    m_rearRightEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
  }

  /**
   * Returns the current state of the drivetrain.
   *
   * @return The current state of the drivetrain.
   */
  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getVelocity(),
        m_frontRightEncoder.getVelocity(),
        m_rearLeftEncoder.getVelocity(),
        m_rearRightEncoder.getVelocity());
  }

  /**
   * Returns the current distances measured by the drivetrain.
   *
   * @return The current distances measured by the drivetrain.
   */
  public MecanumDriveWheelPositions getCurrentDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getPosition(),
        m_frontRightEncoder.getPosition(),
        m_rearLeftEncoder.getPosition(),
        m_rearRightEncoder.getPosition());
  }

  /**
   * Set the desired speeds for each wheel.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    final double frontLeftFeedforward = m_feedforward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = m_feedforward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedforward = m_feedforward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedforward = m_feedforward.calculate(speeds.rearRightMetersPerSecond);

    final double frontLeftOutput =
        m_frontLeftPIDController.calculate(
            m_frontLeftEncoder.getVelocity(), speeds.frontLeftMetersPerSecond);
    final double frontRightOutput =
        m_frontRightPIDController.calculate(
            m_frontRightEncoder.getVelocity(), speeds.frontRightMetersPerSecond);
    final double backLeftOutput =
        m_backLeftPIDController.calculate(
            m_rearLeftEncoder.getVelocity(), speeds.rearLeftMetersPerSecond);
    final double backRightOutput =
        m_backRightPIDController.calculate(
            m_rearRightEncoder.getVelocity(), speeds.rearRightMetersPerSecond);

    m_frontLeftMotor.setVoltage(frontLeftOutput + frontLeftFeedforward);
    m_frontRightMotor.setVoltage(frontRightOutput + frontRightFeedforward);
    m_backLeftMotor.setVoltage(backLeftOutput + backLeftFeedforward);
    m_backRightMotor.setVoltage(backRightOutput + backRightFeedforward);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive() {

    double xSpeed,  ySpeed,  rot,  periodSeconds;

    driveSpeed = ((-m_stick.getRawAxis(3) + 1) / 2);
    SmartDashboard.putNumber("Drive %", driveSpeed * 100);


    xSpeed = (m_stick.getX()) * driveSpeed;
    ySpeed = (-m_stick.getY()) * driveSpeed;
    rot = (m_stick.getZ()) * driveSpeed;

    boolean fieldRelative = SmartDashboard.getBoolean("Feild/Robot", true);;

    MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                Robot.kDefaultPeriod));
    mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), getCurrentDistances());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("gyroAngle", m_gyro.getRotation2d().getDegrees());

    SmartDashboard.putNumber("FL_SPEED", m_frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("RL_SPEED", m_rearLeftEncoder.getVelocity());
    SmartDashboard.putNumber("FR_SPEED", m_frontRightEncoder.getVelocity());
    SmartDashboard.putNumber("RR_SPEED", m_rearRightEncoder.getVelocity());
 
  }
}
