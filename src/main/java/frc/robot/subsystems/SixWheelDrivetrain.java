
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SixWheelDrivetrain extends SubsystemBase {

  public DifferentialDrive driver;

  private XboxController myController;

  private SlewRateLimiter steeringLimiter;
  private SlewRateLimiter throttleLimiter;

  public static boolean driveOverride = false;

  WPI_TalonSRX m_blinkin = new WPI_TalonSRX(14);

  WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(3);
  WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(5);
  WPI_TalonSRX m_frontRight = new WPI_TalonSRX(4);
  WPI_TalonSRX m_rearRight = new WPI_TalonSRX(6);

  Accelerometer accelerometer = new BuiltInAccelerometer();

  /** Creates a new SixWheelDrivetrain. */
  public SixWheelDrivetrain(XboxController controller) {
    // 2 groups of motors

    MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);
    m_left.setInverted(true);
    m_frontLeft.setNeutralMode(NeutralMode.Coast);
    m_rearLeft.setNeutralMode(NeutralMode.Coast);

    MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);
    m_right.setInverted(false);
    m_frontRight.setNeutralMode(NeutralMode.Coast);
    m_rearRight.setNeutralMode(NeutralMode.Coast);

    driver = new DifferentialDrive(m_left, m_right);
    steeringLimiter = new SlewRateLimiter(2.5);
    throttleLimiter = new SlewRateLimiter(2.0);

    myController = controller;

    // imu = new ADIS16470_IMU();
  }

  private boolean BRAKE_ON = true;
  private boolean BRAKE_OFF = !BRAKE_ON;
  boolean brakeState;

  public void brakeToggle() {
    brakeState = !brakeState;

    if (brakeState == BRAKE_OFF) {
      m_frontLeft.setNeutralMode(NeutralMode.Coast);
      m_rearLeft.setNeutralMode(NeutralMode.Coast);
      m_frontRight.setNeutralMode(NeutralMode.Coast);
      m_rearRight.setNeutralMode(NeutralMode.Coast);
    } else {
      m_frontLeft.setNeutralMode(NeutralMode.Brake);
      m_rearLeft.setNeutralMode(NeutralMode.Brake);
      m_frontRight.setNeutralMode(NeutralMode.Brake);
      m_rearRight.setNeutralMode(NeutralMode.Brake);
    }
  }

  double xAccel;
  double yAccel;
  double zAccel;

  @Override
  public void periodic() {
    xAccel = accelerometer.getX();
    yAccel = accelerometer.getY();
    zAccel = accelerometer.getZ();

    SmartDashboard.putNumber("xAccel", xAccel);
    SmartDashboard.putNumber("yAccel", yAccel);
    SmartDashboard.putNumber("zAccel", zAccel);
  }

  double targetSpeed = 0;
  double currentSpeed = 0;

  final double MAX_ACCEL = .2;
  final double STEER_LIMIT_FACTOR = .28;

  public void drive() {
    if (driveOverride)
      return;

    // Logging.log("drivetrain", "driving");

    long currentTime = System.currentTimeMillis();
    SmartDashboard.putNumber("time", currentTime);

    double leftStickX = myController.getLeftX();
    double leftStickY = myController.getLeftY();

    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("leftStickX", leftStickX);
    // SmartDashboard.putNumber("leftStickY", leftStickY);
    SmartDashboard.putNumber("speed", currentSpeed);

    double rightTrigger = myController.getRightTriggerAxis();
    // SmartDashboard.putNumber("rightTrigger", rightTrigger);
    double leftTrigger = myController.getLeftTriggerAxis();
    // SmartDashboard.putNumber("leftTrigger", leftTrigger);
    // rt = backwards, lt = foward
    double throttle = rightTrigger + (-leftTrigger);

    SmartDashboard.putNumber("throttle", throttle);

    boolean quickturn = throttle < .05 && throttle > -.05;

    double steerLimit = -STEER_LIMIT_FACTOR * throttle + 1;

    if (quickturn) {
      steerLimit = .33;
    }

    SmartDashboard.putNumber("steerLimit", steerLimit);

    double steerInput = leftStickX;
    if (Math.abs(leftStickX) > steerLimit)
      steerInput = steerLimit * steerInput;

    double steerOutput = steeringLimiter.calculate((steerInput));
    targetSpeed = throttleLimiter.calculate(throttle);

    double blinkin_color = (Math.abs(targetSpeed * 1000 + 1000));

    m_blinkin.set(blinkin_color);

    SmartDashboard.putNumber("steerOutput", steerOutput * 100);

    // TODO Come back to this
    // if(Math.abs(targetSpeed - currentSpeed) < MAX_ACCEL){
    // currentSpeed = targetSpeed;
    // } else if(currentSpeed < targetSpeed){
    // currentSpeed += MAX_ACCEL;
    // } else {
    // currentSpeed -= MAX_ACCEL;
    // }
    // Logging.log("drivetrain", "ending");

    driver.curvatureDrive(-targetSpeed, steerOutput, quickturn);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void goAtSpeed(double speed) {
    System.out.println("GO at speed " + speed);
    driver.curvatureDrive(speed, 0, false);
    m_blinkin.set(1500);
  }

  public double tiltAngle;

  public double getTiltValue() {
    tiltAngle = Math.atan2(yAccel, xAccel);

    return tiltAngle; 

  }

}
