
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SixWheelDrivetrain extends SubsystemBase {

  // L and R encoders
  private Encoder lEncoder;
  private Encoder rEncoder;

  public DifferentialDrive driver;

  private XboxController myController;

  private ADIS16470_IMU imu;

  private SlewRateLimiter steeringLimiter;
  private SlewRateLimiter throttleLimiter;

  public static boolean driveOverride = false;

  WPI_TalonSRX m_blinkin = new WPI_TalonSRX(8);

  /** Creates a new SixWheelDrivetrain. */
  public SixWheelDrivetrain(XboxController controller) {
    // 2 groups of motors
    WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(3);
    WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(5);
    MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);
    m_left.setInverted(true);

    MotorController m_frontRight = new WPI_TalonSRX(4);
    MotorController m_rearRight = new WPI_TalonSRX(6);
    MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);
    m_right.setInverted(false);

    driver = new DifferentialDrive(m_left, m_right);
    steeringLimiter = new SlewRateLimiter(2.5);
    throttleLimiter = new SlewRateLimiter(1.3);

    myController = controller;

    //imu = new ADIS16470_IMU();
  }

  // radians / sec
  protected double rotationRate() {

    // double unitsPers = l2Talon.getSelectedSensorVelocity(0) * 10;
    // SmartDashboard.putNumber("RotationRate", unitsPers/unitsPerRadian);
    // TODO: Make in radians
    return 0; // unitsPerRadian;
  }

  @Override
  public void periodic() {

  }

  double targetSpeed = 0;
  double currentSpeed = 0;

  final double MAX_ACCEL = .2;
  final double STEER_LIMIT_FACTOR = .38;

  public void drive() {
    if(driveOverride)
      return;
    
   //Logging.log("drivetrain", "driving");

    long currentTime = System.currentTimeMillis();
    SmartDashboard.putNumber("time", currentTime);

    double leftStickX = myController.getLeftX();
    double leftStickY = myController.getLeftY();

    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("leftStickX", leftStickX);
    //SmartDashboard.putNumber("leftStickY", leftStickY);
    SmartDashboard.putNumber("speed", currentSpeed);

    double rightTrigger = myController.getRightTriggerAxis();
    //SmartDashboard.putNumber("rightTrigger", rightTrigger);
    double leftTrigger = myController.getLeftTriggerAxis();
    //SmartDashboard.putNumber("leftTrigger", leftTrigger);
//rt = backwards, lt = foward
    double throttle = rightTrigger + (-leftTrigger);

    SmartDashboard.putNumber("throttle", throttle);

    boolean quickturn = throttle < .05 && throttle > -.05;

    double steerLimit = -STEER_LIMIT_FACTOR * throttle + 1;

    if(quickturn){
      steerLimit = .43;
    }

    SmartDashboard.putNumber("steerLimit", steerLimit);

    double steerInput = leftStickX;
    if (Math.abs(leftStickX) > steerLimit)
      steerInput = steerLimit * steerInput;

    double steerOutput = steeringLimiter.calculate((steerInput));
    targetSpeed = throttleLimiter.calculate( throttle );
    
    double blinkin_color = targetSpeed * 1000 + 1000;

    m_blinkin.set(blinkin_color);

    SmartDashboard.putNumber("steerOutput", steerOutput);

    //TODO Come back to this
    // if(Math.abs(targetSpeed - currentSpeed) < MAX_ACCEL){
    //   currentSpeed = targetSpeed;
    // } else if(currentSpeed < targetSpeed){
    //   currentSpeed += MAX_ACCEL;
    // } else {
    //   currentSpeed -= MAX_ACCEL;
    // }
    //Logging.log("drivetrain", "ending");

    driver.curvatureDrive(-targetSpeed, steerOutput, quickturn);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void goAtSpeed(double speed){
    System.out.println("GO at speed "+speed);
    driver.curvatureDrive(speed, 0, false);
    m_blinkin.set(1500);
  }

}
