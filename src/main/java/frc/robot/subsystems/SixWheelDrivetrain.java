
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;

import java.security.Timestamp;
import java.sql.Time;
import java.text.SimpleDateFormat;
import java.util.Date;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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

  /** Creates a new SixWheelDrivetrain. */
  public SixWheelDrivetrain(XboxController cont) {
    // 2 groups of motors
    WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(6);
    WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(4);
    MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

    MotorController m_frontRight = new WPI_TalonSRX(5);
    MotorController m_rearRight = new WPI_TalonSRX(3);
    MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);
    m_right.setInverted(true);

    driver = new DifferentialDrive(m_left, m_right);

    myController = cont;

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

  public void drive() {

    Logging.log("drivetrain", "starting");

    long currentTime = System.currentTimeMillis();
    SmartDashboard.putNumber("time", currentTime);

    double leftStickX = myController.getLeftX();
    double leftStickY = myController.getLeftY();

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("leftStickX", leftStickX);
    SmartDashboard.putNumber("leftStickY", leftStickY);

    double rightTrigger = myController.getRightTriggerAxis();
    SmartDashboard.putNumber("rightTrigger", rightTrigger);
    double leftTrigger = myController.getLeftTriggerAxis();
    SmartDashboard.putNumber("leftTrigger", leftTrigger);

    double throttle = rightTrigger + (-leftTrigger);

    SmartDashboard.putNumber("throttle", throttle);

    boolean quickturn = throttle < .05 && throttle > -.05;

    double steerLimit = -0.66 * throttle + 1;

    SmartDashboard.putNumber("steerLimit", steerLimit);

    double steerOutput = leftStickX;
    if (leftStickX > steerLimit)
      steerOutput = steerLimit;

    targetSpeed = throttle;

    //TODO Come back to this
    // if(Math.abs(targetSpeed - currentSpeed) < MAX_ACCEL){
    //   currentSpeed = targetSpeed;
    // } else if(currentSpeed < targetSpeed){
    //   currentSpeed += MAX_ACCEL;
    // } else {
    //   currentSpeed -= MAX_ACCEL;
    // }
    Logging.log("drivetrain", "ending");

    driver.curvatureDrive(targetSpeed, steerOutput, quickturn);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
