
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  public SixWheelDrivetrain() {
    // 2 groups of motors
    WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(3);
    WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(5);
    MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

    MotorController m_frontRight = new WPI_TalonSRX(4);
    MotorController m_rearRight = new WPI_TalonSRX(6);
    MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);
    m_right.setInverted(true);

    driver = new DifferentialDrive(m_left, m_right);

    myController = new XboxController(0);

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

    long currentTime = System.currentTimeMillis();

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

    if(Math.abs(targetSpeed - currentSpeed) < MAX_ACCEL){
      currentSpeed = targetSpeed;
    } else if(currentSpeed < targetSpeed){
      currentSpeed += MAX_ACCEL;
    } else {
      currentSpeed -= MAX_ACCEL;
    }

    driver.curvatureDrive(currentSpeed, steerOutput, quickturn);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
