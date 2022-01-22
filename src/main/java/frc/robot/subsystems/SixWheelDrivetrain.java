
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

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SixWheelDrivetrain extends SubsystemBase {

  // These are the two motors characteristic of a treadbot
  private TalonSRX l1Talon;
  private TalonSRX r1Talon;
  private TalonSRX l2Talon;
  private TalonSRX r2Talon;

  public DifferentialDrive driver;

  private XboxController myController;

  /** Creates a new ExampleSubsystem. */
  public SixWheelDrivetrain() {
    WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(1);
    WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(2);

    MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

    MotorController m_frontRight = new WPI_TalonSRX(3);
    MotorController m_rearRight = new WPI_TalonSRX(4);
    MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);

    driver = new DifferentialDrive(m_left, m_right);

    myController = new XboxController(0);
  }

  public void setWheelSpeed(double leftSpeed, double rightSpeed) {
    l1Talon.set(ControlMode.PercentOutput, leftSpeed);
    l2Talon.set(ControlMode.PercentOutput, leftSpeed);
    r1Talon.set(ControlMode.PercentOutput, rightSpeed);
    r2Talon.set(ControlMode.PercentOutput, rightSpeed);
  }

  // radians / sec
  protected double rotationRate() {

    double unitsPers = l2Talon.getSelectedSensorVelocity(0) * 10;
    // SmartDashboard.putNumber("RotationRate", unitsPers/unitsPerRadian);
    // TODO: Make in radians
    return unitsPers; // unitsPerRadian;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("leftStickX", myController.getRawAxis(0));
    SmartDashboard.putNumber("leftStickY", myController.getRawAxis(1));

    long currentTime = System.currentTimeMillis();

    double leftStickX = myController.getRawAxis(0);
    double leftStickY = myController.getRawAxis(1);

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

    driver.curvatureDrive(rightTrigger, steerOutput, quickturn);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
