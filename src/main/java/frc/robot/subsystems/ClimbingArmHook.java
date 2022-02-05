// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingArmHook extends SubsystemBase {

  // Stepper motor
  public TalonSRX lifterMotor;
  // limit switches

  // TODO: figure out the ports that we're using
  public DigitalInput toplimitSwitch = new DigitalInput(0);
  public DigitalInput bottomlimitSwitch = new DigitalInput(1);

  public static enum STATE {
    EXTENDED,
    RETRACTED,
    EXTENDING,
    RETRACTING
  }

  public STATE state = STATE.RETRACTED;

  // Creates an example subsystem
  public ClimbingArmHook() {
  }

  // This function moves the climbing hook upwards to hook onto a bar,
  // but only when the limit switch on the top is not
  // active. This is when the hook is not extended fully.
  // It moves until the limit switch on the top is active and then stops
  // moving. This is when the hook is extended fully. It stays like this until
  // retractHook is called.
  public void erectHook() {
    // if top limit active set motor to 0

    state=STATE.EXTENDING;

    // requires: motor 1
    // If button pressed & top limit switch off
    // motor 1 moves postively
    // else
    // do nothing
    // check limit switches every second

  }

  // This function moves the climbing hook downwards to pull a robot up on a bar,
  // but
  // only when the limit switch on the bottom is not
  // active. This is when the hook is not retracted fully.
  // It moves until the limit switch on the bottom is active and then stops
  // moving. This is when the hook is retracted fully. It stays like this until
  // extendHook is called.
  public void retractHook() {
    // if bottom limit switch is active motor set to 0
    state = STATE.RETRACTING;
    // requires: motor 1
    // If button pressed & bottom limit switch off
    // motor 1 moves negatively
    // else
    // do nothing
    // check limit switches every second
  }

  private static final double LIFTER_SPEED = .25;

  @Override
  public void periodic() {

    switch (state) {
      case EXTENDING:
        if (toplimitSwitch.get()) {
          lifterMotor.set(ControlMode.PercentOutput, 0);
          state=STATE.EXTENDED;
        } else {
          lifterMotor.set(ControlMode.PercentOutput, LIFTER_SPEED);
        }
        break;
      case RETRACTING:
        if (bottomlimitSwitch.get()) {
          lifterMotor.set(ControlMode.PercentOutput, 0);
          state=STATE.RETRACTED;
        } else {
          lifterMotor.set(ControlMode.PercentOutput, -LIFTER_SPEED);
        }
        break;
      default:
        break;
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
