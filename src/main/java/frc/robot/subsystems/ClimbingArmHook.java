// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;

public class ClimbingArmHook extends SubsystemBase {

  private DoubleSolenoid leftSideSol, rightSideSol;

  // Creates an example subsystem
  public ClimbingArmHook() {
    leftSideSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    rightSideSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
  }

  // This function moves the climbing hook upwards to hook onto a bar,
  // but only when the limit switch on the top is not
  // active. This is when the hook is not extended fully.
  // It moves until the limit switch on the top is active and then stops
  // moving. This is when the hook is extended fully. It stays like this until
  // retractHook is called.
  public void erectHook() {
    Logging.log("hooks", "Extending");
    leftSideSol.set(DoubleSolenoid.Value.kForward);
    rightSideSol.set(DoubleSolenoid.Value.kForward);
  }

  // This function moves the climbing hook downwards to pull a robot up on a bar,
  // but
  // only when the limit switch on the bottom is not
  // active. This is when the hook is not retracted fully.
  // It moves until the limit switch on the bottom is active and then stops
  // moving. This is when the hook is retracted fully. It stays like this until
  // extendHook is called.
  public void retractHook() {
    Logging.log("hooks", "Retracting");
    leftSideSol.set(DoubleSolenoid.Value.kReverse);
    rightSideSol.set(DoubleSolenoid.Value.kReverse);
  }

  public void stopHook() {
    Logging.log("hooks", "Stopping");
    leftSideSol.set(DoubleSolenoid.Value.kOff);
    rightSideSol.set(DoubleSolenoid.Value.kOff);

  }

  final String stateKey = "state";

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Motor Speed", LIFTER_SPEED);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
