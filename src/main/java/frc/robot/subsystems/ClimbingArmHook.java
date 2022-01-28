// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingArmHook extends SubsystemBase {

    //Stepper motor
    //limit switch
 
    //Creates an example subsystem
  public ClimbingArmHook() {}


// This function moves the climbing hook upwards, but only when the limit switch on the top is not
// active. This is when the hook is not extended fully.
// It moves until the limit switch on the top is active and then stops
// moving. This is when the hook is extended fully.
 public void erectHook(){
// The function moves a motor to move a chain positively, which only raises the hook if the top limit switch isn't active. 
// When the top limit switch becomes active, the motor and chain stop moving.
 }
// This function moves the climbing hook downwards, but only when the limit switch on the bottom is not
// active. This is when the hook is not retracted fully.
// It moves until the limit switch on the bottom is active and then stops
// moving. This is when the hook is retracted fully.
 public void retractHook(){
  // The function moves a motor to move a chain negatively, which only lowers/*  */ the hook if the bottom limit switch isn't active. 
 // When the top limit switch becomes active, the motor and chain stop moving.
 }

 
 
 
 
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
