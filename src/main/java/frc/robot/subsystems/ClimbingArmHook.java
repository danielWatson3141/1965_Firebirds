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


// This function moves the climbing hook upwards to hook onto a bar, 
// but only when the limit switch on the top is not
// active. This is when the hook is not extended fully.
// It moves until the limit switch on the top is active and then stops
// moving. This is when the hook is extended fully. It stays like this until
// retractHook is called.
 public void erectHook(){  
   // requires: servo motor 1
  // If button pressed & top limit switch off
  // servo motor 1 moves positively
  // else
  // do nothing
  // check limit switches every second
  

 }
// This function moves the climbing hook downwards to pull a robot up on a bar, but 
//  only when the limit switch on the bottom is not
// active. This is when the hook is not retracted fully.
// It moves until the limit switch on the bottom is active and then stops
// moving. This is when the hook is retracted fully. It stays like this until
// extendHook is called.
 public void retractHook(){
   // requires: servo motor 1
  // If button pressed & bottom limit switch off
  // servo motor 1 moves negatively
  // else
  // do nothing
  // check limit switches every second
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
