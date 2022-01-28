// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //Extends the robots arm to push the button in front of it.
  //If the arm is already extended, this does nothing
  //If the arm is retracted, it extends
  //Once this function is called, the arm stays extended until retractArm is called
  //The arm takes 1 second to retract

  public void extendArm(){
    //Requires: Servo motor 1 and 2, and proximity sensor 1

    //if proximity sensor detects button && arm is retracted
    
      //move servo motor 1 to extended position

      //move servo motor 2 to extended position

    //else
      //do nothing 
  }

  public void retratArm(){
    //Requires: Servo motor 1 and 2
    
    //move servo motor 1 to retracted position

    //move servo motor 2 to retracted position
  }
}
