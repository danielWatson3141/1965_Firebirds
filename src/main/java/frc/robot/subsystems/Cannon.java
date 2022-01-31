// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cannon extends SubsystemBase {

  // Stoppers
  // Pneumatic cylinders

  // Proximity sensors

  // Motor
  private TalonSRX beltDriveTalon;

  /** Creates a new Cannon Subsystem. */
  public Cannon() {
  }
  

  public void beltController(){
    //Requires: TalonSRX
    
  }

  //public void beltBackwards(){

  //}

  //public void beltOff(){
    //
  //}

  public void setPegToggle(int peg, boolean up){
   //Proximity sensor, sensors the balls in the basket.
   //The dectected balls are then collected into 2 spots
   
//The code below is complete hell and we need to rethink and go over with the cannon team how they want the pegs to work and how to implement it into out code.
//This code sorta works but has many problems and questions that need to be answered. (proximity2=top) (proximity1=bottom)
      //if proximity2 == 1 "peg 2 and peg 3 raise"
         //else proximity2 == 0 "no pegs raise"
    
      //if proximity1 == 1 "peg 1 raises"
          //else proximity1 == 0 "no pegs raise"
  
  
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
