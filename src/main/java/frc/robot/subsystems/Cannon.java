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
  

  public void turnOnBelt(){
        //Turns belt on to drop balls
  }

  public void turnOffBelt(){

  }

  public void setPegActive(int peg, boolean up){
    
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
