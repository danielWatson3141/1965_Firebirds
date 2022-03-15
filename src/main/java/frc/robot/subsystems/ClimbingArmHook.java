// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ClimbingArmHook extends SubsystemBase {

  DoubleSolenoid piston;
  
  boolean pnumUp = false;
    
    public void pnumUpDown(){
      if (pnumUp = false) {
        pnumUp = true;
      }
      else {
        pnumUp = false;
      }
      
     
      //Pnumatic boolean.
      //if true extend solenoid
      //if false solenoid is off
      //make a if pressed changes value true
  
  
    }
    
  }


