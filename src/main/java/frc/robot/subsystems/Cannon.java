// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cannon extends SubsystemBase {

  // Stoppers
  // Pneumatic cylinders

  // Creates a ping-response Ultrasonic object on DIO 1 and 2.
  Ultrasonic ultrasonic1 = new Ultrasonic(1, 2);
  Ultrasonic ultrasonic2 = new Ultrasonic(1, 2);
  // Motor
  private TalonSRX cannonMotor;

  /** Creates a new Cannon Subsystem. */
  public Cannon() {
    cannonMotor = new TalonSRX(8);
    Ultrasonic.setAutomaticMode(true);
  }

  // Set belt on/off and direction
  // forward controls forward/back motion
  // on controls on/off

  private static final double BELT_SPEED = 0.3;

  public void toggleBelt(boolean enabled) {
    if (enabled) {
      cannonMotor.set(ControlMode.PercentOutput, BELT_SPEED);
    } else {
      cannonMotor.set(ControlMode.PercentOutput, -0.15);
    }
  }

  // Boolean determines position of the pegs
  // peg determines which peg (1,2,3)
  public void setPegToggle(int peg, boolean up) {
    // Pegs are changed pneumatically
  }

  // ultra sensor detects balls within 5 inches
  public boolean isBallPresent(int slot) {
    if (slot == 1) {
      // "ultrasonic1" returns true or false from the boolean equation
      return (ultrasonic1.getRangeInches() < 5);
    } else if (slot == 2) {
      // "ultrasonic2" returns true or false from the boolean equation
      return (ultrasonic2.getRangeInches() < 5);
    } else {

      return false;
      // error condition when a slot doesnt exist
    }
  }
  // Check if proximity sensor (slot) is activated
  // Starts the ultrasonic sensor running in automatic mode
  // Creates a ping-response Ultrasonic object on DIO 1 and 2.

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
