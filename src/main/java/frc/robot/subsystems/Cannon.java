// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cannon extends SubsystemBase {

  // Stoppers
  // Pneumatic cylinders

  // Proximity sensors

  // Motor
  private TalonSRX cannonMotor;

  /** Creates a new Cannon Subsystem. */
  public Cannon() {
    cannonMotor = new TalonSRX(8);
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

  // is a ball detected by proximity sensor
  // slot is which slot we're asking about (1,2)
  public boolean isBallPresent(int slot) {
    // Check if proximity sensor (slot) is activated
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
