// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Cannon extends SubsystemBase {

  // Stoppers
  // Pneumatic cylinders
  DoubleSolenoid pistonArm1;
  DoubleSolenoid pistonArm2;
  DoubleSolenoid pistonArm3;
  Compressor compressor;

  // Proximity sensors

  // Motor
  private TalonSRX cannonMotor;

  /** Creates a new Cannon Subsystem. */
  public Cannon() {
    cannonMotor = new TalonSRX(8);

    pistonArm1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
    pistonArm2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);
    pistonArm3 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 6);
    compressor = new Compressor(null);
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
  public void setPegToggle(int pegNumber, boolean up) {
    DoubleSolenoid peg = (new DoubleSolenoid[]{pistonArm1, pistonArm2, pistonArm3})[pegNumber-1];

    DoubleSolenoid.Value direction = up ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kOff;

    peg.set(direction);    

  }

  // is a ball detected by proximity sensor
  // slot is which slot we're asking about (1,2)
  public boolean isBallPresent(int slot) {
    return false;
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
