// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;

public class Intake extends SubsystemBase {

  DoubleSolenoid piston;

  boolean spinner_enabled = false;
  boolean spinner_dropped = false;

  // Motor
  private VictorSPX intakeMotor;

  /** Creates a new Intake. */
  public Intake() {
    // motor that extends arm
    intakeMotor = new VictorSPX(9);
    intakeMotor.setInverted(true);

    piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    Logging.log("intake", "initialized");
  }

  private static final double SPINNER_SPEED = -0.65;
  public boolean spinnerEnabled = false;

  // activates/deactivates the spinner based on enabled
  public void setSpinnerEnabled(boolean enabled) {
    spinnerEnabled = enabled;

    if (Cannon.DEFENSE_MODE) {
      spinnerEnabled = false;
      enabled = false;
    }

    if (enabled) {
      dropSpinner();
      intakeMotor.set(ControlMode.PercentOutput, SPINNER_SPEED);
      spinner_enabled = true;
      Logging.log("intake", "spinner enabled");
    } else {
      intakeMotor.set(ControlMode.PercentOutput, 0);
      spinner_enabled = false;
      // Logging.log("intake", "disabled");
    }
  }

  // drops the spinner
  // assumes spinner is up
  // does nothing otherwise

  public void dropSpinner() {

    if (Cannon.DEFENSE_MODE) {
      piston.set(DoubleSolenoid.Value.kReverse);
    } else{
      piston.set(DoubleSolenoid.Value.kForward);
      Logging.log("intake", "dropped spinner");
    }
    
  }

  public void raiseSpinner() {
    piston.set(DoubleSolenoid.Value.kReverse);
    Logging.log("intake", "raised spinner");
  }

  public void toggleSpinner() {
    boolean up = piston.get() == DoubleSolenoid.Value.kForward;
    if (up) {
      raiseSpinner();
    } else {
      dropSpinner();
    }
  }

  public void toggleRoller() {
    setSpinnerEnabled(!spinnerEnabled);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("setSpinnerEnabled", spinner_enabled);
    SmartDashboard.putBoolean("dropSpinner", spinner_dropped);
    // This function will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setSpinnerReverse() {
    
    if (Cannon.DEFENSE_MODE)
      return;
      intakeMotor.set(ControlMode.PercentOutput, -SPINNER_SPEED);
  }
}
