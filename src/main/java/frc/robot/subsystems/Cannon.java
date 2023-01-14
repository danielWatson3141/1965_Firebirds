// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;

public class Cannon extends SubsystemBase {

  public static final boolean DEFENSE_MODE = false;

  // Stoppers
  // Pneumatic cylinders which control pegs
  // 3 of them
  DoubleSolenoid piston;

  Compressor compressor;

  // // Creates a ping-response Ultrasonic object on DIO 0 and 1.
  // Ultrasonic ultrasonic1 = new Ultrasonic(0, 1);
//amongus
  // // Creates a ping-response Ultrasonic object on DIO 2 and 3.
  // Ultrasonic ultrasonic2 = new Ultrasonic(2, 3);

  // Motor
  private VictorSPX cannonMotor;

  /** Creates a new Cannon Subsystem. */
  public Cannon() {
    cannonMotor = new VictorSPX(10);

    cannonMotor.setInverted(true);

    piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    // Make the ultrasonic sensor always on
    // Ultrasonic.setAutomaticMode(true);
    Logging.log("canon", "initialized");
  }

  // Set belt on/off and direction
  // forward controls forward/back motion
  // on controls on/off

  private static final double BELT_SPEED = 1.0;
  public boolean beltEnabled = false;

  public void toggleBelt(boolean enabled) {
    beltEnabled = enabled;

    if (DEFENSE_MODE) {
      beltEnabled = false;
      enabled = false;
    }

    if (enabled) {
      cannonMotor.set(ControlMode.PercentOutput, BELT_SPEED);
      Logging.log("cannon", "belt enabled");
    } else {
      cannonMotor.set(ControlMode.PercentOutput, 0);
      Logging.log("cannon", "belt disabled");
    }
  }

  public void setBeltReverse() {

    if (DEFENSE_MODE)
      return;

    Logging.log("cannon", "belt reversed");
    cannonMotor.set(ControlMode.PercentOutput, -BELT_SPEED);
  }

  public void toggleBelt() {
    toggleBelt(!beltEnabled);

  }

  // Boolean determines position of the pegs
  // peg determines which peg (0,1,2)
  public void setPegToggle(boolean up) {

    DoubleSolenoid.Value direction = !up ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;
    piston.set(direction);
    if (up)
      Logging.log("cannon", "pegs up");
    else
      Logging.log("cannon", "pegs down");

    SmartDashboard.putBoolean("enable peg", up);
  }

  public void togglePeg() {
    boolean up = piston.get() == DoubleSolenoid.Value.kReverse;
    setPegToggle(!up);
  }

  static int pegNum = 1;

  public void testPegs() {
    setPegToggle(true);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    setPegToggle(false);
  }

  private static final double ULTRASONIC_DETECTION_RANGE_MM = 10;

  // ultra sensor detects balls within 5 inches
  public boolean isBallPresent(int slot) {

    // TODO: Check out this alternate implementation
    // return slot == 1 ?
    // ultrasonic1.getRangeMM() < 10 :
    // ultrasonic2.getRangeMM() < 10 ;

    return true;
  }
  // Check if proximity sensor (slot) is activated
  // Starts the ultrasonic sensor running in automatic mode
  // Creates a ping-response Ultrasonic object on DIO 1 and 2.

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("UltraSonic Sensor", ultrasonic1.getRangeMM());

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Object pegToggle() {
    return null;
  }
}
