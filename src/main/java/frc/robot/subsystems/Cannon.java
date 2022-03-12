// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import javax.swing.text.WrappedPlainView;

public class Cannon extends SubsystemBase {

  // Stoppers
  // Pneumatic cylinders which control pegs
  // 3 of them
  DoubleSolenoid piston;

  Compressor compressor;

  // Motor
  private VictorSPX cannonMotor;

  /** Creates a new Cannon Subsystem. */
  public Cannon() {
    cannonMotor = new VictorSPX(10);

    cannonMotor.setInverted(true);

    piston=new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    Logging.log("canon", "initialized");
  }

  // Set belt on/off and direction
  // forward controls forward/back motion
  // on controls on/off

  private static final double BELT_SPEED = 0.3;

  public void toggleBelt(boolean enabled) {
    
    if (enabled) {
      cannonMotor.set(ControlMode.PercentOutput, BELT_SPEED);
      Logging.log("cannon", "belt enabled");
    } else {
      cannonMotor.set(ControlMode.PercentOutput, 0);
      Logging.log("cannon", "belt disabled");
    }
  }

  // Boolean determines position of the pegs
  // peg determines which peg (0,1,2)
  public void setPegToggle(boolean up) {

    DoubleSolenoid.Value direction = up ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;
    piston.set(direction);
    if(up)
      Logging.log("cannon", "pegs up");
    else
      Logging.log("cannon", "pegs down");
  }

  public void togglePeg(){
    boolean up = piston.get() == DoubleSolenoid.Value.kForward;
    setPegToggle(!up);
  }

  static int pegNum = 1;

  public void testPegs(){
    setPegToggle(true);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    setPegToggle(false);
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
