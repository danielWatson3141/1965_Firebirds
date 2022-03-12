// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbingArmHook extends SubsystemBase {

  // Stepper motor
  public WPI_TalonSRX lifterMotor1;
  public WPI_TalonSRX lifterMotor2;
  public MotorControllerGroup lifterMotor;
  // limit switches

  // TODO: figure out the sensor ports that we're using
  public DigitalInput topleftlimitSwitch = new DigitalInput(0);
  public DigitalInput bottomleftlimitSwitch = new DigitalInput(3);
  public DigitalInput toprightlimitSwitch = new DigitalInput(1);
  public DigitalInput bottomrightlimitSwitch = new DigitalInput(4);

  private Hook leftHook;
  private Hook rightHook;

  public static enum STATE {
    EXTENDED, // Fully extended, top limit switch on, motor speed = 0
    RETRACTED, // Fully retracted, bottom limit switch on, motor speed = 0
    EXTENDING, // Motor moving in a positive direction, top limit switch not on, motor speed = 0.25
    RETRACTING // Motor moving in a negative direction, bottom limit switch not on, motor speed = -0.25
  }

  public STATE state = STATE.RETRACTED;

  // Creates an example subsystem
  public ClimbingArmHook() {
    lifterMotor1 = new WPI_TalonSRX(7);
    lifterMotor2 = new WPI_TalonSRX(8);
    lifterMotor = new MotorControllerGroup(lifterMotor1, lifterMotor2);

    leftHook = new Hook(lifterMotor1, topleftlimitSwitch, bottomleftlimitSwitch);
    rightHook = new Hook(lifterMotor2, toprightlimitSwitch, bottomrightlimitSwitch);

  }

  // This function moves the climbing hook upwards to hook onto a bar,
  // but only when the limit switch on the top is not
  // active. This is when the hook is not extended fully.
  // It moves until the limit switch on the top is active and then stops
  // moving. This is when the hook is extended fully. It stays like this until
  // retractHook is called.
  public void erectHook() {
    Logging.log("hooks","Extending");
    leftHook.extend();
    rightHook.extend();
  }

  // This function moves the climbing hook downwards to pull a robot up on a bar,
  // but
  // only when the limit switch on the bottom is not
  // active. This is when the hook is not retracted fully.
  // It moves until the limit switch on the bottom is active and then stops
  // moving. This is when the hook is retracted fully. It stays like this until
  // extendHook is called.
  public void retractHook() {
    Logging.log("hooks","Extending");
    leftHook.retract();
    rightHook.retract();
  }

  public void raiseHook(){
    lifterMotor.set(LIFTER_SPEED);
  }
  
  public void lowerHook(){
    lifterMotor.set(-LIFTER_SPEED);
  }

  public void stopHook(){
    lifterMotor.set(0);
  }

  private static final double LIFTER_SPEED = .25;
  // defines LIFTER_SPEED


  // checks limit switches every second if case is extending
  // if top limit switch is on and case = extending
  // change case to extended
  // if top limit switch is off and case = extending
  // continue to move at LIFTER_SPEED (0.25)

  final String stateKey = "state";

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Motor Speed", LIFTER_SPEED);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private class Hook{

    private final double MOTOR_SPEED = .25;

    MotorController motor;
    DigitalInput top, bottom;

    public Hook(MotorController motorController, DigitalInput topSwitch, DigitalInput bottomSwitch){
      motor = motorController;
      top = topSwitch;
      bottom = bottomSwitch;
    }

    public void extend(){
      motor.set(MOTOR_SPEED);

      while(!top.get());

      motor.set(0);
    
    }
    public void retract(){
      motor.set(-MOTOR_SPEED);

      while(!bottom.get());

      motor.set(0);
    }
  }

}
