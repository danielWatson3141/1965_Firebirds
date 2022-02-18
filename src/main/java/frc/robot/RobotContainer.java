// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DropBalls;
import frc.robot.commands.ExtendHook;
import frc.robot.commands.GrabBalls;
import frc.robot.commands.RetractHook;
import frc.robot.subsystems.Cannon;
import frc.robot.subsystems.ClimbingArmHook;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SixWheelDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SixWheelDrivetrain drivetrain = new SixWheelDrivetrain();
  private ClimbingArmHook arm = new ClimbingArmHook();
  private Intake intake = new Intake();

  private Cannon cannon = new Cannon();
  private XboxController myController = new XboxController(0);

  private JoystickButton aButton = new JoystickButton(myController, XboxController.Button.kA.value);
  private JoystickButton bButton = new JoystickButton(myController, XboxController.Button.kB.value);
  private JoystickButton xButton = new JoystickButton(myController, XboxController.Button.kX.value);
  private JoystickButton yButton = new JoystickButton(myController, XboxController.Button.kY.value);

  private JoystickButton lbButton = new JoystickButton(myController, XboxController.Button.kLeftBumper.value);
  private JoystickButton rbButton = new JoystickButton(myController, XboxController.Button.kRightBumper.value);

  private JoystickButton backButton = new JoystickButton(myController, XboxController.Button.kBack.value);
  private JoystickButton startButton = new JoystickButton(myController, XboxController.Button.kStart.value);

  private JoystickButton leftStickButton = new JoystickButton(myController, XboxController.Button.kLeftStick.value);
  private JoystickButton rightStickButton = new JoystickButton(myController, XboxController.Button.kRightStick.value);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> drivetrain.drive(),
            drivetrain));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //rb and lb
    rbButton.whenPressed(
        new ExtendHook(arm));
    lbButton.whenPressed(
        new RetractHook(arm));

    //B button
    bButton.whenPressed(    //TODO: Decide if these should be in their own file.
        new RunCommand(() -> intake.setSpinnerEnabled(true), intake));
    bButton.whenReleased(
        new RunCommand(() -> intake.setSpinnerEnabled(false), intake));

    //Y Button
    yButton.whenPressed(
        new RunCommand(() -> intake.dropSpinner(), intake));

    //A Button
    aButton.whileActiveOnce(
        new GrabBalls(cannon, intake));   
    
    //X Button
    xButton.whileActiveOnce(
        new DropBalls(cannon));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return null;
  }
}
