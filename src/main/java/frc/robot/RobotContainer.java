// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.EjectBall;
import frc.robot.commands.GrabBalls;
import frc.robot.commands.RollAuto;
import frc.robot.subsystems.Cannon;
import frc.robot.subsystems.ClimbingArmHook;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SixWheelDrivetrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    private XboxController driverController = new XboxController(0);
    private XboxController coPilotController = new XboxController(1);

    private final SixWheelDrivetrain drivetrain = new SixWheelDrivetrain(driverController);
    private ClimbingArmHook arm = new ClimbingArmHook();
    private Intake intake = new Intake();
    private Cannon cannon = new Cannon();
    private Vision vision_system = new Vision();

    private JoystickButton coPilotBButton = new JoystickButton(coPilotController, XboxController.Button.kB.value);

    private JoystickButton aButton = new JoystickButton(driverController, XboxController.Button.kA.value);
    private JoystickButton bButton = new JoystickButton(driverController, XboxController.Button.kB.value);
    private JoystickButton xButton = new JoystickButton(driverController, XboxController.Button.kX.value);
    private JoystickButton yButton = new JoystickButton(driverController, XboxController.Button.kY.value);

    private JoystickButton lbButton = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    private JoystickButton rbButton = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);

    private JoystickButton backButton = new JoystickButton(driverController, XboxController.Button.kBack.value);
    private JoystickButton startButton = new JoystickButton(driverController, XboxController.Button.kStart.value);

    private JoystickButton leftStickButton = new JoystickButton(driverController, XboxController.Button.kLeftStick.value);
    private JoystickButton rightStickButton = new JoystickButton(driverController, XboxController.Button.kRightStick.value);


    // The container for the robot. Contains subsystems, OI devices, and commands.

    // The container for the robot. Contains subsystems, OI devices, and commands.

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
        Logging.log("robot container", "buttons configured");
        // rb and lb
        rbButton.onTrue(
                new InstantCommand(() -> arm.erectHook(), arm));

        lbButton.onTrue(
                new InstantCommand(() -> arm.retractHook(), arm));

        lbButton.onFalse(
                new InstantCommand(() -> arm.stopHook(), arm));

        rbButton.onFalse(
                new InstantCommand(() -> arm.stopHook(), arm));

        // // Y Button
        yButton.whileTrue(
                new EjectBall(cannon, intake));

        // A Button
        aButton.whileTrue(
                new GrabBalls(cannon, intake));

        // X Button
        xButton.onTrue(
                new InstantCommand(() -> vision_system.switchCamera()));

        // B Button
        bButton.onTrue(
                new InstantCommand(() -> cannon.togglePeg(), cannon));

        coPilotBButton.onTrue(
                new InstantCommand(() -> cannon.togglePeg(), cannon));

    }

    

    public void test() {
        // Logging.log("robot container", "testing mode");
        // test the hooks
        if (driverController.getLeftBumperPressed()) {
            arm.erectHook();
        }
        if (driverController.getRightBumperPressed()) {
            arm.retractHook();
        }
        if (driverController.getRightBumperReleased() ||
                driverController.getLeftBumperReleased()) {
            arm.stopHook();
        }

        // test the peg
        if (driverController.getXButtonPressed()) {
            cannon.setPegToggle(true);
        }
        if (driverController.getXButtonReleased()) {
            cannon.setPegToggle(false);
        }

        // test the belt
        if (driverController.getAButtonPressed()) {
            cannon.toggleBelt(true);
        }
        if (driverController.getAButtonReleased()) {
            cannon.toggleBelt(false);
        }
        // intake
        if (driverController.getBButtonPressed()) {
            intake.setSpinnerEnabled(true);
        }
        if (driverController.getBButtonReleased()) {
            intake.setSpinnerEnabled(false);
        }
        if (driverController.getYButtonPressed()) {
            intake.toggleSpinner();
        }

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new GrabBalls(cannon, intake).andThen(
                new WaitCommand(3).andThen(
                    new GrabBalls(cannon, intake).andThen(
                        new WaitCommand(6).raceWith(new RollAuto(drivetrain)))));
    }
}
