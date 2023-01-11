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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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

    UsbCamera camera1;
    UsbCamera camera2;
    NetworkTableEntry cameraSelection;

    // The container for the robot. Contains subsystems, OI devices, and commands.

    // The container for the robot. Contains subsystems, OI devices, and commands.

    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        camera1 = CameraServer.startAutomaticCapture(0);
        camera1.setResolution(300, 300);
        camera2 = CameraServer.startAutomaticCapture(1);
        camera2.setResolution(300, 300);

        cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

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
        rbButton.whenPressed(
                new InstantCommand(() -> arm.erectHook(), arm));

        lbButton.whenPressed(
                new InstantCommand(() -> arm.retractHook(), arm));

        lbButton.whenReleased(
                new InstantCommand(() -> arm.stopHook(), arm));

        rbButton.whenReleased(
                new InstantCommand(() -> arm.stopHook(), arm));

        // // Y Button
        yButton.whileActiveOnce(
                new EjectBall(cannon, intake));

        // A Button
        aButton.whileActiveOnce(
                new GrabBalls(cannon, intake));

        // X Button
        xButton.whenPressed(
                new InstantCommand(() -> switchCamera()));

        // B Button
        bButton.whenPressed(
                new InstantCommand(() -> cannon.togglePeg(), cannon));

        coPilotBButton.whenPressed(
                new InstantCommand(() -> cannon.togglePeg(), cannon));

    }

    boolean frontCamera = true;

    public void switchCamera() {
        if (frontCamera) {
            Logging.log("Camera", "Switching to camera 2");
            cameraSelection.setString(camera2.getName());
            frontCamera = false;
        } else {
            Logging.log("Camera", "Switching to camera 1");
            cameraSelection.setString(camera1.getName());
            frontCamera = true;
        }
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
