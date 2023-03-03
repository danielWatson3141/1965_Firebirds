// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RollAuto;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.SixWheelDrivetrain;

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
    private final Lifter lifter = new Lifter(driverController);
    private final Claw claw = new Claw(driverController);

    //private Vision visionSystem = new Vision();

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
    NetworkTableEntry tx, ty, ta;

    // The container for the robot. Contains subsystems, OI devices, and commands.

    // The container for the robot. Contains subsystems, OI devices, and commands.

    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        drivetrain.setDefaultCommand(
                new RunCommand(
                        () -> drivetrain.drive(),
                        drivetrain));

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        

        //hold to open claw command on shuffleboard
        //individual open/shut commands on shuffleboard
        // SmartDashboard.putData("Unfurl Claw", new InstantCommand(() -> claw.clawOpen(), claw));
        // SmartDashboard.putData("UnUnfurl Claw", new InstantCommand(() -> claw.clawShut(), claw));

        // Send commands to dashboard
        //These will be displayed on the commands panel Ex from last year:
        // SmartDashboard.putData("Erect", new InstantCommand(() -> arm.erectHook(), arm));
        // SmartDashboard.putData("Retract", new InstantCommand(() -> arm.retractHook(), arm));
        // SmartDashboard.putData("Stop", new InstantCommand(() -> arm.stopHook(), arm));
        // SmartDashboard.putData("Switch",new InstantCommand(() -> switchCamera(), arm));
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
                new InstantCommand(() -> lifter.moveArmUp(), lifter));
        lbButton.onTrue(
                new InstantCommand(() -> lifter.moveArmDown(), lifter));
        // X Button
        xButton.onTrue(
                new InstantCommand(() -> switchCamera()));
        
        // a Button
        aButton.onTrue(
                new InstantCommand(() -> claw.clawToggle()));

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
       //ShuffleBoard widget commands for movement of arm
       SmartDashboard.putData("GoToTop", new InstantCommand(() -> lifter.goToTop(), lifter));
       SmartDashboard.putData("GoToMiddle", new InstantCommand(() -> lifter.goToMiddle(), lifter));
       SmartDashboard.putData("GoToBottom", new InstantCommand(() -> lifter.goToBottom(), lifter));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new RollAuto(drivetrain);
    }
 
}
