// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.MecanumDrivetrain;
import frc.robot.subsystems.Shooter;

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
    private Joystick m_stick = new Joystick(0);

    private final MecanumDrivetrain m_drivetrain = new MecanumDrivetrain(m_stick);
    //private final PIDMecanum m_drivetrain = new PIDMecanum(m_stick);
    private final Shooter m_shooter = new Shooter();
    private final Lifter m_lifter = new Lifter();
   
    private JoystickButton triggerButton = new JoystickButton(m_stick, 1);
    private JoystickButton sideButton = new JoystickButton(m_stick, 2);
    private JoystickButton topRightButton = new JoystickButton(m_stick, 6);
    private JoystickButton bottomRightButton = new JoystickButton(m_stick, 4);
    private JoystickButton topLeftButton = new JoystickButton(m_stick, 5);
    private JoystickButton bottomLeftButton = new JoystickButton(m_stick, 3);
//10,12 uncomfort 8,11 can be bad dpepnds on grip : 9,7 easiest 
//most comfortable buttons
    private JoystickButton nineButton = new JoystickButton(m_stick, 9);
    private JoystickButton sevenButton = new JoystickButton(m_stick, 7);
//average comfortable buttons
    private JoystickButton eightButton = new JoystickButton(m_stick, 8);
    private JoystickButton elevenButton = new JoystickButton(m_stick, 11);
//least comfortable buttons
    private JoystickButton tenButton = new JoystickButton(m_stick, 10);
    private JoystickButton twelveButton = new JoystickButton(m_stick, 12);
//topJoy
    //private JoystickButton topStickButton = new JoystickButton(m_stick, );
//joystick tbd...
    //private JoystickButton leftStickButton = new JoystickButton(m_stick, );
    //private JoystickButton rightStickButton = new JoystickButton(m_stick,);

    UsbCamera camera1;
    UsbCamera camera2;
    NetworkTableEntry cameraSelection;
    NetworkTableEntry tx, ty, ta;

    double POVvalue;
    double POVspeed;

    // The container for the robot. Contains subsystems, OI devices, and commands.

    // The container for the robot. Contains subsystems, OI devices, and commands.

    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        POVspeed = m_drivetrain.driveSpeed;

        m_drivetrain.setDefaultCommand(
                new RunCommand(() -> m_drivetrain.drive(),
                        m_drivetrain));
        //camera1 = CameraServer.startAutomaticCapture(0);
        //camera1.setResolution(300, 300);
        //camera2 = CameraServer.startAutomaticCapture(1);
        //camera2.setResolution(100, 100);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        m_shooter.shooterCommands.add(new InstantCommand(() -> m_shooter.getShootCommand()));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.m_stick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Logging.log("robot container", "buttons configured");

        triggerButton.onTrue(        
            m_shooter.getShootCommand()
        );

        sevenButton.onTrue(
            new InstantCommand(() -> m_lifter.toggleLifter())
        );

        eightButton.onTrue(
            new InstantCommand(() -> m_drivetrain.gyroReset())
        );

         elevenButton.onTrue(
             m_shooter.testShootRunCommand()
         );

         twelveButton.onTrue(
             m_shooter.testShootStopCommand()
         );
    }


    public void configureTestButtonBindings() {
        tenButton.onTrue(getTestCommand());
        triggerButton.whileTrue(getTestCommand());
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

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    long DRIVE_TIME = 5;

    public Command getAutonomousCommand() {
        return m_drivetrain.driveAutoCommand();
        //return new RollAuto(drivetrain).withTimeout(DRIVE_TIME);
    }

    public Command getTestCommand () {
     

        return Commands.none();
    }

}
