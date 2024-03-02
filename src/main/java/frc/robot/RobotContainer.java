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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.MecanumDrivetrain;

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
    private final IntakeShooter m_intakeshooter = new IntakeShooter(m_stick);
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

        //camera1 = CameraServer.startAutomaticCapture(0);
        //camera1.setResolution(300, 300);
        //camera2 = CameraServer.startAutomaticCapture(1);
        //camera2.setResolution(100, 100);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        // m_shooter.shooterCommands.add(new InstantCommand(() -> m_shooter.shooterMotorSet(.2)));
        // m_shooter.shooterCommands.add(new InstantCommand(() -> m_shooter.shooterMotorSet(.8)));
        // SmartDashboard.putData("lifter toggle", new InstantCommand(() -> m_lifter.toggleLifter()));
        //SmartDashboard.putData("activate shooter", new InstantCommand(() -> m_shooter.getShootCommand()));
        //SmartDashboard.putData("Slow Down Shooter", new InstantCommand(() -> m_shooter.shooterMotorSet(.2)));
        // SmartDashboard.putData("Speed Up Shooter", new InstantCommand(() -> m_shooter.shooterMotorSet(.8)));
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

        triggerButton.whileTrue(
            m_intakeshooter.getShootCommand()
        );

        sevenButton.onTrue(
            new InstantCommand(() -> m_lifter.toggleLifter())
        );

        eightButton.onTrue(
            new InstantCommand(() -> m_drivetrain.gyroReset())
        );

        elevenButton.onTrue(
            m_intakeshooter.getIntakeCommand()
        );

    }


    public void configureTestButtonBindings() {
        nineButton.onTrue(m_intakeshooter.testShootRunCommand());
        tenButton.onTrue(m_intakeshooter.testShootStopCommand());
        elevenButton.onTrue(m_intakeshooter.testIntakeRunCommand());
        twelveButton.onTrue(m_intakeshooter.testIntakeStopCommand());
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
    public void stopAutonomousCommand(){
        m_intakeshooter.stopIntakeSequence();
        m_intakeshooter.stopShooterSequence();
        m_drivetrain.driveAuto(0);

    }

    public Command getAutonomousCommand() {
        Command r_command = Commands.sequence(
            new RunCommand(() -> m_drivetrain.driveAuto(-0.2)).withTimeout(0.2),
            new RunCommand(() -> m_drivetrain.driveAuto(0.2)).withTimeout(0.2),
            m_intakeshooter.getShootCommand().withTimeout(3),
            new InstantCommand(() -> m_intakeshooter.runIntakeMotors(m_intakeshooter.INTAKE_SPEED)),
            Commands.race(new RunCommand(() -> m_drivetrain.driveAuto(0.2)).until(() -> m_intakeshooter.switch1State()), Commands.waitSeconds(3))
        );
         
        r_command = r_command.finallyDo(() -> stopAutonomousCommand());

        r_command.addRequirements(m_drivetrain);
        r_command.addRequirements(m_intakeshooter);

        return r_command;

        //return new RollAuto(drivetrain).withTimeout(DRIVE_TIME);
    }

    public Command getTestCommand () {
     

        return Commands.none();
    }

    public Command getTeleopCommand(){
        return new RunCommand(() -> m_drivetrain.drive() , m_drivetrain);
    }

}
