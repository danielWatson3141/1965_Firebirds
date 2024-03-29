package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private WPI_TalonSRX frontRoller = new WPI_TalonSRX(69);
    private WPI_TalonSRX rearRoller =  new WPI_TalonSRX(69);
    private WPI_TalonSRX indexMotor = new WPI_TalonSRX(69);

    private double motorSpeed;

    private boolean sensor1;
    private boolean sensor2;

    public double intakeTimeoutSeconds = 5;

    public final ShuffleboardTab intakeTab = Shuffleboard.getTab(getName());
    private GenericEntry intakeTimeoutEntry = intakeTab.add("Timeout Slider", intakeTimeoutSeconds)
        .withSize(2, 1) 
        .withPosition(0, 0)
    .getEntry();
    private GenericEntry noteLoadEntry = intakeTab.add("Note Load", false)
        .withSize(1, 1)
        .withPosition(0, 1)
    .getEntry();


    public Intake() {
        motorSpeed = 0.7;
        sensor1 = false;
        sensor2 = false;
    }

    //runs the roller motors
    public void runRoller() {
        frontRoller.set(motorSpeed);
        rearRoller.set(motorSpeed);
    }

    //tells us if the 1st sensor is covered or not
    public boolean sensor1State() {
        if (sensor1) {
            return true;
        } else {
            return false;
        }
    }

    //tells us if the 2nd sensor is covered or not
    public boolean sensor2State() {
        if (sensor2) {
            return true;
        } else {
            return false;
        }
    }

    //kills the roller motors
    public void stopRoller() {
        frontRoller.set(0);
        rearRoller.set(0);

    }

    //runs the index motor
    //kills the roller motors
    public void runIndex() {
        indexMotor.set(motorSpeed);
        stopRoller();
    }

    //places a smartdashboard message up telling the driver the note is ready to shoot
    //kills the index motor
    public void noteLoaded() {
        noteLoadEntry.setBoolean(sensor2 == true);
        stopIndex();
    }

    //kills the index motor
    public void stopIndex() {
        indexMotor.set(0);
    }

    //useless rn
    public void periodic() {
        intakeTimeoutSeconds = intakeTimeoutEntry.getDouble(intakeTimeoutSeconds);
    }

    /*creates the command; creates a timeout window 5 seconds on the command. starts running the
    rollers. if the 1st sensor does not get triggered within five seconds, it kills the sequence and 
    the rollers stop. if it gets triggered, the rollers stop, but the index motor starts. the index
    roller also has a timeout window of five seconds; if the 2nd sensor is not triggered within that
    time, the index cuts, and if it is triggered noteloaded sends a message to the dash saying that the
    note is loaded and then the index stops. r_command.addRequirements makes it so no other commands in 
    this subsytem can run while this is going on, and this whole sequence returns the command.
     */
    public Command getIntakeCommand() {
        Command r_command = new InstantCommand(() -> runRoller()).withTimeout(intakeTimeoutSeconds).until(this::sensor1State).andThen(

                new ConditionalCommand(
                        new InstantCommand(() -> runIndex()).withTimeout(intakeTimeoutSeconds).until(this::sensor2State).andThen(
                                new ConditionalCommand(
                                        new InstantCommand(() -> noteLoaded()),
                                        new InstantCommand(() -> stopIndex()),
                                        () -> sensor2State())),
                        new InstantCommand(() -> stopRoller()),
                        () -> sensor1State()));

        r_command.addRequirements(this);
        return r_command;
    }

}
