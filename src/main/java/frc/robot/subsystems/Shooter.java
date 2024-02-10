package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {

    private double shooterSpeed = .5;

    private WPI_TalonSRX shooterMotor1;
    private WPI_TalonSRX shooterMotor2;
    private WPI_TalonSRX canMotor;

    private double shooterTimerTest = 2;
    private double shooterSpeedTest = .22;

    public final ShuffleboardTab shooterTab = Shuffleboard.getTab(getName());
    public final ShuffleboardLayout shooterCommands = shooterTab.getLayout("commands", BuiltInLayouts.kList)
        .withSize(1, 2).withPosition(1, 0);
    public final ShuffleboardLayout shooterSliders = shooterTab.getLayout("Sliders", BuiltInLayouts.kList)
        .withSize(1, 2).withPosition(0, 0);
    private GenericEntry testShooterSpeedSlider = shooterSliders.add("Speed Slider", shooterSpeedTest)
        .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
    private GenericEntry finalSpeedTest = shooterTab.add("final value", shooterSpeedTest)
        .withSize(2, 1).withPosition(2, 0)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
    private GenericEntry testShooterTimer = shooterSliders.add("Timer Slider", shooterTimerTest)
        .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
    private GenericEntry shooterSpeedPercent = shooterTab.add("Shooter Speed %", 0)
        .withSize(2, 1).withPosition(2, 1)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "max", 100))
    .getEntry();

    public Shooter() {
        Logging.log("Shooter:Shooter", "Constuctor");

        // sets motors to port number
        shooterMotor1 = new WPI_TalonSRX(8);
        shooterMotor2 = new WPI_TalonSRX(6);
        canMotor = new WPI_TalonSRX(7);
    }

    // sets shooting motor's voltage to the variable
    public void shooterSpinup() {
        Logging.log(getSubsystem(), "Shooter Speed is " + shooterSpeed);
        shooterMotor1.set(shooterSpeed);
        shooterMotor2.set(shooterSpeed);
    }

    // sets the can motor's votage to the variable
    public void startCan() {
        canMotor.set(shooterSpeed);
    }

    // stops all motors
    public void shooterStop() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
        canMotor.set(0);
    }

    public Command getShootCommand() {
        Command r_command = Commands.sequence(
                // Starts up the shooter motors
                new InstantCommand(() -> shooterSpinup()),
                // waits for variable miliseconds
                Commands.waitSeconds(shooterTimerTest),
                // Starts can motor
                new InstantCommand(() -> startCan()),
                // wait for variable miliseconds
                Commands.waitSeconds(shooterTimerTest),
                // stop all motors
                new InstantCommand(() -> shooterStop()));
        r_command.addRequirements(this);
        return r_command;
    }

    public Command testShootRunCommand() {
        Command r_command = new InstantCommand(() -> shooterSpinup());
        r_command.addRequirements(this);
        return r_command;
    }

    public Command testShootStopCommand() {
        Command r_command = new InstantCommand(() -> shooterStop());
        r_command.addRequirements(this);
        return r_command;
    }

    /*
     * VERY IMPORTANT (\0_0)\
     * This function works by calling the function and putting the desired speed
     * into the requirements.
     * It will not work if you do "motorSet()"
     * It HAS to have a value like "motorSet(.2)"
     */
    public void shooterMotorSet(double setSpeed) {
        shooterSpeed = setSpeed;
        double speedPercentage = shooterSpeed * 100;

        shooterSpeedPercent.setDouble(speedPercentage);
    }

    @Override
    public void periodic() {
        finalSpeedTest.setDouble(testShooterSpeedSlider.getDouble(shooterSpeedTest));
        shooterSpeedTest = testShooterSpeedSlider.getDouble(shooterSpeedTest);
        shooterTimerTest = testShooterTimer.getDouble(shooterTimerTest);
    }
}
