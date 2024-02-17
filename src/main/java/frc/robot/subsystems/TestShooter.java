package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
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

public class TestShooter extends SubsystemBase {

    private WPI_TalonSRX shooterMotor1 = new WPI_TalonSRX(8);
    private WPI_TalonSRX shooterMotor2 = new WPI_TalonSRX(6);
    private WPI_TalonSRX frontRoller = new WPI_TalonSRX(9);
    private WPI_TalonSRX midRoller = new WPI_TalonSRX(0);
    private DigitalInput top_limit_switch = new DigitalInput(25);
    private DigitalInput bottom_limit_switch = new DigitalInput(28);

    private double shooterTimerTest = 2;
    private double shooterSpeedTest = .22;
    private double m_frontRollerSpeed = 0;
    private double m_midRollerSpeed = 0;

    public final ShuffleboardTab shooterTab = Shuffleboard.getTab(getName());
    public final ShuffleboardLayout shooterCommands = shooterTab.getLayout("Commands", BuiltInLayouts.kList)
        .withSize(1, 2).withPosition(1, 0);
    public final ShuffleboardLayout shooterSliders = shooterTab.getLayout("Sliders", BuiltInLayouts.kList)
        .withSize(1, 2).withPosition(0, 0);
    private final GenericEntry testShooterSpeedSlider = shooterSliders.add("Speed Slider", shooterSpeedTest)
        .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
    private final GenericEntry finalSpeedTest = shooterTab.add("final value", shooterSpeedTest)
        .withSize(2, 1).withPosition(2, 0)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
    private final GenericEntry testShooterTimer = shooterSliders.add("Timer Slider", shooterTimerTest)
        .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
    private final GenericEntry shooterSpeedPercent = shooterTab.add("Shooter Speed %", 0)
        .withSize(2, 1).withPosition(2, 1)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "max", 100))
        .getEntry();

    public TestShooter() {
        Logging.log("Shooter:Shooter", "Constuctor");

        // dont need to set shooterMotor2 becasue it will follow what shooterMotor1 does
        shooterMotor2.follow(shooterMotor1);
    }

    public void frontRollerStart(double speed) {
        frontRoller.set(speed);
    }

    public void midRollerStart(double speed) {
        midRoller.set(speed);
    }
    public void frontStartFunc() {
        Logging.log(getName()+":shooterStartFunc", "Speed = " + shooterSpeedTest);
        frontRollerStart(shooterSpeedTest);
    }
    public void shooterStart(double speed) {
        shooterMotor1.set(speed);
    }

    public void frontRollerStop() {
        frontRoller.stopMotor();
    }

    public void midRollerStop() {
        midRoller.stopMotor();
    }

    public void shooterStop() {
        shooterMotor1.stopMotor();
        setShooterMotor(0.0);
    }

    public boolean getTopLimitSwitch() {
        return top_limit_switch.get();
    }

    public boolean getBottomLimitSwitch() {
        return bottom_limit_switch.get();
    }

    public void Startstate1() {
        frontRollerStart(m_frontRollerSpeed);
        midRollerStart(m_midRollerSpeed);
    }

    public void Stopstate1() {
        frontRollerStop();
        midRollerStop();
    }

    public Command getShootCommand() {
        Command r_command = Commands.sequence(
                // Starts up the shooter motors
                new InstantCommand(() -> frontStartFunc()),
                // waits for variable miliseconds
                Commands.waitSeconds(shooterTimerTest),
                // Starts can motor
                // new InstantCommand(() -> startCan()),
                // wait for variable miliseconds
                Commands.waitSeconds(shooterTimerTest),
                // stop all motors
                new InstantCommand(() -> shooterStop()));
        r_command.addRequirements(this);
        return r_command;
    }

    public Command testShootRunCommand() {
        Command r_command = new InstantCommand(() -> frontStartFunc());
        r_command.addRequirements(this);
        return r_command;
        
    }

    public Command testShootStopCommand() {
        Command r_command = new InstantCommand(() -> frontRollerStop());
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
    public void setShooterMotor(double speed) {
        double speedPercentage = speed * 100;
        shooterSpeedPercent.setDouble(speedPercentage);
    }

    @Override
    public void periodic() {
        finalSpeedTest.setDouble(testShooterSpeedSlider.getDouble(shooterSpeedTest));
        shooterSpeedTest = testShooterSpeedSlider.getDouble(shooterSpeedTest);
        shooterTimerTest = testShooterTimer.getDouble(shooterTimerTest);
        m_frontRollerSpeed = SmartDashboard.getNumber("frontRollerSpeed", 5);
        m_midRollerSpeed = SmartDashboard.getNumber("midRollerSpeed", 5);
    }

}
