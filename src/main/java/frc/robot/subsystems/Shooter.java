package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {

    private double shooterSpeed = .5;

    private WPI_TalonSRX shooterMotor1 = new WPI_TalonSRX(8);
    private WPI_TalonSRX shooterMotor2 = new WPI_TalonSRX(6);
    private WPI_TalonSRX frontRoller = new WPI_TalonSRX(69);
    private WPI_TalonSRX midRoller = new WPI_TalonSRX(0);
    private DigitalInput top_limit_switch = new DigitalInput(0);
    private DigitalInput bottom_limit_switch = new DigitalInput(0);

    private double shooterTimerTest = 2;
    private double shooterSpeedTest = .22;
    private double m_frontRollerSpeed = 0;
    private double m_midRollerSpeed = 0;

    public Shooter() {
        Logging.log("Shooter:Shooter", "Constuctor");

        // dont need to set shooterMotor2 becasue it will follow what shooterMotor1 does
        shooterMotor2.follow(shooterMotor1);

        if (Robot.iron_man) {
            SmartDashboard.putNumber("Shooter Speed Slider", shooterSpeedTest);
            SmartDashboard.putNumber("Shooter Timer Slider", shooterTimerTest);
        }
    }

    public void frontRollerStart(double speed) {
        frontRoller.set(speed);
    }

    public void midRollerStart(double speed) {
        midRoller.set(speed);
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
    }

    public boolean getTopLimitSwitch() {
        return top_limit_switch.get();
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
        /*
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
        */
        return null;
    }

    public Command testShootRunCommand() {
        //Command r_command = new InstantCommand(() -> shooterSpinup());
        //r_command.addRequirements(this);
        //return r_command;
        return null;
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
    /*
    public void shooterMotorSet(double setSpeed) {
        shooterSpeed = setSpeed;
        double speedPercentage = shooterSpeed * 100;

        SmartDashboard.putNumber("Shooter %", speedPercentage);
    }
    */

    @Override
    public void periodic() {
        if (Robot.iron_man) {
            shooterSpeedTest = SmartDashboard.getNumber("Shooter Speed Slider", 0);
            shooterTimerTest = SmartDashboard.getNumber("Shooter Timer Slider", 0);
        }
        double intakeTimeoutSeconds = SmartDashboard.getNumber("intakeTimeoutSlider", 5); // todo
        m_frontRollerSpeed = SmartDashboard.getNumber("frontRollerSpeed", 5);
        m_midRollerSpeed = SmartDashboard.getNumber("midRollerSpeed", 5);
    }

}
