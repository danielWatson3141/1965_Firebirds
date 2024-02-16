package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeShooter extends SubsystemBase {

    TalonSRX rollerMotor = new WPI_TalonSRX(9);

    TalonSRX indexMotor = new WPI_TalonSRX(11);

    double INTAKE_TIMEOUT;
    double INTAKE_EJECT_TIMEOUT;
    double INTAKE_SPEED;

    DigitalInput limitSwitch1 = new DigitalInput(0);
    DigitalInput limitSwitch2 = new DigitalInput(1);

    TalonSRX shooterMotor1 = new WPI_TalonSRX(8);
    TalonSRX shooterMotor2 = new WPI_TalonSRX(6);

    double SHOOTER_TIMER;
    double SHOOTER_SPEED;

    boolean shooterMode;


    public IntakeShooter() {

        shooterMotor2.follow(shooterMotor1);
        SHOOTER_TIMER = 1700;
        SHOOTER_SPEED = .8;
        shooterMode = true;

        INTAKE_TIMEOUT = 7000;
        INTAKE_EJECT_TIMEOUT = 3000;
        INTAKE_SPEED = 0.6;

        indexMotor.follow(rollerMotor);
    }

    public boolean switch1State() {
        if (limitSwitch1 != null) {
            return true;
        } else {
            return false;
        }
    }

    public boolean switch2State() {
        if (limitSwitch1 != null) {
            return true;
        } else {
            return false;
        }
    }

    public void setShooterMode() {
        shooterMode = !shooterMode;

        if (shooterMode) {
            SHOOTER_SPEED = .8;
        } else {
            SHOOTER_SPEED = .22;
        }
    }

    public void setIntakeMotors(double speed){
        rollerMotor.set(ControlMode.Current, speed);
    }

    public void setShooterMotors(double speed){
        shooterMotor1.set(ControlMode.Current, speed);
    }

     public void setIndexMotor(double speed){
        indexMotor.set(ControlMode.Current, speed);
    }

    public Command getShootCommand() {
        Command r_command = Commands.sequence(
            new InstantCommand(() -> setShooterMotors(SHOOTER_SPEED)),
            Commands.waitSeconds(SHOOTER_TIMER),
            new InstantCommand(() -> setIndexMotor(SHOOTER_SPEED)),
            Commands.waitSeconds(SHOOTER_TIMER),
            new InstantCommand(() -> setIndexMotor(0)).andThen(new InstantCommand(() -> setShooterMotors(0)))
        );

        r_command.addRequirements(this);
        return r_command;
    }

    public Command getIntakeCommand() {
        Command r_command = (
           new InstantCommand(() -> setIntakeMotors(INTAKE_SPEED)).withTimeout(INTAKE_TIMEOUT).until(this::switch2State)
           .andThen(new ConditionalCommand(new InstantCommand(() -> setIntakeMotors(0))/*log capacity*/, new InstantCommand(() -> setIntakeMotors(0)), this::switch2State))
           .andThen(new ConditionalCommand(new InstantCommand(() -> setIntakeMotors(-INTAKE_SPEED)).withTimeout(INTAKE_EJECT_TIMEOUT), new InstantCommand(() -> setIntakeMotors(0)), this::switch1State))

        );

        r_command.addRequirements(this);
        return r_command;
    }



    public void periodic() {


    }

}

//new ConditionalCommand(new InstantCommand(() -> setIntakeMotors(0)),  )