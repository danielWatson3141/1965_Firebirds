package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeShooter extends SubsystemBase {

    TalonSRX rollerMotor = new WPI_TalonSRX(69);

    TalonSRX indexMotor = new WPI_TalonSRX(69);

    TalonSRX shooterMotor1 = new WPI_TalonSRX(69);
    TalonSRX shooterMotor2 = new WPI_TalonSRX(69);

    double SHOOTER_TIMER;
    double SHOOTER_SPEED;

    boolean shooterMode;

    public IntakeShooter() {

        shooterMotor2.follow(shooterMotor1);
        SHOOTER_TIMER = 1700;
        SHOOTER_SPEED = .8;
        shooterMode = true;
    }

    public void setShooterMode() {
        shooterMode = !shooterMode;

        if (shooterMode) {
            SHOOTER_SPEED = .8;
        } else {
            SHOOTER_SPEED = .22;
        }
    }

    public void setShooterMotor(double speed){
        shooterMotor1.set(ControlMode.Current, speed);
    }

     public void setIndexMotor(double speed){
        indexMotor.set(ControlMode.Current, speed);
    }

    public Command getShootCommand() {
        Command r_command = Commands.sequence(
            new InstantCommand(() -> setShooterMotor(SHOOTER_SPEED)),
            Commands.waitSeconds(SHOOTER_TIMER),
            new InstantCommand(() -> setIndexMotor(SHOOTER_SPEED)),
            Commands.waitSeconds(SHOOTER_TIMER),
            new InstantCommand(() -> setIndexMotor(0)).andThen(new InstantCommand(() -> setShooterMotor(0)))
        );

        r_command.addRequirements(this);
        return r_command;
    }

    public void periodic() {

    }

}
