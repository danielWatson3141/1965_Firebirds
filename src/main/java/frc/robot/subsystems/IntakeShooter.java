package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;

public class IntakeShooter extends SubsystemBase {

    private final boolean HAVE_INDEX_MOTOR = true;

    WPI_TalonSRX rollerMotor = new WPI_TalonSRX(9);

    WPI_TalonSRX indexShooter = new WPI_TalonSRX(10);
    WPI_TalonSRX indexIntake = new WPI_TalonSRX(11);

    public double INTAKE_TIMEOUT_SECONDS;
    double INTAKE_SPEED;

    DigitalInput limitSwitch1 = new DigitalInput(0);
    DigitalInput limitSwitch2 = new DigitalInput(1);

    WPI_TalonSRX shooterMotor1 = new WPI_TalonSRX(8);
    WPI_TalonSRX shooterMotor2 = new WPI_TalonSRX(6);

    double SHOOTER_TIMER_SECONDS;
    double SHOOTER_SPEED;

    boolean shooterMode;


    public IntakeShooter() {

        shooterMotor2.follow(shooterMotor1);
        SHOOTER_TIMER_SECONDS = 1.7;
        SHOOTER_SPEED = .8;
        shooterMode = true;

        INTAKE_TIMEOUT_SECONDS = 3;
        INTAKE_SPEED = 0.6;

    }

    public boolean switch1State() {
        return !limitSwitch1.get();
    }

    public boolean switch2State() {
      return !limitSwitch2.get();
    }

    public void setShooterMode() {
        shooterMode = !shooterMode;

        if (shooterMode) {
            SHOOTER_SPEED = .8;
        } else {
            SHOOTER_SPEED = .22;
        }
    }

    public void runIntakeMotors(double speed){
        rollerMotor.set(speed);
        indexIntake.set(speed);
        Logging.log("IntakeShooter", "set intake");
    }
    public void stopIntakeSequence(){
        rollerMotor.stopMotor();
        indexIntake.stopMotor();
        Logging.log("IntakeShooter", "stop intake");
    }


    public void runShooterMotors(double speed){
        shooterMotor1.set(speed);
    }
    public void stopShooterSequence(){
        shooterMotor1.stopMotor();
        indexShooter.stopMotor();

    }

     public void setIndexMotor(double speed){
        indexShooter.set(speed);
    
    }




    public Command getShootCommand() {
        Command r_command = Commands.sequence(
            new InstantCommand(() -> runShooterMotors(SHOOTER_SPEED)),
            Commands.waitSeconds(SHOOTER_TIMER_SECONDS),
            new InstantCommand(() -> setIndexMotor(SHOOTER_SPEED)),
            Commands.waitSeconds(SHOOTER_TIMER_SECONDS),
            new InstantCommand(() -> stopShooterSequence())
        );

        r_command.addRequirements(this);
        return r_command;
    }

    public Command getIntakeCommand() {
        Command r_command = (
           new InstantCommand(() -> runIntakeMotors(INTAKE_SPEED)).until(() -> switch2State())//.andThen(new InstantCommand(() -> stopIntakeSequence())
        );

        r_command.addRequirements(this);
        return r_command;
    }



    public void periodic() {
        switch1State();
        switch2State();
        SmartDashboard.putBoolean("switch state 1", switch1State());
        SmartDashboard.putBoolean("switch state 2", switch2State());
    }

}

//new ConditionalCommand(new InstantCommand(() -> setIntakeMotors(0)),  )