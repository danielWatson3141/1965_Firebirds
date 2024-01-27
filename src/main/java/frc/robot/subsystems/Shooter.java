package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;


public class Shooter extends SubsystemBase {

    final long shooterTimerMs = 2000;
    final double shooterSpeed = 0.5;

    final int shooterState__WaitSpinup = 1;
    final int shooterState__WaitCan    = 2;
    final int shooterState__Done       = 3;
    int shooterState                   = 0;

    private long timeWhenPressed;

    private WPI_TalonSRX shooterMotor1;
    private WPI_TalonSRX shooterMotor2;
    private WPI_TalonSRX canMotor;

    public Shooter() {
        Logging.log("Shooter:Shooter", "Constuctor");
    
        shooterMotor1 = new WPI_TalonSRX(9);
        shooterMotor2 = new WPI_TalonSRX(8);
        canMotor = new WPI_TalonSRX(7);
    
        shooterState = shooterState__WaitSpinup;
    }
    
     public void shooterInit() {
        shooterMotor1.set(shooterSpeed);
        shooterMotor2.set(shooterSpeed);
    }

    public void startCan() {
        canMotor.set(shooterSpeed);
    }

    public void shooterStop() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
        canMotor.set(0);
    }

    public void periodic() {
        
    }

    public Command getShootCommand(){
        Command r_command = 
        Commands.sequence(
                new InstantCommand(() -> shooterInit()),
                Commands.waitSeconds(shooterTimerMs/1000),
                new InstantCommand(() -> startCan()),
                Commands.waitSeconds(shooterTimerMs/1000),
                new InstantCommand(() -> shooterStop())
            );
        r_command.addRequirements(this);
        return r_command;
    }
}
