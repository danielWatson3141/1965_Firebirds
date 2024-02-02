package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;


public class Shooter extends SubsystemBase {

    final long SHOOTER_TIMER_MS = 2000;
    private double shooterSpeed = .5;

    private WPI_TalonSRX shooterMotor1;
    private WPI_TalonSRX shooterMotor2;
    private WPI_TalonSRX canMotor;

    public Shooter() {
        Logging.log("Shooter:Shooter", "Constuctor");
    
        //sets motors to port number
        shooterMotor1 = new WPI_TalonSRX(9);
        shooterMotor2 = new WPI_TalonSRX(8);
        canMotor = new WPI_TalonSRX(7);
    }
    
    //sets shooting motor's voltage to the variable
     public void shooterInit() {
        shooterMotor1.set(shooterSpeed);
        shooterMotor2.set(shooterSpeed);
    }
        
    //sets the can motor's votage to the variable
    public void startCan() {
        canMotor.set(shooterSpeed);
    }

    //stops all motors
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
                //Starts up the shooter motors
                new InstantCommand(() -> shooterInit()),
                //waits for variable miliseconds
                Commands.waitSeconds(SHOOTER_TIMER_MS/1000),
                //Starts can motor
                new InstantCommand(() -> startCan()),
                //wait for variable miliseconds
                Commands.waitSeconds(SHOOTER_TIMER_MS/1000),
                //stop all motors
                new InstantCommand(() -> shooterStop())
            );
        r_command.addRequirements(this);
        return r_command;
    }

    /*
        VERY IMPORTANT (\0_0)\ 
        This function works by calling the function and putting the desired speed into the requirements.
        It will not work if you do "motorSet()"
        It HAS to have a value like "motorSet(.2)"
    */
    public void shooterMotorSet(double setSpeed) {
        shooterSpeed = setSpeed;
        double speedPercentage = shooterSpeed * 100;

        String speed = "speed is at" + speedPercentage + " percent";
        
        Logging.log("Shooter:shooterMotorSet", speed);
        SmartDashboard.putNumber("Shooter %", speedPercentage);
    }

}
