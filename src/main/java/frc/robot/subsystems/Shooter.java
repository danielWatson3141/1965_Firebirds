package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;


public class Shooter extends SubsystemBase {

    final long shooterTimer = 2000;
    final double shooterSpeed = 0.5;

    final int shooterState__WaitSpinup = 1;
    final int shooterState__WaitCan    = 2;
    final int shooterState__Done       = 3;
    int shooterState                   = 0;

    private long timeWhenPressed;
    
    public Shooter() {
        Logging.log("Shooter:Shooter", "Constuctor");
    
        WPI_TalonSRX shooterMotor1 = new WPI_TalonSRX(9);
        WPI_TalonSRX shooterMotor2 = new WPI_TalonSRX(8);
        WPI_TalonSRX canMotor = new WPI_TalonSRX(7);
    
        shooterState = shooterState__WaitSpinup;
        this.shooterInit();
    }

    // What does this do?
    // This determines if the requested time has elapsed
    public boolean CheckTimer(long timerValue) {
        long elapsedTime = System.currentTimeMillis() - timeWhenPressed;
        return(elapsedTime >= shooterTimer);
    }
    
    public void StartTimer() { 
        Logging.log("Shooter:StartTimer", "started timer");
        timeWhenPressed = System.currentTimeMillis();
     }
    
     public void shooterInit() {
        StartTimer();
        shooterMotor1.set(shooterSpeed);
        shooterMotor2.set(shooterSpeed);
    }

    public void startCan() {
        StartTimer();
        canMotor.set(shooterSpeed);
    }

    public void shooterStop() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
        canMotor.set(0);
    }

    public void periodic() {
        switch (shooterState) {
            case shooterState__WaitSpinup:
                if (this.CheckTimer(shooterTimer)) {
                    Logging.log("Shooter:periodic", "Initial timer expires");
                    this.startCan();
                    shooterState = shooterState__WaitCan;
                }
                break;
            case shooterState__WaitCan:
                if (this.CheckTimer(shooterTimer)) { 
                    Logging.log("Shooter:periodic", "Final timer expires - all done");
                    this.shooterStop();
                    shooterState = shooterState__Done;
                }
                break;
            default:
                break;
        }
    }
}
