package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;


public class Shooter extends SubsystemBase{
    
    WPI_TalonSRX shooterMotor1 = new WPI_TalonSRX(9);
    WPI_TalonSRX shooterMotor2 = new WPI_TalonSRX(8);
    WPI_TalonSRX canMotor = new WPI_TalonSRX(7);

    long elapsedTime = 0;
    long timeWhenPressed = 0;

    double shooterTimer = 2000;
    

    public shooterAccel() {
        timeWhemPressed = System.currentTimeMillis();

        shooterMotor1.set(.5);
        shooterMotor2.set(.5);

        if (shooterMotorDelay()) {
            shooterMotor1.set(0);
            shooterMotor2.set(0);
        }

    }

    public bool shooterMotorDelay() {
        if (elapsedTime >= shooterTimer) {
            return true;
        } else {
            return false;
        }
    }

    public void periodic() {
        elapsedTime = System.currentTimeMillis() - timeWhemPressed;

        shooterMotorDelay();
    }

}
