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

public class Shooter extends SubsystemBase{
    
    WPI_TalonSRX sm_one = new WPI_TalonSRX(9);
    WPI_TalonSRX sm_two = new WPI_TalonSRX(8);
    WPI_TalonSRX sm_three = new WPI_TalonSRX(7);

    public void moveMf(double speed) {
        sm_one.set(speed);
        sm_two.set(speed);
        sm_three.set(speed);
    }
}
