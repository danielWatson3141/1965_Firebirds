package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Lifter extends SubsystemBase{

    DoubleSolenoid lifterArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    
    public void toggleLifter(){
        lifterArm.toggle();
        lifterArm.get();

        SmartDashboard.putBoolean("armState", lifterArm.get()==Value.kForward);

    }

    
}
