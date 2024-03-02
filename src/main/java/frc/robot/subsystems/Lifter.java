package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;

public class Lifter extends SubsystemBase{

    private DoubleSolenoid lifterArm1;
    private DoubleSolenoid lifterArm2;


    public final ShuffleboardTab lifterTab = Shuffleboard.getTab(getName());
    private GenericEntry armStateEntry = lifterTab.add("Is Arm Up", false)
    .withSize(1, 1).withPosition(0, 1)
    .getEntry();

    public Lifter() {
        lifterArm1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 2);
        lifterArm2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 3);

        lifterArm1.set(Value.kForward);
        lifterArm2.set(Value.kForward);
    }

    public void toggleLifter(){
        lifterArm1.toggle();
        lifterArm2.toggle();

        armStateEntry.setBoolean(lifterArm1.get()==Value.kForward);
        armStateEntry.setBoolean(lifterArm2.get()==Value.kForward);

    }

    
}
