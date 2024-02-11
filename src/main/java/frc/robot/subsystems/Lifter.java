package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lifter extends SubsystemBase{

    private DoubleSolenoid lifterArm;

    public final ShuffleboardTab lifterTab = Shuffleboard.getTab(getName());
    private GenericEntry armStateEntry = lifterTab.add("Is Arm Up", false)
    .withSize(1, 1).withPosition(0, 1)
    .getEntry();

    public Lifter() {
        lifterArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    }

    public void toggleLifter(){
        lifterArm.toggle();

        armStateEntry.setBoolean(lifterArm.get()==Value.kForward);

    }

    
}
