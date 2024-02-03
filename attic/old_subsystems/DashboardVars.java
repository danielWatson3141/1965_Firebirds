package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//********FOR DOCUMENTATION PURPOSES********
public class DashboardVars extends SubsystemBase {

    double exampleVar;

    public DashboardVars() {
        //published wigit
        SmartDashboard.putNumber("example", 0);
    }
    
    @Override
    public void periodic() {
        //gets value from wigit
        exampleVar = SmartDashboard.getNumber("example", 0);
        //gives value with unlimeted decimals
        SmartDashboard.putNumber("read raw value", exampleVar);
        //gives value to the nearest hundrith
        int exampleVarInt = (int)(exampleVar*100);
        SmartDashboard.putNumber("read modified value", ((double)exampleVarInt)/100.0);
    }
}
