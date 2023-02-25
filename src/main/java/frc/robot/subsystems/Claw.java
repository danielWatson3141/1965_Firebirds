package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Logging;

public class Claw extends SubsystemBase {

    private XboxController myController;

    public static final double OPEN_RATE_LIMIT = .2;
    public static final double SHUT_RATE_LIMIT = .2;

    TalonSRX clawMotor;

    double setPoint = 0;

    private ShuffleboardTab clawTab = Shuffleboard.getTab("Claw");
    private GenericEntry clawPositionEntry = clawTab.add("clawPosition", 0).getEntry();
    private GenericEntry clawSetPointEntry = clawTab.add("clawSetpoint", 0).getEntry();
    public Claw(XboxController cont){
        myController = cont;
        // check device number 
        clawMotor = new TalonSRX(1);
        configMotor();
    }

    public void setClawState(double cposition) {
        setPoint = cposition;
        Logging.log("claw:setClawState", "Setting ClawState to: " + cposition);
    }

    public final double CLAW_SHUT = 50;
    public final double CLAW_OPEN = 100;

    public void clawOpen() {
        setClawState(CLAW_OPEN);
        Logging.log("Claw:clawOpen", "setting claw position to open");
    }

    public void clawShut() {
        setClawState(CLAW_SHUT);
        Logging.log("Claw:clawShut", "setting claw position to shut");
    }

    @Override
    public void periodic() {
       
    }

   
}