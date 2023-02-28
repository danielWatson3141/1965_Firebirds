package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
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

    public Claw(XboxController cont) {
        myController = cont;
        // check device number
        clawMotor = new TalonSRX(1);
    }

    DigitalInput toplimitSwitch = new DigitalInput(0);
    DigitalInput bottomlimitSwitch = new DigitalInput(1);

    private boolean CLAW_OPEN = true;
    private boolean CLAW_CLOSED = !CLAW_OPEN;
    boolean clawState;

    double CLAW_SPEED = 0.1;

    public void clawToggle() {
        clawState = !clawState;

        Logging.log("Claw:clawToggle","setting claw position to " + (clawState ? "OPEN" : "CLOSE" ));
    }

   
    @Override
    public void periodic() {
        if (clawState == CLAW_OPEN) {
            if (toplimitSwitch.get()) {
                clawMotor.set(ControlMode.PercentOutput, 0);
            } else {
                clawMotor.set(ControlMode.PercentOutput, CLAW_SPEED);
            }
        } else {
            if (bottomlimitSwitch.get()) {
                clawMotor.set(ControlMode.PercentOutput, 0);
            } else {
                clawMotor.set(ControlMode.PercentOutput, -CLAW_SPEED);
            }
        }
    }

    public void report_data() {

    }

}