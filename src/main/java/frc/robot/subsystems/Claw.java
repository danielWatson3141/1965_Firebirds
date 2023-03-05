package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;

public class Claw extends SubsystemBase {

    private XboxController myController;

    public static final double OPEN_RATE_LIMIT = .2;
    public static final double SHUT_RATE_LIMIT = .2;

    TalonSRX clawMotor;

    double setPoint = 0;

    private ShuffleboardTab clawTab = Shuffleboard.getTab("Claw");
    private GenericEntry topLimitEntry = clawTab.add("toplimit", 0).getEntry();
    private GenericEntry bottomLimitEntry = clawTab.add("bottomlimit", 0).getEntry();
    private GenericEntry clawStateEntry = clawTab.add("clawState", 0).getEntry();

    public Claw(XboxController cont) {
        myController = cont;
        // check device number
        clawMotor = new TalonSRX(11);
    }

    DigitalInput toplimitSwitch = new DigitalInput(8);
    DigitalInput bottomlimitSwitch = new DigitalInput(9);

    private boolean CLAW_OPEN = true;
    private boolean CLAW_CLOSED = !CLAW_OPEN;
    boolean clawState = true;

    double CLAW_SPEED_OPEN = .4;
    double CLAW_SPEED_CLOSE = 1;

    double CLAW_TIMER = 0;

    public void clawToggle() {
        clawState = !clawState;

        Logging.log("Claw:clawToggle", "setting claw position to " + (clawState ? "OPEN" : "CLOSE"));
        Logging.log("Claw:clawToggle", "top:" + toplimitSwitch.get() + " bottom:" + bottomlimitSwitch.get());
    }

    @Override
    public void periodic() {
        if (clawState == CLAW_OPEN) {
            if (!toplimitSwitch.get()) {
                clawMotor.set(ControlMode.PercentOutput, 0);
            } else {
                if (CLAW_TIMER >= 3) {
                    clawMotor.set(ControlMode.PercentOutput, 0);
                    CLAW_TIMER = 0;
                    Logging.log("Claw:stopping claw", "time exceeded");
                } else {
                    clawMotor.set(ControlMode.PercentOutput, CLAW_SPEED_CLOSE);
                    CLAW_TIMER = CLAW_TIMER + .1;
                }
            }
        } else {
            if (!bottomlimitSwitch.get()) {
                clawMotor.set(ControlMode.PercentOutput, 0);
            } else {
                if (CLAW_TIMER >= 3) {
                    clawMotor.set(ControlMode.PercentOutput, 0);
                    CLAW_TIMER = 0;
                } else {
                    clawMotor.set(ControlMode.PercentOutput, CLAW_SPEED_OPEN);
                    CLAW_TIMER = CLAW_TIMER + .1;
                    Logging.log("Claw:stopping claw", "time exceeded");
                }
            }
        }

        report_data();
    }

    public void report_data() {
        topLimitEntry.setBoolean(toplimitSwitch.get());
        bottomLimitEntry.setBoolean(bottomlimitSwitch.get());
        clawStateEntry.setBoolean(clawState);
    }

}