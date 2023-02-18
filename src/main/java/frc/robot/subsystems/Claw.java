package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Logging;

public class Claw extends SubsystemBase {

    private XboxController myController;

    public static final double OPEN_RATE_LIMIT = .2;
    public static final double SHUT_RATE_LIMIT = .2;

    TalonSRX clawMotor;

    double setPoint = 0;

    public Claw(XboxController cont)
        myController = cont;
        // check device number
        clawMotor = new TalonSRX(1);
        configMotor();

    }

    public void setClawState(double position) {
        setPoint = position;
        Logging.log("claw:setClawState", "Setting ClawState to: " + position);
    }

    public final double CLAW_SHUT = -10000;
    public final double CLAW_OPEN = 10000;

    public void clawOpen() {
        setClawState(OPEN_RATE_LIMIT);
        Logging.log("Claw:clawOpen", "setting claw position to open");
    }

    public void clawShut() {
        setClawState(CLAW_SHUT);
        Logging.log("Claw:clawShut", "setting claw position to shut");
    }

    private void configMotor() {
        clawMotor.configFactoryDefault();
        clawMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        clawMotor.configNeutralDeadband(.001, 30);
        clawMotor.setSensorPhase(false);
        clawMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

        /* Set the peak and nominal outputs */
        clawMotor.configNominalOutputForward(0, 30);
        clawMotor.configNominalOutputReverse(0, 30);
        clawMotor.configPeakOutputForward(1, 30);
        clawMotor.configPeakOutputReverse(-1, 30);

        /* Set Motion Magic gains in slot0 - see documentation */
        clawMotor.selectProfileSlot(0, 0);
        clawMotor.config_kF(0, 0.2, 30);
        clawMotor.config_kP(0, 0.2, 30);
        clawMotor.config_kI(0, 0, 30);
        clawMotor.config_kD(0, 0, 30);

        /* Set acceleration and vcruise velocity - see documentation */
        clawMotor.configMotionCruiseVelocity(3000, 30);
        clawMotor.configMotionAcceleration(3000, 30);

        /* Zero the sensor once on robot boot up */
        clawMotor.setSelectedSensorPosition(0, 0, 30);
    }
}