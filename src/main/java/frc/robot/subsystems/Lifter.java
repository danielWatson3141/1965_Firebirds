package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;

public class Lifter extends SubsystemBase {

    private XboxController myController;

    public static final double UP_RATE_LIMIT = .2;
    public static final double DOWN_RATE_LIMIT = .2;

    TalonSRX lifterMotor;

    double setPoint = 0;

    private ShuffleboardTab lifterTab = Shuffleboard.getTab("Lifter");
    private GenericEntry armPositionEntry = lifterTab.add("armPosition", 0).getEntry();
    private GenericEntry armSPEntry = lifterTab.add("armSetPoint", 0).getEntry();

    public Lifter(XboxController cont) {
        myController = cont;
        lifterMotor = new TalonSRX(2);
        configMotor();
    }
    
    public void setArmPosition(double lposition){
        setPoint = lposition;
        lifterMotor.set(ControlMode.MotionMagic, setPoint);
        Logging.log("Lifter:setArmPosition","Setting ArmPosition to: "+lposition);
    }

    public final double ARM_BOTTOM_POSITION = 100;
    public final double ARM_MIDDLE_POSITION = 500;
    public final double ARM_TOP_POSITION = 900;

    public void goToBottom(){
        setArmPosition(ARM_BOTTOM_POSITION);
      Logging.log("Lifter:goToBottom","Setting ArmPositon to bottom");  
    }

    public void goToMiddle(){
     Logging.log("Lifter:goToMiddle","Setting ArmPosition to middle");
        setArmPosition(ARM_MIDDLE_POSITION);
    }

    public void goToTop(){
        Logging.log("Lifter:goToTop","Setting ArmPosition to top");
        setArmPosition(ARM_TOP_POSITION);
    }

    public void moveArmUp(){
        Logging.log("Lifter:moveArmUp","Setpoint: "+setPoint);  

        Logging.log("Lifter:moveArmUp", "Moving arm up");
        if (setPoint >= ARM_MIDDLE_POSITION)
            goToTop();
        else 
            goToMiddle();
    }

    public void moveArmDown(){
        Logging.log("Lifter:moveArmDown","Setpoint: "+setPoint);  

        Logging.log("Lifter:moveArmDown", "Moving arm Down");
        if (setPoint <= ARM_MIDDLE_POSITION )
            goToBottom();
        else 
            goToMiddle();
    }

    public double getArmPosition(){
        return lifterMotor.getSelectedSensorPosition();
    }
    
    public double getSpeed() {
        return lifterMotor.getSelectedSensorVelocity();
    }

    //10% of range per second
    double ARM_STICK_SPEED = .5;
    double RIGHT_STICK_DEADZONE = 0.05;

    private GenericEntry rightStickEntry = lifterTab.add("RightStick_Y", 0).getEntry();

    @Override
    public void periodic() {
        dashboard_update();

        //get how much time has passed since last iteration
        double elapsedTime = time_elapsed();

        //get input from the controller
        double rightStickY = -myController.getRightY();
        rightStickEntry.setDouble(rightStickY);

        //Don't take stick input if its close to zero
        if (Math.abs(rightStickY) < RIGHT_STICK_DEADZONE)
            rightStickY = 0;

        //Compute new setpoint
        double newSetPoint = setPoint + rightStickY * ARM_STICK_SPEED * elapsedTime;
        
        //Don't move arm above top position
        if (newSetPoint > ARM_TOP_POSITION){
            newSetPoint = ARM_TOP_POSITION;
        }

        //Don't move arm below bottom position
        if (newSetPoint < ARM_BOTTOM_POSITION){
            newSetPoint = ARM_BOTTOM_POSITION;
        }

        //Don't re-apply the setPoint if it has not changed
        if(Math.abs(newSetPoint - setPoint) > 0 )
        {
            setArmPosition(newSetPoint);
        }
    }

    public void dashboard_update() {
        armPositionEntry.setDouble(getArmPosition());
        armSPEntry.setDouble(setPoint);
    }
    /* Motor constant, DONT CHANGE */
    private double KF = .2;
    /* Proportional coefficient */
    private double KP = .1;
    /* Integral coefficient */
    private double KI = .0025;
    /* Dont Care */
    private double KD = 0;

    private double CRUISE_SPEED = 150;
    private double ACCEL = 200;

    private void configMotor(){
        
        /* Revert all configurations to factory default values */
        lifterMotor.configFactoryDefault();
        /* Ensures green light = good */
        lifterMotor.setInverted(true);

        lifterMotor.setNeutralMode(NeutralMode.Brake);

        /* Select the feedback device for the motor controller */
        lifterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        /* Configures the output deadband percentage */
        lifterMotor.configNeutralDeadband(.01, 30);
        /* Sets the period of the given status frame */
        lifterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

        /* Set the peak and nominal outputs */
		lifterMotor.configNominalOutputForward(0, 30);
		lifterMotor.configNominalOutputReverse(0, 30);
		lifterMotor.configPeakOutputForward(1, 30);
		lifterMotor.configPeakOutputReverse(1, 30);

		/* Set Motion Magic gains in slot0 - see documentation */
		lifterMotor.selectProfileSlot(0, 0);
		lifterMotor.config_kF(0, KF, 30);
		lifterMotor.config_kP(0, KP, 30);
		lifterMotor.config_kI(0, KI, 30);
		lifterMotor.config_kD(0, KD, 30);

		/* Set acceleration and vcruise velocity - see documentation */
        /* Max speed */
		lifterMotor.configMotionCruiseVelocity(CRUISE_SPEED, 30);
        /* Speed of acceleration */
		lifterMotor.configMotionAcceleration(ACCEL, 30);
		/* Zero the sensor once on robot boot up */
		lifterMotor.setSelectedSensorPosition(0, 0, 30);

        /* Upper limit of motor */
        lifterMotor.configForwardSoftLimitThreshold(900, 0);
        /* Lower limit of motor */
        lifterMotor.configReverseSoftLimitThreshold(-50, 0);
        /* Enable/disable upper limit of motor */
        lifterMotor.configForwardSoftLimitEnable(false, 0);
        /* Enable/disable lower limit of motor */
        lifterMotor.configReverseSoftLimitEnable(false, 0);
    }

    long previousTime = 0;

    private double time_elapsed(){
        //Get the current time
        long currentTime = System.currentTimeMillis();
        //Calculate how much time has passed
        if(previousTime == 0){
            previousTime = currentTime;
        }

        long elapsedTime = currentTime - previousTime; 
        //Set the previous time up for later
        previousTime = currentTime;

        return elapsedTime;
    }
}