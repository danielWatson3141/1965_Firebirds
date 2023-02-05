package frc.robot.subsystems;

import javax.imageio.plugins.tiff.GeoTIFFTagSet;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Logging;

public class Lifter extends SubsystemBase {

    private XboxController myController;

    public static final double UP_RATE_LIMIT = .2;
    public static final double DOWN_RATE_LIMIT = .2;
    DoubleSolenoid claw_piston;
    TalonSRX lifterMotor;

    double setPoint = 0;
  
    public Lifter(XboxController cont) {
        myController = cont;
        lifterMotor = new TalonSRX(2);
        configMotor();
        
        claw_piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    }

    public void setClawOpen() {
        Logging.log("Lifter:setClawOpen", "opening the claw");
        claw_piston.set(Value.kForward);
    }

    public void setClawClosed() {
        Logging.log( "Lifter:setClawClosed","closing the claw");
    
        claw_piston.set(Value.kReverse);
    }

    public boolean getClawOpen() {
        Value state = claw_piston.get();

        return state == Value.kForward;
    }

    TrapezoidProfile profile;
    
    public void setArmPosition(double position){
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 10),
                                                new TrapezoidProfile.State(5, 0),
                                                new TrapezoidProfile.State(0, 0));
        setPoint = position;
        Logging.log("Lifter:setArmPosition","Setting ArmPosition to: "+position);
    }

    public final double ARM_BOTTOM_POSITION = -10000;
    public final double ARM_MIDDLE_POSITION = 0;
    public final double ARM_TOP_POSITION = 10000;

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
    double ARM_STICK_SPEED = 10;
    double RIGHT_STICK_DEADZONE = 0.05;

    @Override
    public void periodic() {
        report_data();

        //get how much time has passed since last iteration
        double elapsedTime = time_elapsed();

        //get input from the controller
        double rightStickY = myController.getRightY();

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
            setPoint = newSetPoint;
            lifterMotor.set(ControlMode.MotionMagic, setPoint);
        }
    }

    public void report_data() {
        SmartDashboard.putNumber("Position",getArmPosition());
        SmartDashboard.putNumber("Goal Position", setPoint);
        SmartDashboard.putNumber("Speed",getSpeed());
    }

    private void configMotor(){
        
        lifterMotor.configFactoryDefault();
        lifterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        lifterMotor.configNeutralDeadband(.001, 30);
        lifterMotor.setSensorPhase(false);
        lifterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

        /* Set the peak and nominal outputs */
		lifterMotor.configNominalOutputForward(0, 30);
		lifterMotor.configNominalOutputReverse(0, 30);
		lifterMotor.configPeakOutputForward(1, 30);
		lifterMotor.configPeakOutputReverse(-1, 30);

		/* Set Motion Magic gains in slot0 - see documentation */
		lifterMotor.selectProfileSlot(0, 0);
		lifterMotor.config_kF(0, 0.2, 30);
		lifterMotor.config_kP(0, 0.2, 30);
		lifterMotor.config_kI(0, 0, 30);
		lifterMotor.config_kD(0, 0, 30);

		/* Set acceleration and vcruise velocity - see documentation */
		lifterMotor.configMotionCruiseVelocity(3000, 30);
		lifterMotor.configMotionAcceleration(3000, 30);

		/* Zero the sensor once on robot boot up */
		lifterMotor.setSelectedSensorPosition(0, 0, 30);
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