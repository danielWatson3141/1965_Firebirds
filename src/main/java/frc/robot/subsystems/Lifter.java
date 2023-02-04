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

    Compressor compressor;
    SlewRateLimiter lifterSpeedLimiter;

    boolean direct_input_mode = false;

    double target_angle = 0;
    private double kP=2;
    private double kI=0;
    private double kD=0;

    PIDController pid;
    double setPoint = 0;
  
    public Lifter(XboxController cont) {
        myController = cont;
        lifterMotor = new TalonSRX(2);
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

        lifterMotor.setInverted(false);
        
        claw_piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        lifterSpeedLimiter = new SlewRateLimiter(UP_RATE_LIMIT, DOWN_RATE_LIMIT, 0);
        SlewRateLimiter lifterLimiter = new SlewRateLimiter(1.3);

        pid = new PIDController(kP, kI, kD);
        // pid.setinputrange 
        // set output range 
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
    
    public void setArmSpeed(double speed){
        Logging.log("Lifter:setArmPosition", "Setting ArmSpeed to "+speed);
        lifterMotor.set(ControlMode.PercentOutput, speed ); 
    }

    TrapezoidProfile profile;
    double destination;
    
    public void setArmPosition(double position){
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 10),
                                                new TrapezoidProfile.State(5, 0),
                                                new TrapezoidProfile.State(0, 0));
        destination = position;
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
        setArmPosition(ARM_TOP_POSITION);
    }

    public void moveArmUp(){
        Logging.log("Lifter:moveArmUp", "Moving arm up");
        if (destination >= ARM_MIDDLE_POSITION)
            goToTop();
        else 
            goToMiddle();
    }

    public void moveArmDown(){
        Logging.log("Lifter:moveArmDown", "Moving arm Down");
        if (destination > ARM_MIDDLE_POSITION)
            goToMiddle();
        else 
            goToBottom();
    }

    public double getArmPosition(){
        return lifterMotor.getSelectedSensorPosition();
    }
    
    public double getSpeed() {
        return lifterMotor.getSelectedSensorVelocity();
    }

    double ARM_SPEED_FACTOR = 0.25;
    
    public void FineTuning(){
        double rightStickY = myController.getRightY();
        setArmSpeed(ARM_SPEED_FACTOR * rightStickY);
    }

    State setpoint = new State();

    long previousTime = 0;
    @Override
    public void periodic() {
        report_data();

        //Get the current time
        long currentTime = System.currentTimeMillis();
        //Calculate how much time has passed
        if(previousTime == 0){
            previousTime = currentTime;
        }

        long elapsedTime = currentTime - previousTime; 
        //Set the previous time up for later
        previousTime = currentTime;

        //Compute where the arm should be after the time elapsed
        if( profile == null)
            return;
        
        if(direct_input_mode){
            
            double rightStickY = myController.getRightY();

            double targetSpeed = lifterSpeedLimiter.calculate( rightStickY );

            setArmSpeed(targetSpeed);
        }

        {
            // setpoint = profile.calculate(elapsedTime);
            
            double output = pid.calculate(getArmPosition(), setpoint.position);
            SmartDashboard.putNumber("pid output", setpoint.velocity);

            // Logging.log("PID", "position: "+getArmPosition());
            // Logging.log("PID", "setpoint pos: "+setpoint.position);
            // Logging.log("PID", "setpoint vel: "+setpoint.velocity);
            // Logging.log("PID", "setpoint output: "+output);
            //System.out.println();

            // double output = pid.calculate(getSpeed(), setpoint.velocity);
            // lifterMotor.set(ControlMode.PercentOutput, output);
            lifterMotor.set(ControlMode.MotionMagic, destination );
        }
    }

    public void report_data() {
        SmartDashboard.putNumber("Position",getArmPosition());
        SmartDashboard.putNumber("Goal Position", destination);
        SmartDashboard.putNumber("Speed",getSpeed());
        SmartDashboard.putNumber("setPoint Displ", setpoint.position);
        SmartDashboard.putNumber("setPoint Speed", setpoint.velocity);

    }
}