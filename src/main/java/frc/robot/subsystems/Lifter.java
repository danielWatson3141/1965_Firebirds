package frc.robot.subsystems;

import javax.imageio.plugins.tiff.GeoTIFFTagSet;
import com.ctre.phoenix.motorcontrol.ControlMode;
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
    SlewRateLimiter steeringLimiter;

    double target_angle = 0;
    private double kP=2;
    private double kI=0;
    private double kD=0;

    PIDController pid;
    double setPoint = 0;
  
    public Lifter(XboxController cont) {
        myController = cont;
        lifterMotor = new TalonSRX(10);
        lifterMotor.setInverted(true);
        
        claw_piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        steeringLimiter = new SlewRateLimiter(UP_RATE_LIMIT, DOWN_RATE_LIMIT, 0);

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
    
    public void setArmPosition(double position){
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 10),
                                                new TrapezoidProfile.State(5, 0),
                                                new TrapezoidProfile.State(0, 0));
        Logging.log("Lifter:setArmPosition","Setting ArmPosition to: "+position);
    }

    public final double ARM_BOTTOM_POSITION = 0;
    public final double ARM_MIDDLE_POSITION = 0.5;
    public final double ARM_TOP_POSITION = 1;

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
        if (getArmPosition() >= ARM_MIDDLE_POSITION)
            goToTop();
        else 
            goToMiddle();
    }

    public void moveArmDown(){
        if (getArmPosition() > ARM_MIDDLE_POSITION)
            goToMiddle();
        else 
            goToBottom();
    }

    public double getArmPosition(){
        return lifterMotor.getSelectedSensorPosition();
    }
    
    public double getSpeed() {
        return lifterMotor.getMotorOutputPercent();
    }

    double ARM_SPEED_FACTOR = 0.25;
    
    public void FineTuning(){
        double rightStickY = myController.getRightY();
        setArmSpeed(ARM_SPEED_FACTOR * rightStickY);
    }

    long previousTime = 0;
    @Override
    public void periodic() {
        report_data();

        //Get the current time
        long currentTime = System.currentTimeMillis();
        //Calculate how much time has passed
        long elapsedTime = currentTime - previousTime; 
        //Set the previous time up for later
        previousTime = currentTime;

        //Compute where the arm should be after the time elapsed
        State setpoint = profile.calculate(elapsedTime);
        
        double output = pid.calculate(lifterMotor.getSelectedSensorPosition(), setpoint.position);
        lifterMotor.set(ControlMode.Position, output);
    }

    public void report_data() {
        SmartDashboard.putNumber("Set Point", setPoint);
        SmartDashboard.putNumber("Position",getArmPosition());
        SmartDashboard.putNumber("Speed",getSpeed());
    }
}