package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lifter extends SubsystemBase {

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
  
    public Lifter() {
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
        claw_piston.set(Value.kForward);
    }

    public void setClawClosed() {
        claw_piston.set(Value.kReverse);
    }

    public boolean getClawOpen() {
        Value state = claw_piston.get();

        return state == Value.kForward;
    }
    
    public void setArmSpeed(double speed){
        lifterMotor.set(ControlMode.PercentOutput, speed ); 
    }
    
    public void setArmPosition(double position){
        setPoint = position;
    }

    public double getArmPosition(){
        return lifterMotor.getSelectedSensorPosition();
    }
    
    public double getSpeed() {
        return lifterMotor.getSelectedSensorVelocity();
    }

    @Override
    public void periodic() {
        report_data();
        setArmSpeed(
            pid.calculate(
                getArmPosition(), setPoint
            )
        );
    }

    public void report_data() {
        SmartDashboard.putNumber("Set Point", setPoint);
        SmartDashboard.putNumber("Position", getArmPosition());
        SmartDashboard.putNumber("Speed", getSpeed());
    }
}