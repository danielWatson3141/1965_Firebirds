package frc.robot.subsystems;

import javax.imageio.plugins.tiff.GeoTIFFTagSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Lifter {

    public static final double UP_RATE_LIMIT = .2;
    public static final double DOWN_RATE_LIMIT = .2;
    DoubleSolenoid claw_piston;
    VictorSPX lifterMotor;

    Compressor compressor;
    SlewRateLimiter steeringLimiter;
  
    public Lifter() {
        lifterMotor = new VictorSPX(10);
        lifterMotor.setInverted(true);
        
        claw_piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        steeringLimiter = new SlewRateLimiter(UP_RATE_LIMIT, DOWN_RATE_LIMIT, 0);
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
    
    public void setArmPosition(double position){}
        // This function tells the arm to be at a specific position. 

    
    public double getSpeed() {
        return lifterMotor.getMotorOutputPercent();
    }
}