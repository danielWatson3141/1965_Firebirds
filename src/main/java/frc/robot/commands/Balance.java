package frc.robot.commands;

import frc.robot.subsystems.SixWheelDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Balance extends CommandBase{

    SixWheelDrivetrain sixWheelDrivetrain;
    // Called when the command is initially scheduled.


  public Balance(SixWheelDrivetrain my_SixWheelDrivetrain) {
    sixWheelDrivetrain = my_SixWheelDrivetrain;
  }

  double gyroValue = 1;
  private SixWheelDrivetrain drivetrain;

@Override
  public void initialize() {

  } 

  final double DRIVE_SPEED_SLOW = .1;
  public final double GYRO_NEAR_ZERO_VALUE = 0 ;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(gyroValue) < GYRO_NEAR_ZERO_VALUE) 
      drivetrain.goAtSpeed(0); 
    else if (gyroValue < 0) 
      drivetrain.goAtSpeed(DRIVE_SPEED_SLOW);   
    else 
      drivetrain.goAtSpeed(-DRIVE_SPEED_SLOW);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}



//  private void configureButtonBindings() {
//         Logging.log("robot container", "buttons configured");
 
//         aButton.whileTrue(
//             new openClaw(claw_piston.set(Value.kForward));
//         )
//         aButton.whileFalse(
//             new closeClaw(claw_piston.set(Value.kReverse));
//         )
//     }