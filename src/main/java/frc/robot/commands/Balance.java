package frc.robot.commands;

import frc.robot.subsystems.SixWheelDrivetrain;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Balance extends CommandBase {

  SixWheelDrivetrain sixWheelDrivetrain;
  // Called when the command is initially scheduled.



  public Balance(SixWheelDrivetrain my_SixWheelDrivetrain) {
    sixWheelDrivetrain = my_SixWheelDrivetrain;
  }


  private SixWheelDrivetrain drivetrain;

  @Override
  public void initialize() {

  }

  final double DRIVE_SPEED_SLOW = .1;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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

// private void configureButtonBindings() {
// Logging.log("robot container", "buttons configured");

// aButton.whileTrue(
// new openClaw(claw_piston.set(Value.kForward));
// )
// aButton.whileFalse(
// new closeClaw(claw_piston.set(Value.kReverse));
// )
// }