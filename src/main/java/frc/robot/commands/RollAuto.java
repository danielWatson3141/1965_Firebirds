package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SixWheelDrivetrain;
import frc.robot.subsystems.Cannon;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RollAuto extends CommandBase{

    SixWheelDrivetrain drivetrain;
    // Called when the command is initially scheduled.


  public RollAuto(SixWheelDrivetrain drivetrainArg) {
      drivetrain = drivetrainArg; 
      addRequirements(drivetrainArg);
    }

private double auto_speed = -.23;

@Override
  public void initialize() {
  }

  long DRIVE_TIME=3000;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.goAtSpeed(auto_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.goAtSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
