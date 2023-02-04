
package frc.robot.commands;

import frc.robot.subsystems.Lifter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OpenClaw extends CommandBase{

    Lifter lifter;
    // Called when the command is initially scheduled.


  public OpenClaw(Lifter my_lifter) {
    lifter = my_lifter;
  }

//hold to open claw

@Override
  public void initialize() {
    lifter.setClawOpen();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lifter.setClawClosed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
