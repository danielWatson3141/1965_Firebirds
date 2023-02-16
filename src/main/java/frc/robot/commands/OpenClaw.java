
package frc.robot.commands;

import frc.robot.subsystems.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OpenClaw extends CommandBase{

    Claw claw;
    // Called when the command is initially scheduled.


  public OpenClaw(Claw my_claw) {
    claw = my_claw;
  }

//hold to open claw

@Override
  public void initialize() {
    claw.clawCease();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.clawUnfurl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
