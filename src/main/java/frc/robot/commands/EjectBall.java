// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//amonguys
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cannon;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class EjectBall extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final Cannon cannon;
  private final Intake intake;
  //Reverse to get rid of imposter ball


  /**
   * Creates a new EjectBall command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public EjectBall(Cannon c, Intake i) {
    cannon = c;
    intake = i;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cannon, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cannon.setBeltReverse();
    intake.setSpinnerReverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cannon.toggleBelt(false);
    intake.setSpinnerEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
