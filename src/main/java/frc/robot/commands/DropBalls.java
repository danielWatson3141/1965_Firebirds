// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//amonguys
package frc.robot.commands;

import frc.robot.subsystems.Cannon;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DropBalls extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Cannon cannon;

  /**
   * Creates a new DropBalls Command.
   *
   * @param Cannon The subsystem used by this command.
   */
  public DropBalls(Cannon c) {
    cannon = c;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cannon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cannon.toggleBelt(true);
    cannon.setPegToggle(false);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  // called periodically
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cannon.toggleBelt(false);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() { // is finished when everything stops
    return false;
  }
}
