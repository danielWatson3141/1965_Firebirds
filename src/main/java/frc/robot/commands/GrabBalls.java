// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//RIP Kurt Cobain
package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Cannon;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GrabBalls extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Cannon cannon;
  private final Intake intake;

  // eject mode: peg 1 drops
  // storage mode: peg 1-3 raises

  // lift mode: peg 1-3 drops
  // collect mode: belt spins while intake is collecting, peg 3 is raised
  // deposit mode: belt spins while intake is closed, peg 1 is raised

  /**
   * Creates a new GrabBalls command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GrabBalls(Cannon c, Intake i) {
    cannon = c;
    intake = i;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cannon, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cannon.toggleBelt();
    intake.toggleRoller();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // cannon.isBallPresent(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // cannon.toggleBelt(false);
   // intake.setSpinnerEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
