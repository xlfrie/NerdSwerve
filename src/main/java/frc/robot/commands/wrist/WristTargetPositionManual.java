// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class WristTargetPositionManual extends Command {
  /** Creates a new WristTargetPositionManual. */
  Wrist wrist;
  double targetpos;

  public WristTargetPositionManual(Wrist wr, double tar) {
    this.wrist = wr;
    this.targetpos = tar;
    addRequirements(wr);
    // Use addRequirements() here to declare subsystem dependencies. -3
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setTargetPosition(targetpos);
    wrist.setEnabled(true);
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
