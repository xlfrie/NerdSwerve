// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class WristPercentCommand extends Command {
  double speed;
  boolean shuffleboard;
  Wrist wrist;

  /** Creates a new WristPercentCommand. */
  public WristPercentCommand(Wrist w, double s, boolean sb) {
    wrist = w;
    speed = s;
    shuffleboard = sb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setEnabled(false);
    if (shuffleboard){
      wrist.setPercentageShuffleBoard();
    }
    else{
      wrist.setPercentage(speed);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
