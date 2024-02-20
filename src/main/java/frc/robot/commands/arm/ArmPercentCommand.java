// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmPercentCommand extends Command {
  /** Creates a new ArmPercentCommand. */
  Arm arm;
  double speed;
  boolean shuffleboard;
  public ArmPercentCommand(Arm a, double s, boolean sb) {
    this.arm = a;
    this.speed = s;
    this.shuffleboard = sb;
    addRequirements(a);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setEnabled(false);
    if (shuffleboard){
      arm.setPercentageShuffleBoard();
    }
    else{
          arm.setPercentage(speed);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
