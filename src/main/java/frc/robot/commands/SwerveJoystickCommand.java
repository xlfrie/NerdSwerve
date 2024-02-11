// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class SwerveJoystickCommand extends Command {
  /** Creates a new SwerveJoystickCommand. */
  private final SwerveDriveTrain drive;

  private double modifyInputs(double val, boolean isRot){
    if (isRot){
      if (Math.abs(val)<DriveConstants.kAngDeadband){
        val =0;
      }
      return val*drive.getAng();
    }
    else{
      if (Math.abs(val)<DriveConstants.kTanDeadband){
        val =0;
      }
      return val*drive.getTan();
    }
  }

    public void driveFromChassis(ChassisSpeeds speeds){
    var states = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states,SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    drive.setModuleStates(states);
  }

  private DoubleSupplier x,y,z;





  public SwerveJoystickCommand(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot, SwerveDriveTrain instance) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.x = fwd;
    this.y = str;
    this.z = rot;

    drive = instance;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(modifyInputs(x.getAsDouble(), false), modifyInputs(y.getAsDouble(), false),modifyInputs(z.getAsDouble(), true),
    drive.getDriveHeading()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveFromChassis(new ChassisSpeeds());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
