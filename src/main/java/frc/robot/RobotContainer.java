// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Shooter.shooterPercentCommand;
import frc.robot.commands.arm.ArmPercentCommand;
import frc.robot.commands.arm.ArmTargetPositionManual;
import frc.robot.commands.commandgroups.AmpScoringCommand;
import frc.robot.commands.commandgroups.GroundIntakeCommand;
import frc.robot.commands.commandgroups.NeutralPositionCommand;
import frc.robot.commands.commandgroups.ShooterFeedingCommand;
import frc.robot.commands.indexer.IndexerPercentCommand;
import frc.robot.commands.intake.IntakePercentCommand;
import frc.robot.commands.intake.IntakeTargetVelocityManual;
import frc.robot.commands.intake.intakeControlledCommand;
import frc.robot.commands.swerve.SwerveJoystickCommand;
import frc.robot.commands.wrist.WristPercentCommand;
import frc.robot.commands.wrist.WristTargetPositionManual;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.FlyWheelShooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.GyroStuffs.PigeonV2;
import frc.robot.subsystems.LEDs.AboveBumperLEDs;
import frc.robot.subsystems.LEDs.DriveTrainLEDs;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

import javax.print.attribute.standard.NumberUp;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here.

  // Replace with CommandPS4Controller or CommandJoystick if needed
  PS5Controller driveController = new PS5Controller(0);
  private JoystickButton driveXbutton = new JoystickButton(driveController, 1);
  private JoystickButton driveAbutton = new JoystickButton(driveController, 2);
  private JoystickButton driveYbutton = new JoystickButton(driveController, 3);
  private JoystickButton driveBbutton = new JoystickButton(driveController, 4);
  private JoystickButton driveRightBumperButton = new JoystickButton(driveController, 5);
  private JoystickButton driveLeftBumperButton = new JoystickButton(driveController, 6);
  private JoystickButton driveLeftTriggerButton = new JoystickButton(driveController, 7);
  private JoystickButton driveRightTriggerButton = new JoystickButton(driveController, 8);
  private POVButton driverUpPOVButton = new POVButton(driveController, 0);
  private POVButton driverDownPOVButton = new POVButton(driveController, 180);
  private POVButton driverRightPOVButton = new POVButton(driveController, 90);
    private POVButton driverLeftPOVButton = new POVButton(driveController, 270);









  private PigeonV2 gyro = new PigeonV2(0);
  private SwerveDriveTrain drive = new SwerveDriveTrain(gyro);
  private Arm arm = new Arm();
  private Wrist wrist = new Wrist();
  private Intake intake = new Intake();
  private Indexer indexer = new Indexer();
  private FlyWheelShooter shooter = new FlyWheelShooter();
 
  private DriveTrainLEDs dtLEDs = new DriveTrainLEDs();
  // private AboveBumperLEDs abLEDs = new AboveBumperLEDs();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drive.setDefaultCommand(new SwerveJoystickCommand(driveController::getLeftY, driveController::getLeftX, driveController::getRightX, drive));
    // Configure the trigger bindings
    configureBindings();
    initShuffleboard();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

     // driveAbutton.whileTrue(new ArmPercentCommand(arm, 0.75, false));
     // driveBbutton.whileTrue(new ArmPercentCommand(arm, -0.75, false));
      driveAbutton.whileTrue(new IndexerPercentCommand(indexer, -0.2));
      driveBbutton.whileTrue(new IndexerPercentCommand(indexer, 0.2));



      driveXbutton.whileTrue(new WristPercentCommand(wrist, 0.1, false));
      driveYbutton.whileTrue(new WristPercentCommand(wrist, -0.1, false));

      driveLeftTriggerButton.whileTrue(new IntakePercentCommand(intake, 0.2));
      driveRightTriggerButton.whileTrue(new IntakePercentCommand(intake, -0.2));

      // driveAbutton.whileTrue(new IntakePercentCommand(intake, 0.5));
      // driveBbutton.whileTrue(new IntakePercentCommand(intake, -0.5));

      driveLeftBumperButton.whileTrue(new shooterPercentCommand(shooter, 1.0));
      driveRightBumperButton.whileTrue(new shooterPercentCommand(shooter, -1.0));

      //driverUpPOVButton.whileTrue(new ArmTargetPositionManual(arm, 31));
      //driverDownPOVButton.whileTrue(new WristTargetPositionManual(wrist, -3));
      
      driverDownPOVButton.whileTrue(new NeutralPositionCommand(arm, wrist,intake));
      driverRightPOVButton.whileTrue(new GroundIntakeCommand(arm, intake, wrist));
      driverLeftPOVButton.onTrue(new ShooterFeedingCommand(arm, intake, wrist, shooter));
      driverUpPOVButton.onTrue(new AmpScoringCommand(arm, wrist, intake));

      //driverLeftPOVButton.onTrue(new intakeControlledCommand(intake, 2400));

    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.


  }
  public void initShuffleboard() {
  //drive.initModuleShuffleboard(1);
  //drive.initMainShuffleboard(1);
  arm.initShuffleboard(1);
  intake.initShuffleboard(1);
  wrist.initShuffleboard(1);
  shooter.initShuffleboard(1);
  indexer.initShuffleboard(1);

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return new PathPlannerAuto("New Auto");

    // An example command will be run in autonomous
//   return null;
  }
}
