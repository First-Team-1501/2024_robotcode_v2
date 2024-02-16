// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.simple.RevShooter;
import frc.robot.commands.simple.RunIntakeCommand;
import frc.robot.commands.simple.RunOuttakeCommand;
import frc.robot.commands.simple.SetClimberPosition;
import frc.robot.commands.simple.SetDeckPosition;
import frc.robot.commands.simple.SetElevatorPosition;
import frc.robot.commands.simple.ShootNote;
import frc.robot.subsystems.ClimberSubsystem.ClimberPositions;
import frc.robot.subsystems.ClimberSubsystem.ClimberSubsystem;
import frc.robot.subsystems.DeckSubsystem.DeckPositions;
import frc.robot.subsystems.DeckSubsystem.DeckSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveSubsystem;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Swerve subsystem
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  // Our subsystems
  private IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
  private ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
  private DeckSubsystem DECK_SUBSYSTEM = new DeckSubsystem();
  private ElevatorSubsystem ELEVATOR_SUBSYSTEM = new ElevatorSubsystem();
  private ClimberSubsystem CLIMBER_SUBSYSTEM = new ClimberSubsystem();

  // Driver Joysticks
  CommandJoystick driverController = new CommandJoystick(0);
  CommandJoystick rotationController = new CommandJoystick(1);

  // Operatior Xbox Controller
  XboxController operatorXbox = new XboxController(2);

  // Declare buttons here
  private Trigger XBOX_RT = new JoystickButton(operatorXbox, 8); // Intake
  private Trigger XBOX_LT = new JoystickButton(operatorXbox, 7); // Outtake
  private Trigger XBOX_A = new JoystickButton(operatorXbox, 2); // Close shot
  private Trigger XBOX_B = new JoystickButton(operatorXbox, 3); // Medium shot
  private Trigger XBOX_Y = new JoystickButton(operatorXbox, 4); // Far shot

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    defaultCommands();

    // Regualar drive mode
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(rotationController.getRawAxis(0), OperatorConstants.RIGHT_X_DEADBAND));

    // Simulation drive mode
    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(rotationController.getRawAxis(0), OperatorConstants.RIGHT_X_DEADBAND));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

  }

  private void configureBindings() {
    // Intake sequence: extend elevator, lower deck, and intake
    XBOX_RT.whileTrue(new SetElevatorPosition(ELEVATOR_SUBSYSTEM, ElevatorPositions.intake))
        .whileTrue(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.intake))
        .whileTrue(new RunIntakeCommand(INTAKE_SUBSYSTEM))
        .onFalse(new SetElevatorPosition(ELEVATOR_SUBSYSTEM, ElevatorPositions.zero))
        .onFalse(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.home));

    // Outtake: Spits out the note
    XBOX_LT.whileTrue(new RunOuttakeCommand(INTAKE_SUBSYSTEM));

    // Close shot
    XBOX_A.whileTrue(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.closeup))
        .whileTrue(new RevShooter(SHOOTER_SUBSYSTEM));

    // Medium shot
    XBOX_B.whileTrue(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.podium))
        .whileTrue(new RevShooter(SHOOTER_SUBSYSTEM));

    // Far shot
    XBOX_Y.whileTrue(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.backline))
        .whileTrue(new RevShooter(SHOOTER_SUBSYSTEM));

    // Run intake to shoot note
    driverController.button(1).whileTrue(new ShootNote(INTAKE_SUBSYSTEM));

    // Preclimb position
    driverController.button(3).onTrue(new SetClimberPosition(CLIMBER_SUBSYSTEM, ClimberPositions.preclimb));

    // Climb
    driverController.button(2).onTrue(new SetClimberPosition(CLIMBER_SUBSYSTEM, ClimberPositions.climb));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("Path1", true);
  }

  // Default commands - these are setting the default positions for the elevator
  // and the deck
  public void defaultCommands() {
    DECK_SUBSYSTEM.setDefaultCommand(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.home));
    ELEVATOR_SUBSYSTEM.setDefaultCommand(new SetElevatorPosition(ELEVATOR_SUBSYSTEM, ElevatorPositions.zero));
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void autonomousInit() {

  }
}
