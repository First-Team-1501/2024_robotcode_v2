// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.climber.JogClimberDown;
import frc.robot.commands.climber.JogClimberUp;
import frc.robot.commands.climber.SetClimberPosition;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.RunOuttakeCommand;
import frc.robot.commands.intake.ShootNote;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.subsystems.climber.ClimberPositions;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.deck.DeckSubsystem;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterConfig;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

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
      new File(Filesystem.getDeployDirectory(), "swerve/neo")); // DO NOT UNDER ANY CIRCUMSTANCE CHANGE THIS FROM
                                                                // "SWERVE/NEO"!!!!!!!!!!!!!!

  // Our subsystems
  private IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
  private ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
  private DeckSubsystem DECK_SUBSYSTEM = new DeckSubsystem();
  private ElevatorSubsystem ELEVATOR_SUBSYSTEM = new ElevatorSubsystem();
  private ClimberSubsystem CLIMBER_SUBSYSTEM = new ClimberSubsystem();

  // Driver Joysticks
  CommandJoystick driverController = new CommandJoystick(0);
  CommandJoystick rotationController = new CommandJoystick(1);

  // Operator Xbox Controller
  XboxController operatorXbox = new XboxController(2);

  // Buttons for Xbox Controller
  private Trigger XBOX_RT = new JoystickButton(operatorXbox, 8); // Intake
  private Trigger XBOX_LT = new JoystickButton(operatorXbox, 7); // Outtake
  private Trigger XBOX_A = new JoystickButton(operatorXbox, 2); // Close shot
  private Trigger XBOX_B = new JoystickButton(operatorXbox, 3); // Medium shot
  private Trigger XBOX_Y = new JoystickButton(operatorXbox, 4); // Far shot

  // Buttons for Drive Joystick
  private Trigger DRIVE_TRIG = driverController.button(1);
  private Trigger DRIVE_B2 = driverController.button(2);
  private Trigger DRIVE_B3 = driverController.button(3);
  private Trigger DRIVE_B4 = driverController.button(4);
  private Trigger DRIVE_B5 = driverController.button(5);


  // Buttons for Roation Joystick
  private Trigger ROTATE_TRIG = rotationController.button(1);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //defaultCommands();

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
    XBOX_RT.whileTrue(
      new SetElevatorPosition(ELEVATOR_SUBSYSTEM, ElevatorPositions.intake).andThen
      (
        new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.intake)
        .alongWith(new RunIntakeCommand(INTAKE_SUBSYSTEM))
      )
      .andThen(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.home))
      .andThen(new SetElevatorPosition(ELEVATOR_SUBSYSTEM, ElevatorPositions.zero))
    )
    .onFalse
    (
      new SetElevatorPosition(ELEVATOR_SUBSYSTEM, ElevatorPositions.zero)
      .andThen(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.home))
    );        

    // Outtake: Spits out the note
    XBOX_LT.whileTrue(new RunOuttakeCommand(INTAKE_SUBSYSTEM));

    // Close shot
    XBOX_A.whileTrue(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.closeup))
        .whileTrue(new RevShooter(SHOOTER_SUBSYSTEM, ShooterConfig.closeLeftSpeed, ShooterConfig.closeRightSpeed))
        .onFalse(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.home));

    // Medium shot
    XBOX_B.whileTrue(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.podium))
        .whileTrue(new RevShooter(SHOOTER_SUBSYSTEM, ShooterConfig.podiumLeftSpeed, ShooterConfig.podiumRightSpeed))
        .onFalse(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.home));

    // Far shot
    XBOX_Y.whileTrue(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.backline))
        .whileTrue(new RevShooter(SHOOTER_SUBSYSTEM, ShooterConfig.farLeftSpeed, ShooterConfig.farRightSpeed))
        .onFalse(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.home));

    // Run intake to shoot note
    DRIVE_TRIG.whileTrue(new ShootNote(INTAKE_SUBSYSTEM));

    // Preclimb position
    DRIVE_B3.onTrue(new SetClimberPosition(CLIMBER_SUBSYSTEM, ClimberPositions.preclimb)
    .alongWith(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.preClimb)));

    // Climb
    DRIVE_B2.onTrue
    (
      new SetClimberPosition(CLIMBER_SUBSYSTEM, ClimberPositions.midClimb)
      .andThen
      (
        new SetClimberPosition(CLIMBER_SUBSYSTEM, ClimberPositions.climb)
        .alongWith(new SetDeckPosition(DECK_SUBSYSTEM, DeckPositions.climb))
      )
    );

    // Zero Gyro
    ROTATE_TRIG.onTrue(new InstantCommand(drivebase::zeroGyro));

    // Jog Climber Up
    DRIVE_B5.whileTrue(new JogClimberUp(CLIMBER_SUBSYSTEM));

    // Jog Climber Down
    DRIVE_B4.whileTrue(new JogClimberDown(CLIMBER_SUBSYSTEM));

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

  public void autonomousInit() 
  {
    new AutoSelec
  }
}
