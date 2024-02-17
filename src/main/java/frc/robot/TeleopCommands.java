
package frc.robot;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.shooter.ShooterConfig;


public class TeleopCommands 
{

  RobotContainer robot;
 
  // Driver Joysticks
  CommandJoystick driverController ;
  CommandJoystick rotationController;

  // Operator Xbox Controller
  XboxController operatorXbox;
  
  // Buttons for Xbox Controller
  private Trigger XBOX_RT; // Intake
  private Trigger XBOX_LT; // Outtake
  private Trigger XBOX_A; // Close shot
  private Trigger XBOX_B; // Medium shot
  private Trigger XBOX_Y; // Far shot

  // Buttons for Drive Joystick
  private Trigger DRIVE_TRIG;
  private Trigger DRIVE_B2;
  private Trigger DRIVE_B3;
  private Trigger DRIVE_B4;
  private Trigger DRIVE_B5;
    
  // Buttons for Roation Joystick
  private Trigger ROTATE_TRIG;

    
  public TeleopCommands(RobotContainer container)
  {
      robot = container;
      ConfigureBindings();
      SetupDefaultCommands();
      SetupOperatorCommands();
      SetupDriverCommands();
  }

  private void ConfigureBindings()
  {
      driverController = new CommandJoystick(0);
      rotationController = new CommandJoystick(1);

      operatorXbox = new XboxController(2);

      // triggers

      // TODO: I think the xbox controller has an enum for buttons? if not, create one, and rename these variables
      // so that they have to do with the action they get bound to rather than their button. That way we don't need comments
      // explaining what they do

      // operator
      XBOX_RT = new JoystickButton(operatorXbox, 8);
      XBOX_LT = new JoystickButton(operatorXbox, 7);

      XBOX_A = new JoystickButton(operatorXbox, 2);
      XBOX_B = new JoystickButton(operatorXbox, 3); 
      XBOX_Y = new JoystickButton(operatorXbox, 4);


      // driver
      DRIVE_TRIG = driverController.button(1);
      DRIVE_B2 = driverController.button(2);
      DRIVE_B3 = driverController.button(3);
      DRIVE_B4 = driverController.button(4);
      DRIVE_B5 = driverController.button(5);

      ROTATE_TRIG = rotationController.button(1);

  }

  private void SetupDefaultCommands()
  {
        // Regualar drive mode
      Command driveFieldOrientedDirectAngle = robot.getDrivebase().driveCommand(
          () -> -MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
          () -> -MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
          () -> -MathUtil.applyDeadband(rotationController.getRawAxis(0), OperatorConstants.RIGHT_X_DEADBAND));

      // Simulation drive mode
      Command driveFieldOrientedDirectAngleSim = robot.getDrivebase().simDriveCommand(
          () -> -MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
          () -> -MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
          () -> -MathUtil.applyDeadband(rotationController.getRawAxis(0), OperatorConstants.RIGHT_X_DEADBAND));

      robot.getDrivebase().setDefaultCommand(
          !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  private void SetupOperatorCommands()
  {
    // Intake sequence: extend elevator, lower deck, and intake
    XBOX_RT.whileTrue(
      new SetElevatorPosition(robot.getElevator(), ElevatorPositions.intake).andThen
      (
        new SetDeckPosition(robot.getDeck(), DeckPositions.intake)
        .alongWith(new RunIntakeCommand(robot.getIntake()))
      )
      .andThen(new SetDeckPosition(robot.getDeck(), DeckPositions.home))
      .andThen(new SetElevatorPosition(robot.getElevator(), ElevatorPositions.zero))
    )
    .onFalse
    (
      new  SetDeckPosition(robot.getDeck(), DeckPositions.home)
      .andThen(new SetElevatorPosition(robot.getElevator(), ElevatorPositions.zero))
    );        

    // Outtake: Spits out the note
    XBOX_LT.whileTrue(new RunOuttakeCommand(robot.getIntake()));

    // Close shot
    XBOX_A.whileTrue(new SetDeckPosition(robot.getDeck(), DeckPositions.closeup))
        .whileTrue(new RevShooter(robot.getShooter(), ShooterConfig.closeLeftSpeed, ShooterConfig.closeRightSpeed))
        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

    // Medium shot
    XBOX_B.whileTrue(new SetDeckPosition(robot.getDeck(), DeckPositions.podium))
        .whileTrue(new RevShooter(robot.getShooter(), ShooterConfig.podiumLeftSpeed, ShooterConfig.podiumRightSpeed))
        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

    // Far shot
    XBOX_Y.whileTrue(new SetDeckPosition(robot.getDeck(), DeckPositions.backline))
        .whileTrue(new RevShooter(robot.getShooter(), ShooterConfig.farLeftSpeed, ShooterConfig.farRightSpeed))
        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

  }

  private void SetupDriverCommands()
  {
      // Run intake to shoot note
    DRIVE_TRIG.whileTrue(new ShootNote(robot.getIntake()));

    // Preclimb position
    DRIVE_B3.onTrue(new SetClimberPosition(robot.getClimber(), ClimberPositions.preclimb)
    .alongWith(new SetDeckPosition(robot.getDeck(), DeckPositions.preClimb)));

    // Climb
    DRIVE_B2.onTrue
    (
      new SetClimberPosition(robot.getClimber(), ClimberPositions.midClimb)
      .andThen
      (
        new SetClimberPosition(robot.getClimber(), ClimberPositions.climb)
        .alongWith(new SetDeckPosition(robot.getDeck(), DeckPositions.climb))
      )
    );

    // Zero Gyro
    ROTATE_TRIG.onTrue(new InstantCommand(robot.getDrivebase()::zeroGyro));

    // Jog Climber Up
    DRIVE_B5.whileTrue(new JogClimberUp(robot.getClimber()));

    // Jog Climber Down
    DRIVE_B4.whileTrue(new JogClimberDown(robot.getClimber()));

  }


}
