
package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ResetRobot;
import frc.robot.commands.climber.JogClimberDown;
import frc.robot.commands.climber.JogClimberUp;
import frc.robot.commands.climber.SetClimberPosition;
import frc.robot.commands.deck.AutoDeckAim;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.intake.AmpDeckCommand;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.ScoreTrap;
import frc.robot.commands.intake.ShootNote;
import frc.robot.commands.sequential.RetractIntakeSequence;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.commands.stabilizer.SetStabilizerPosition;
import frc.robot.subsystems.climber.ClimberPositions;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.shooter.ShooterConfig;
import frc.robot.subsystems.stabilizer.StabilizerPositions;


public class TeleopCommands 
{


  private enum ControllerButton
  {
    A (2),
    B (3),
    X (1),
    Y (4),

    LeftTrigger (7),
    RightTrigger (8),

    LeftBumper(5),
    RightBumper(6);


    public final int value;

    ControllerButton(int val)
    {
      value = val;
    }
  }

  RobotContainer robot;
 
  // Driver Joysticks
  CommandJoystick driverController ;
  CommandJoystick rotationController;

  // Operator Xbox Controller
  XboxController operatorXbox;
  
  // Buttons for Xbox Controller
  private Trigger jogIntake;
  private Trigger runIntake; // Intake
  private Trigger jogOutake; // Outtake
  private Trigger closeShot; // Close shot
  private Trigger mediumShot; // Medium shot
  private Trigger farShot; // Far shot
  private Trigger preAmp; // Ready for amp score
  private Trigger autoAim; //autoAimShooter

  // Buttons for Drive Joystick
  private Trigger shoot;
  private Trigger climb;
  private Trigger preclimb;
  private Trigger climbDown;
  private Trigger climbUp;
    
  // Buttons for Roation Joystick
  private Trigger zeroGyro;
  private Trigger autoSteer;

  //Button for Roborio
  private Trigger reset;

    
  public TeleopCommands(RobotContainer container, BooleanSupplier isRed)
  {
      robot = container;
      ConfigureBindings();
      SetupDefaultCommands();
      SetupOperatorCommands();
      SetupDriverCommands(isRed);
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
      
      runIntake = new JoystickButton(operatorXbox, ControllerButton.RightBumper.value);
      jogOutake = new JoystickButton(operatorXbox, ControllerButton.LeftTrigger.value);
      jogIntake = new JoystickButton(operatorXbox, ControllerButton.RightTrigger.value);
      preAmp = new JoystickButton(operatorXbox, ControllerButton.LeftBumper.value);

      closeShot = new JoystickButton(operatorXbox, ControllerButton.A.value);
      mediumShot = new JoystickButton(operatorXbox, ControllerButton.B.value); 
      farShot = new JoystickButton(operatorXbox, ControllerButton.Y.value);
      autoAim = new JoystickButton(operatorXbox, ControllerButton.X.value);


      // driver
      shoot = driverController.button(1);
      climb = driverController.button(2);
      preclimb = driverController.button(3);
      climbDown = driverController.button(4);
      climbUp = driverController.button(5);
      

      zeroGyro = rotationController.button(1);
      autoSteer = rotationController.button(2);

      //RoboRio
      reset = new Trigger(() -> RobotController.getUserButton());

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
    runIntake.whileTrue(
      new SetDeckPosition(robot.getDeck(), DeckPositions.home).
      andThen(new SetElevatorPosition(robot.getElevator(), ElevatorPositions.intake)).andThen
      (
        new SetDeckPosition(robot.getDeck(), DeckPositions.intake)
        .alongWith(new RunIntakeCommand(robot.getIntake()))
      )
      .andThen(new SetDeckPosition(robot.getDeck(), DeckPositions.home)
      .alongWith(new SetElevatorPosition(robot.getElevator(), ElevatorPositions.zero))
      )
    )
    .onFalse
    (
      new  SetDeckPosition(robot.getDeck(), DeckPositions.home)
      .alongWith(new SetElevatorPosition(robot.getElevator(), ElevatorPositions.zero))
    );        

    jogIntake.whileTrue(new RunIntakeCommand(robot.getIntake()));

    // Outtake: Spits out the note
    jogOutake.whileTrue(new ScoreTrap(robot.getIntake()));

    // Close shot
    closeShot.whileTrue(new SetDeckPosition(robot.getDeck(), DeckPositions.closeup)
        .alongWith(new RevShooter(robot.getShooter(), ShooterConfig.closeLeftSpeed, ShooterConfig.closeRightSpeed)))
        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

    // Medium shot
    mediumShot.whileTrue(new SetDeckPosition(robot.getDeck(), DeckPositions.podium)
        .alongWith(new RevShooter(robot.getShooter(), ShooterConfig.podiumLeftSpeed, ShooterConfig.podiumRightSpeed)))
        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

    // Far shot
    farShot.whileTrue(new SetDeckPosition(robot.getDeck(), DeckPositions.backline)
        .alongWith(new RevShooter(robot.getShooter(), ShooterConfig.farLeftSpeed, ShooterConfig.farRightSpeed)))
        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

    preAmp.whileTrue(new SetDeckPosition(robot.getDeck(), DeckPositions.amp)
    .alongWith(new AmpDeckCommand(robot.getIntake())));
    
    autoAim.whileTrue(new AutoDeckAim(robot.getDeck())
        .alongWith(new RevShooter(robot.getShooter(), ShooterConfig.podiumLeftSpeed, ShooterConfig.podiumRightSpeed)))
        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

  }

  private void SetupDriverCommands(BooleanSupplier isRed)
  {
    //Auto Aim Commands
    Command driveFieldOrientedAutoAim = robot.getDrivebase().driveCommand(
          () -> -MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
          () -> -MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
          () -> -robot.limelight_aim_proportional());

      // Run intake to shoot note
    shoot.whileTrue(new ShootNote(robot.getIntake()));

    // Preclimb position
    preclimb.onTrue(new SetStabilizerPosition(robot.getStabilizer(), StabilizerPositions.climb)
    .andThen
      (new SetDeckPosition(robot.getDeck(), DeckPositions.preClimb)));

    // Climb
    climb.onTrue
    (
      new SetClimberPosition(robot.getClimber(), ClimberPositions.midClimb)
      .andThen
      (
        new SetClimberPosition(robot.getClimber(), ClimberPositions.climb)
        .alongWith(new SetDeckPosition(robot.getDeck(), DeckPositions.climb))
      )
      .andThen
      (
        new SetElevatorPosition(robot.getElevator(), 20)
      )
      .andThen
      (
        new AmpDeckCommand(robot.getIntake())
      )
      .andThen
      (
        new SetElevatorPosition(robot.getElevator(), 25)
      )
      .andThen
      (
        new ScoreTrap(robot.getIntake())
      .alongWith(new SetElevatorPosition(robot.getElevator(), 25)
      .alongWith(new SetDeckPosition(robot.getDeck(), 135)))
      )
    );

    // Zero Gyro
    zeroGyro.onTrue(new InstantCommand(robot.getDrivebase()::zeroGyro));

    // Jog Climber Up
    climbUp.whileTrue(new JogClimberUp(robot.getClimber()));

    // Jog Climber Down
    climbDown.whileTrue(new JogClimberDown(robot.getClimber()));

    autoSteer.whileTrue(driveFieldOrientedAutoAim);


    reset.onTrue(new ResetRobot(robot, isRed));



  }

  public Command onTeleopInit()
  {
      return new RetractIntakeSequence(robot.getDeck(), robot.getElevator()).andThen(new InstantCommand(()->robot.getShooter().stop()));
  }
 


}
