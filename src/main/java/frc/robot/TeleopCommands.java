
package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
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
import frc.robot.commands.climber.SetClimberPosition;
import frc.robot.commands.deck.AutoDeckAim;
import frc.robot.commands.deck.JogDeck;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.elevator.JogElevator;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.intake.AmpDeckCommand;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.RunOuttakeCommand;
import frc.robot.commands.intake.ScoreTrap;
import frc.robot.commands.intake.ShootNote;
import frc.robot.commands.intake.TrapOuttake;
import frc.robot.commands.sequential.RetractIntakeSequence;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.commands.stabilizer.SetStabilizerPosition;
import frc.robot.subsystems.climber.ClimberPositions;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.shooter.ShooterConfig;
import frc.robot.subsystems.stabilizer.StabilizerPositions;
import edu.wpi.first.wpilibj2.command.WaitCommand;


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
    RightBumper(6),

    Select(10);


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

  //buttonboard
  private GenericHID buttonBoard;
  
  // Buttons for Xbox Controller
  private Trigger jogIntake;
  private Trigger runIntake; // Intake
  private Trigger jogOutake; // Outtake
  private Trigger runOuttake;
  private Trigger closeShot; // Close shot
  private Trigger mediumShot; // Medium shot
  private Trigger preAmp; // Ready for amp score
  private Trigger autoAim; //autoAimShooter

  // Buttons for Button Board
  private Trigger jogDeckUp;
  private Trigger jogDeckDown;
  private Trigger jogElevatorOut;
  private Trigger jogElevatorIn;
  private Trigger scoreTrap;

  // Buttons for Drive Joystick
  private Trigger shoot;
  private Trigger shootAlt;
  private Trigger climb;
  private Trigger preclimb;
    
  // Buttons for Roation Joystick
  private Trigger zeroGyro;
  private Trigger autoSteer;
  private Trigger autoSteerAlt;

  //Button for Roborio
  private Trigger reset;

  //Go home
  private Trigger home;

    
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
      buttonBoard = new GenericHID(3);

      // triggers

      // TODO: I think the xbox controller has an enum for buttons? if not, create one, and rename these variables
      // so that they have to do with the action they get bound to rather than their button. That way we don't need comments
      // explaining what they do

      // operator
      
      runIntake = new JoystickButton(operatorXbox, ControllerButton.RightBumper.value);
      jogOutake = new JoystickButton(operatorXbox, ControllerButton.LeftTrigger.value);
      jogIntake = new JoystickButton(operatorXbox, ControllerButton.RightTrigger.value);
      runOuttake = new JoystickButton(operatorXbox, ControllerButton.LeftBumper.value);
      
      preAmp = new JoystickButton(operatorXbox, ControllerButton.Y.value);
      closeShot = new JoystickButton(operatorXbox, ControllerButton.A.value);
      mediumShot = new JoystickButton(operatorXbox, ControllerButton.B.value); 
      autoAim = new JoystickButton(operatorXbox, ControllerButton.X.value);

      // Button Board

      jogDeckUp = new JoystickButton(buttonBoard, 11);
      jogDeckDown = new JoystickButton(buttonBoard, 3);
      jogElevatorIn = new JoystickButton(buttonBoard, 1);
      jogElevatorOut = new JoystickButton(buttonBoard, 10);
      scoreTrap = new JoystickButton(buttonBoard, 12);


      // driver
      shoot = driverController.button(1);
      shootAlt = driverController.button(11);
      climb = driverController.button(2);
      preclimb = driverController.button(3);
      

      zeroGyro = rotationController.button(1);
      autoSteer = rotationController.button(2);
      autoSteerAlt = rotationController.button(6);

      //RoboRio
      reset = new Trigger(() -> RobotController.getUserButton());

      home = new JoystickButton(operatorXbox, ControllerButton.Select.value);

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

    runOuttake.whileTrue(new RunOuttakeCommand(robot.getIntake()));

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

    preAmp.whileTrue(new SetDeckPosition(robot.getDeck(), DeckPositions.amp)
    .alongWith(new AmpDeckCommand(robot.getIntake())));
    
    autoAim.whileTrue(new AutoDeckAim(robot.getDeck())
        .alongWith(new RevShooter(robot.getShooter(), ShooterConfig.podiumLeftSpeed, ShooterConfig.podiumRightSpeed)))
        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

    home.onTrue
    (
      new SetElevatorPosition(robot.getElevator(), 0)
    .andThen
    (
      new SetDeckPosition(robot.getDeck(), 0)
    )
    .andThen
    (
      new SetStabilizerPosition(robot.getStabilizer(),0)
    )
    );


    //  button board commands

    // jog Deck Up
    jogDeckUp.whileTrue(new JogDeck(robot.getDeck(), 1));
    //jog Deck Down
    jogDeckDown.whileTrue(new JogDeck(robot.getDeck(), -1));

    //jog elevator out
    jogElevatorOut.whileTrue(new JogElevator(robot.getElevator(), 1));
    //jog elevator in
    jogElevatorIn.whileTrue(new JogElevator(robot.getElevator(), -1));

    //score trap
    scoreTrap.whileTrue(new ScoreTrap(robot.getIntake()));


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
    shootAlt.whileTrue(new ShootNote(robot.getIntake()));

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
        new TrapOuttake(robot.getIntake())
      )
      .andThen
      (
        new SetClimberPosition(robot.getClimber(), ClimberPositions.climb)
        .alongWith(new SetDeckPosition(robot.getDeck(), 110))
      )
      
      .andThen
      (
        new WaitCommand(.25)
        .alongWith(new SetDeckPosition(robot.getDeck(), 130)
        .alongWith(new SetElevatorPosition(robot.getElevator(),38)))
      )
      
      
      
      
    );

    // Zero Gyro
    zeroGyro.onTrue(new InstantCommand(robot.getDrivebase()::zeroGyro));
    

    autoSteer.whileTrue(driveFieldOrientedAutoAim);
    autoSteerAlt.whileTrue(driveFieldOrientedAutoAim);


    reset.onTrue(new ResetRobot(robot, isRed));



  }

  public Command onTeleopInit()
  {
      return new RetractIntakeSequence(robot.getDeck(), robot.getElevator()).andThen(new InstantCommand(()->robot.getShooter().stop()));
  }
 


}
