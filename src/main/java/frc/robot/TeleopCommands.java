
package frc.robot;

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
import frc.robot.commands.climber.SetClimberPosition;
import frc.robot.commands.deck.AutoDeckAim;
import frc.robot.commands.deck.JogDeck;
import frc.robot.commands.deck.SetDeckMaxOutput;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.elevator.JogElevator;
import frc.robot.commands.elevator.ResetElevatorPosition;
import frc.robot.commands.elevator.SetElevatorAmpLimit;
import frc.robot.commands.elevator.SetElevatorMaxOutput;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.intake.AmpDeckCommand;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.RunOuttakeCommand;
import frc.robot.commands.intake.ScoreTrap;
import frc.robot.commands.intake.SimpleShootNote;
import frc.robot.commands.intake.TrapOuttake;
import frc.robot.commands.reset.ResetRobot;
import frc.robot.commands.sequential.AutoNotePickup;
import frc.robot.commands.sequential.IntakeSequenceTeleop;
import frc.robot.commands.sequential.RetractIntakeSequence;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.commands.stabilizer.SetStabilizerPosition;
import frc.robot.commands.swervedrive.drivebase.NoteAutoAim;
import frc.robot.commands.swervedrive.drivebase.SpeakerAutoAim;
import frc.robot.subsystems.climber.ClimberPositions;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.shooter.ShooterConfig;
import frc.robot.subsystems.stabilizer.StabilizerPositions;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TeleopCommands {

        private enum ControllerButton {
                A(2),
                B(3),
                X(1),
                Y(4),

                LeftTrigger(7),
                RightTrigger(8),

                LeftBumper(5),
                RightBumper(6),

                Select(10),
                Back(9);

                public final int value;

                ControllerButton(int val) {
                        value = val;
                }
        }


        RobotContainer robot;

        // Driver Joysticks
        CommandJoystick driverController;
        CommandJoystick rotationController;

        // Operator Xbox Controller
        XboxController operatorXbox;

        // Buttonboard
        private GenericHID buttonBoard;

        // Buttons for Xbox Controller
        private Trigger jogIntake;
        private Trigger runIntake;
        private Trigger jogOutake;
        private Trigger runOuttake;
        private Trigger closeShot;
        private Trigger mediumShot;
        private Trigger preAmp;
        private Trigger autoAim;
        private Trigger home;
        private Trigger operatorAutoNotePickup;

        // Buttons for Button Board
        private Trigger jogDeckUp;
        private Trigger jogDeckDown;
        private Trigger jogElevatorOut;
        private Trigger jogElevatorIn;
        private Trigger scoreTrap;
        private Trigger sourceForward;
        private Trigger middleForward;

        // Buttons for Drive Joystick
        private Trigger climb;
        private Trigger preclimb;
        private Trigger simpleshoot;
        private Trigger simpleshootAlt;
        private Trigger jogClimberUp;
        private Trigger jogClimberDown;
        private Trigger autoNotePickup;

        // Buttons for Roation Joystick
        private Trigger zeroGyro;
        private Trigger autoSteer;
        private Trigger autoSteerAlt;
        private Trigger notePickup;
        private Trigger headingMode;

        // Button for Roborio
        private Trigger reset;
        private Trigger zeroElevator;

        public TeleopCommands(RobotContainer container) {
                robot = container;
                ConfigureBindings();
                SetupDefaultCommands();
                SetupOperatorCommands();
                SetupDriverCommands();
        }

        private void ConfigureBindings() {

                driverController = new CommandJoystick(0);
                rotationController = new CommandJoystick(1);
                operatorXbox = new XboxController(2);
                buttonBoard = new GenericHID(3);

                // TRIGGERS

                // OPERATOR
                runIntake = new JoystickButton(operatorXbox, ControllerButton.RightBumper.value);
                jogOutake = new JoystickButton(operatorXbox, ControllerButton.LeftTrigger.value);
                jogIntake = new JoystickButton(operatorXbox, ControllerButton.RightTrigger.value);
                runOuttake = new JoystickButton(operatorXbox, ControllerButton.LeftBumper.value);
                preAmp = new JoystickButton(operatorXbox, ControllerButton.Y.value);
                closeShot = new JoystickButton(operatorXbox, ControllerButton.A.value);
                mediumShot = new JoystickButton(operatorXbox, ControllerButton.B.value);
                autoAim = new JoystickButton(operatorXbox, ControllerButton.X.value);
                home = new JoystickButton(operatorXbox, ControllerButton.Select.value);
                zeroElevator = new JoystickButton(operatorXbox, ControllerButton.Back.value);

                // BUTTON BOARD
                jogDeckUp = new JoystickButton(buttonBoard, 11);
                jogDeckDown = new JoystickButton(buttonBoard, 3);
                jogElevatorIn = new JoystickButton(buttonBoard, 1);
                jogElevatorOut = new JoystickButton(buttonBoard, 10);
                scoreTrap = new JoystickButton(buttonBoard, 12);
                sourceForward = new JoystickButton(buttonBoard, 8);
                middleForward = new JoystickButton(buttonBoard, 9);
                operatorAutoNotePickup = new JoystickButton(buttonBoard, 6);

                // DRIVER
                simpleshoot = driverController.button(1);
                simpleshootAlt = driverController.button(11);
                climb = driverController.button(2);
                preclimb = driverController.button(3);
                jogClimberUp = driverController.button(5);
                jogClimberDown = driverController.button(4);
                autoNotePickup = driverController.button(10);

                // ROTATION
                zeroGyro = rotationController.button(1);
                autoSteer = rotationController.button(2);
                autoSteerAlt = rotationController.button(6);
                notePickup = rotationController.button(7);
                headingMode = rotationController.button(3);

                // ROBORIO
                reset = new Trigger(() -> RobotController.getUserButton());

        }

        private void SetupDefaultCommands() {
                // Regualar drive mode
                Command driveFieldOrientedDirectAngle = robot.getDrivebase().driveCommand(
                                () -> -MathUtil.applyDeadband(driverController.getY(),
                                                OperatorConstants.LEFT_Y_DEADBAND),
                                () -> -MathUtil.applyDeadband(driverController.getX(),
                                                OperatorConstants.LEFT_X_DEADBAND),
                                () -> -MathUtil.applyDeadband(rotationController.getRawAxis(0),
                                                OperatorConstants.RIGHT_X_DEADBAND));

                // Simulation drive mode
                Command driveFieldOrientedDirectAngleSim = robot.getDrivebase().simDriveCommand(
                                () -> -MathUtil.applyDeadband(driverController.getY(),
                                                OperatorConstants.LEFT_Y_DEADBAND),
                                () -> -MathUtil.applyDeadband(driverController.getX(),
                                                OperatorConstants.LEFT_X_DEADBAND),
                                () -> -MathUtil.applyDeadband(rotationController.getRawAxis(0),
                                                OperatorConstants.RIGHT_X_DEADBAND));

                robot.getDrivebase().setDefaultCommand(
                                !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle
                                                : driveFieldOrientedDirectAngleSim);
        }

        private void SetupOperatorCommands() {

                // Intake sequence: extend elevator, lower deck, and intake
                runIntake.whileTrue
                (
                        new IntakeSequenceTeleop(robot.getIntake(), robot.getDeck(), robot.getElevator(), robot.getLeds())
                )
                .onFalse
                (
                        new SetElevatorAmpLimit(robot.getElevator(), 30)
                        .alongWith
                        (
                                new SetDeckPosition(robot.getDeck(), DeckPositions.home)
                        )
                        .andThen
                        (
                                new SetElevatorPosition(robot.getElevator(),ElevatorPositions.zero)
                        )
                );

                jogIntake.whileTrue(new RunIntakeCommand(robot.getIntake(), robot.getLeds()));

                runOuttake.whileTrue(new RunOuttakeCommand(robot.getIntake()));

                // Outtake: Spits out the note
                jogOutake.whileTrue(new ScoreTrap(robot.getIntake()));

                // Close shot
                closeShot.whileTrue(new SetDeckPosition(robot.getDeck(), DeckPositions.closeup)
                                .alongWith(new RevShooter(robot.getShooter(), ShooterConfig.closeLeftSpeed,
                                                ShooterConfig.closeRightSpeed)))
                                .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

                // Medium shot
                mediumShot.whileTrue(new SetDeckPosition(robot.getDeck(), DeckPositions.podium)
                                .alongWith(new RevShooter(robot.getShooter(), ShooterConfig.podiumLeftSpeed,
                                                ShooterConfig.podiumRightSpeed)))
                                .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

                preAmp.whileTrue(new SetDeckPosition(robot.getDeck(), DeckPositions.amp)
                                .alongWith(new AmpDeckCommand(robot.getIntake())));

                autoAim.whileTrue(new AutoDeckAim(robot.getDeck(), robot.getLimelight())
                                .alongWith(new RevShooter(robot.getShooter(), ShooterConfig.podiumLeftSpeed,
                                                ShooterConfig.podiumRightSpeed)))
                                .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

                home.onTrue(new SetElevatorPosition(robot.getElevator(), 0)
                                        .andThen(new SetDeckPosition(robot.getDeck(), 0))
                                        .andThen(new SetStabilizerPosition(robot.getStabilizer(), 0)));

                zeroElevator.onTrue(new ResetElevatorPosition(robot.getElevator()));

                // Button Board Commands

                // Jog Deck Up
                jogDeckUp.whileTrue(new JogDeck(robot.getDeck(), 2));
                // Jog Deck Down
                jogDeckDown.whileTrue(new JogDeck(robot.getDeck(), -2));

                // Jog Elevator Out
                jogElevatorOut.whileTrue(new JogElevator(robot.getElevator(), 1));
                // Jog Elevator In
                jogElevatorIn.whileTrue(new JogElevator(robot.getElevator(), -1));

                // Score Trap
                scoreTrap.whileTrue(new ScoreTrap(robot.getIntake()));

                //source forwarding
                sourceForward.whileTrue(new SetDeckPosition(robot.getDeck(), 3)
                        .alongWith(new RevShooter(robot.getShooter(),0.5 ,0.4)))
                        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

                //middle forwarding
                middleForward.whileTrue(new SetDeckPosition(robot.getDeck(), 3)
                        .alongWith(new RevShooter(robot.getShooter(),0.55 ,0.45)))
                        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

                //auto note pickup
                operatorAutoNotePickup.whileTrue(
                        new AutoNotePickup(robot.getDeck(), robot.getElevator(), robot.getIntake(),robot.getDrivebase(), robot.getLeds()))
                        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home)
                                .alongWith(new SetElevatorPosition(robot.getElevator(), ElevatorPositions.zero)));

        }

        private void SetupDriverCommands() {

                Command driveWithHeading = robot.getDrivebase().driveCommand(
                        () -> -MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
                        () -> -MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
                        () -> -rotationController.getX(),
                        () -> rotationController.getY());


                // runOuttake.whileTrue(new RunOuttakeCommand(robot.getIntake()));
                simpleshoot.onTrue(new SimpleShootNote(robot.getIntake()));
                simpleshootAlt.onTrue(new SimpleShootNote(robot.getIntake()));

                // Preclimb position
                preclimb.onTrue(new SetStabilizerPosition(robot.getStabilizer(), StabilizerPositions.climb)
                                .andThen(new SetDeckPosition(robot.getDeck(), DeckPositions.preClimb)));

                // Climb
                climb.onTrue(
                        new SetClimberPosition(robot.getClimber(), ClimberPositions.midClimb)
                                .andThen
                                (
                                        new TrapOuttake(robot.getIntake())
                                )
                                .andThen
                                (
                                        new SetClimberPosition(robot.getClimber(), ClimberPositions.climb)
                                        .alongWith(new SetDeckPosition(robot.getDeck(), 91))
                                        .alongWith(new WaitCommand(0.5))
                                )
                                .andThen
                                (
                                        new SetElevatorPosition(robot.getElevator(), 22.3)
                                        .alongWith(new WaitCommand(0.5))
                                )
                                .andThen
                                (
                                        new SetDeckPosition(robot.getDeck(), 108)
                                        .alongWith(new SetElevatorPosition(robot.getElevator(), 45))
                                        .alongWith(new WaitCommand(.5))
                                )
                                .andThen
                                (
                                        new SetDeckMaxOutput(robot.getDeck(), 0.6)
                                        .alongWith(new SetElevatorMaxOutput(robot.getElevator(), 0.6))
                                )
                                .andThen
                                (
                                        new SetDeckPosition(robot.getDeck(), 118)   
                                        .alongWith(new WaitCommand(.5))
                                )
                                .andThen
                                (
                                        new SetDeckPosition(robot.getDeck(), 102)
                                )
                                .andThen(
                                        new SetDeckMaxOutput(robot.getDeck(), 1.0)
                                        .alongWith(new SetElevatorMaxOutput(robot.getElevator(), 1.0))
                                )

                );

                jogClimberUp.whileTrue(new SetClimberPosition(robot.getClimber(), ClimberPositions.climb));
                jogClimberDown.whileTrue(new SetClimberPosition(robot.getClimber(), 0));

                // Zero Gyro
                zeroGyro.onTrue(new InstantCommand(robot.getDrivebase()::zeroGyro));

                // Auto Aim Swerve
                autoSteer.whileTrue(new SpeakerAutoAim(robot.getDrivebase(), driverController, rotationController));
                autoSteerAlt.whileTrue(new SpeakerAutoAim(robot.getDrivebase(), driverController, rotationController));

                notePickup.whileTrue(new NoteAutoAim(robot.getDrivebase(), driverController, rotationController));
                autoNotePickup.whileTrue(
                        new AutoNotePickup(robot.getDeck(), robot.getElevator(), robot.getIntake(),robot.getDrivebase(),robot.getLeds()))
                        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home)
                                .alongWith(new SetElevatorPosition(robot.getElevator(), ElevatorPositions.zero)));

                headingMode.whileTrue(driveWithHeading);

                // Reset Robot
                reset.onTrue(new ResetRobot(robot));

        }

        public Command onTeleopInit() {
                return new RetractIntakeSequence(robot.getDeck(), robot.getElevator())
                                .andThen(new InstantCommand(() -> robot.getShooter().stop()));
        }

}
