
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.climber.SetClimberPosition;
import frc.robot.commands.climber.StopClimber;
import frc.robot.commands.deck.AutoDeckAim;
import frc.robot.commands.deck.JogDeck;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.elevator.JogElevator;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.intake.AmpDeckCommand;
import frc.robot.commands.intake.IndexNote;
import frc.robot.commands.intake.ScoreAmp;
import frc.robot.commands.intake.ScoreTrap;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.intake.TeleopShoot;
import frc.robot.commands.limelight.ChangePipeline;
import frc.robot.commands.reset.ResetRobot;
import frc.robot.commands.sequential.AutoAmpSequence;
import frc.robot.commands.sequential.AutoNotePickup;
import frc.robot.commands.sequential.IntakeSequenceAutoAim;
import frc.robot.commands.sequential.IntakeSequenceTeleop;
import frc.robot.commands.sequential.RetractIntakeSequence;
import frc.robot.commands.sequential.TeleopAimShoot;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.commands.shooter.ToggleForward;
import frc.robot.commands.stabilizer.SetStabilizerPosition;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AmpAutoAim;
import frc.robot.commands.swervedrive.drivebase.RobotOrientedMode;
import frc.robot.commands.swervedrive.drivebase.SpeakerAutoAim;
import frc.robot.subsystems.climber.ClimberPositions;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.shooter.ShooterConfig;
import frc.robot.subsystems.stabilizer.StabilizerPositions;

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
        private Trigger normalIntake;
        private Trigger closeShot;
        private Trigger mediumShot;
        private Trigger preAmp;
        private Trigger autoAim;
        private Trigger home;

        // Buttons for Button Board
        private Trigger jogDeckUp;
        private Trigger jogDeckDown;
        private Trigger jogElevatorOut;
        private Trigger jogElevatorIn;
        private Trigger scoreTrap;
        private Trigger sourceForward;
        private Trigger middleForward;
        private Trigger intakeAutoAim;
        private Trigger fullAutoShooting;
        private Trigger operatorPreClimb;
        private Trigger raiseStabilizer;

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
        private Trigger autoAmp;

        // Button for Roborio
        private Trigger reset;
        private Trigger zeroEverything;

        public TeleopCommands(RobotContainer container) {
                robot = container;
                ConfigureBindings();
                //SetupDefaultCommands();
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
                normalIntake = new JoystickButton(operatorXbox, ControllerButton.LeftBumper.value);
                preAmp = new JoystickButton(operatorXbox, ControllerButton.Y.value);
                closeShot = new JoystickButton(operatorXbox, ControllerButton.A.value);
                mediumShot = new JoystickButton(operatorXbox, ControllerButton.B.value);
                autoAim = new JoystickButton(operatorXbox, ControllerButton.X.value);
                home = new JoystickButton(operatorXbox, ControllerButton.Select.value);
                zeroEverything = new JoystickButton(operatorXbox, ControllerButton.Back.value);
                

                // BUTTON BOARD
                jogDeckUp = new JoystickButton(buttonBoard, 11);
                jogDeckDown = new JoystickButton(buttonBoard, 3);
                jogElevatorIn = new JoystickButton(buttonBoard, 1);
                jogElevatorOut = new JoystickButton(buttonBoard, 10);
                scoreTrap = new JoystickButton(buttonBoard, 12);
                sourceForward = new JoystickButton(buttonBoard, 8);
                middleForward = new JoystickButton(buttonBoard, 9);
                intakeAutoAim = new JoystickButton(buttonBoard, 2);
                fullAutoShooting = new JoystickButton(buttonBoard, 6);
                operatorPreClimb = new JoystickButton(buttonBoard, 4);
                raiseStabilizer = new JoystickButton(buttonBoard, 7);

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
                autoAmp = rotationController.button(11);

                // ROBORIO
                reset = new Trigger(() -> RobotController.getUserButton());

        }

        public void SetupDefaultCommands() {
                // Regualar drive mode
                Command driveFieldOrientedDirectAngle = robot.getDrivebase().driveCommand(
                                () -> -MathUtil.applyDeadband(driverController.getY(),
                                                OperatorConstants.LEFT_Y_DEADBAND),
                                () -> -MathUtil.applyDeadband(driverController.getX(),
                                                OperatorConstants.LEFT_X_DEADBAND),
                                () -> -MathUtil.applyDeadband(rotationController.getRawAxis(0),
                                                OperatorConstants.RIGHT_X_DEADBAND));

                // ! This is for fixing the issue with the robot being inverted after running the red auto
                Command driveFieldOrientedDirectAngleRed = robot.getDrivebase().driveCommand(
                                () -> MathUtil.applyDeadband(driverController.getY(),
                                                OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(driverController.getX(),
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


                Command driveCommand;

                                var alliance = DriverStation.getAlliance();
                                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) 
                                {
                                        //set driveCommand equal to correct command 
                                        driveCommand = driveFieldOrientedDirectAngle;
                                } 
                                else if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
                                {
                                        driveCommand = driveFieldOrientedDirectAngleRed;      
                                }
                                else 
                                {
                                        //set driveCommand equal to correct command
                                        driveCommand = driveFieldOrientedDirectAngle;  
                                }
                
                robot.getDrivebase().setDefaultCommand(
                                !RobotBase.isSimulation() ? driveCommand
                                : driveFieldOrientedDirectAngleSim);
        }

        private void SetupOperatorCommands() {

                // Intake sequence: extend elevator, lower deck, and intake
                intakeAutoAim.whileTrue(new ScoreAmp(robot.getIntake()));;

                jogIntake.whileTrue(new IndexNote(robot.getIntake()));

                normalIntake.whileTrue
                (
                        new IntakeSequenceTeleop(robot.getIntake(), robot.getDeck(), robot.getElevator(), robot.getLeds())
                        .andThen(new RetractIntakeSequence(robot.getDeck(), robot.getElevator(), robot.getIntake()))
                )
                .onFalse
                (
                        new SetDeckPosition(robot.getDeck(), DeckPositions.home)
                        .alongWith(new SetElevatorPosition(robot.getElevator(), ElevatorPositions.zero))
                        .alongWith(new StopIntake(robot.getIntake()))
                );

                // Outtake: Spits out the note
                //jogOutake.whileTrue(new ScoreAmp(robot.getIntake()));
                jogOutake.whileTrue(new ScoreTrap(robot.getIntake())
                .alongWith(new SetElevatorPosition(robot.getElevator(), 9)))
                .onFalse(new SetElevatorPosition(robot.getElevator(), 0));

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


                preAmp.whileTrue
                (
                        new AutoAmpSequence(robot.getLimelight(), robot.getDeck(), robot.getIntake(), robot.getDrivebase(), driverController, rotationController, robot.getElevator()) 
                )
                .onFalse(new ChangePipeline(robot.getLimelight(), "limelight-intake", 0));

                fullAutoShooting.whileTrue(new AutoDeckAim(robot.getDeck(), robot.getLimelight())
                                .alongWith(new RevShooter(robot.getShooter(), ShooterConfig.podiumLeftSpeed,
                                                ShooterConfig.podiumRightSpeed)))
                                .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));
                
                autoAim.whileTrue(new TeleopAimShoot(robot.getShooter(), robot.getDeck(), robot.getLimelight(), robot.getDrivebase(), driverController, rotationController, robot.getIntake()))
                .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

                home.onTrue(new SetElevatorPosition(robot.getElevator(), 0)
                                        .andThen(new SetDeckPosition(robot.getDeck(), 0))
                                        .andThen(new SetStabilizerPosition(robot.getStabilizer(), 0)));

                zeroEverything.onTrue(new ResetRobot(robot, false));

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
                /*sourceForward.whileTrue(new SetDeckPosition(robot.getDeck(), 3)
                        .alongWith(new RevShooter(robot.getShooter(),0.5 ,0.4)))
                        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));*/

                var alliance = DriverStation.getAlliance();
                boolean isBlue = (alliance.get() == DriverStation.Alliance.Blue);
                
                Command blueForwarding = new AbsoluteDrive(robot.getDrivebase(), 
                        () -> -MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND), 
                        () -> -MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND), 
                        () -> -0.4, 
                        () -> 0.4);

                Command redForwarding = new AbsoluteDrive(robot.getDrivebase(), 
                        () -> MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND), 
                        () -> MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND), 
                        () -> -0.4, 
                        () -> -0.4);

                ConditionalCommand forwardingDrive = new ConditionalCommand(blueForwarding, redForwarding, () -> isBlue);
                
                sourceForward.whileTrue(
                        forwardingDrive.alongWith(
                                new SetDeckPosition(robot.getDeck(), 7)))
                .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));

                //middle forwarding
                /*middleForward.whileTrue(new SetDeckPosition(robot.getDeck(), 3)
                        .alongWith(new RevShooter(robot.getShooter(),0.55 ,0.45)))
                        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home));*/
                // * This is for the forwarding boolean
                middleForward.onTrue(new ToggleForward(robot.getShooter()));

                runIntake.whileTrue(new IntakeSequenceAutoAim(robot.getIntake(), robot.getDeck(), robot.getElevator(), robot.getLeds(), robot.getDrivebase(), driverController, rotationController)
                .andThen(new RetractIntakeSequence(robot.getDeck(), robot.getElevator(), robot.getIntake())))
                .onFalse(
                        new SetDeckPosition(robot.getDeck(), DeckPositions.home)
                        .alongWith(new SetElevatorPosition(robot.getElevator(), ElevatorPositions.zero))
                        .alongWith(new StopIntake(robot.getIntake()))
                );

                operatorPreClimb.onTrue(new SetStabilizerPosition(robot.getStabilizer(), StabilizerPositions.climb)
                                .alongWith(new SetElevatorPosition(robot.getElevator(), ElevatorPositions.zero))
                                .andThen(new SetDeckPosition(robot.getDeck(), DeckPositions.preClimb)));

        raiseStabilizer.onTrue(new SetStabilizerPosition(robot.getStabilizer(), StabilizerPositions.zero));

        }

        private void SetupDriverCommands() {

                Command driveWithHeading = robot.getDrivebase().driveCommand(
                        () -> -MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
                        () -> -MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
                        () -> -rotationController.getX(),
                        () -> rotationController.getY());

                Command driveWithHeadingRed = robot.getDrivebase().driveCommand(
                        () -> MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
                        () -> MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
                        () -> rotationController.getX(),
                        () -> -rotationController.getY());

                Command headingDrive;

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) 
                        {
                                        //set driveCommand equal to correct command 
                                headingDrive = driveWithHeading;
                        } 
                        else if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
                        {
                                headingDrive = driveWithHeadingRed;      
                        }
                        else 
                        {
                                        //set driveCommand equal to correct command
                                headingDrive = driveWithHeading;  
                        }


                // runOuttake.whileTrue(new RunOuttakeCommand(robot.getIntake()));
                simpleshoot.whileTrue(new TeleopShoot(robot.getIntake()));
                simpleshootAlt.whileTrue(new TeleopShoot(robot.getIntake()));

                // Preclimb position
                preclimb.onTrue(new SetStabilizerPosition(robot.getStabilizer(), StabilizerPositions.climb)
                                .alongWith(new SetElevatorPosition(robot.getElevator(), ElevatorPositions.zero))
                                .andThen(new SetDeckPosition(robot.getDeck(), DeckPositions.preClimb))
                                .andThen(new SetClimberPosition(robot.getClimber(), ClimberPositions.zero)));

                // Climb
                climb.onTrue
                (
                        new SetClimberPosition(robot.getClimber(), ClimberPositions.climb)
                        .alongWith
                        (
                                new WaitCommand(1)
                                .andThen(new SetDeckPosition(robot.getDeck(), 120)) //101
                                .alongWith(new AmpDeckCommand(robot.getIntake()))
                        )      
                                
                        .andThen
                        (
                                new SetElevatorPosition(robot.getElevator(), 45)
                                .andThen(new SetDeckPosition(robot.getDeck(), 104))
                        )
                        .andThen
                        (
                                new WaitCommand(0.5)
                        )
                        .andThen
                        (
                                new ScoreTrap(robot.getIntake())
                        ) 
                )
                .onFalse(new StopClimber(robot.getClimber()));


                jogClimberUp.whileTrue(new SetClimberPosition(robot.getClimber(), ClimberPositions.climb))
                .onFalse(new StopClimber(robot.getClimber()));
                jogClimberDown.whileTrue(new SetClimberPosition(robot.getClimber(), 0))
                .onFalse(new StopClimber(robot.getClimber()));

                // Zero Gyro
                zeroGyro.onTrue(new InstantCommand(robot.getDrivebase()::zeroGyro));

                // Auto Aim Swerve
                autoSteer.whileTrue(new SpeakerAutoAim(robot.getDrivebase(), driverController, rotationController));
                autoSteerAlt.whileTrue(new SpeakerAutoAim(robot.getDrivebase(), driverController, rotationController));

                notePickup.whileTrue(new RobotOrientedMode(robot.getDrivebase()));
                autoNotePickup.whileTrue(
                        new AutoNotePickup(robot.getDeck(), robot.getElevator(), robot.getIntake(),robot.getDrivebase(),robot.getLeds()))
                        .onFalse(new SetDeckPosition(robot.getDeck(), DeckPositions.home)
                        .alongWith(new SetElevatorPosition(robot.getElevator(), ElevatorPositions.zero))
                        .alongWith(new StopIntake(robot.getIntake())));

                headingMode.whileTrue(headingDrive);

                // Reset Robot
                reset.onTrue(new ResetRobot(robot, true));

                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) 
                {
                       autoAmp.whileTrue
                        (
                                new ChangePipeline(robot.getLimelight(), "limelight-intake", 1)
                                .andThen(new AmpAutoAim(robot.getDrivebase(), driverController, rotationController))
                        )
                        .onFalse(new ChangePipeline(robot.getLimelight(), "limelight-intake", 0));  
                } 
                else if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
                {
                        autoAmp.whileTrue
                        (
                                new ChangePipeline(robot.getLimelight(), "limelight-intake", 2)
                                .andThen(new AmpAutoAim(robot.getDrivebase(), driverController, rotationController))
                        )
                        .onFalse(new ChangePipeline(robot.getLimelight(), "limelight-intake", 0));    
                }
                
                
                

        }

        public Command setAmpPipelineCommand;

        public Command onTeleopInit() {
                return new SetDeckPosition(robot.getDeck(), DeckPositions.home)
                .alongWith(new SetElevatorPosition(robot.getElevator(), ElevatorPositions.zero))
                .alongWith(new StopIntake(robot.getIntake()))
                                .andThen(new InstantCommand(() -> robot.getShooter().stop()));
        }

}
