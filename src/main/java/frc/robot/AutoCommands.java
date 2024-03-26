package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShootParams;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.intake.SimpleShootNote;
import frc.robot.commands.sequential.Auto1Shoot;
import frc.robot.commands.sequential.AutoNotePickup;
import frc.robot.commands.sequential.AutoShoot;
import frc.robot.commands.sequential.IntakeSequence;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.elevator.ElevatorPositions;

public class AutoCommands {

        private RobotContainer robot;

        private Command auto1;
        private Command auto2;
        private Command auto3;
        private Command auto4;
        private Command auto5;
        private Command auto6;
        private Command auto7;

        public AutoCommands(RobotContainer robot) {
                this.robot = robot;

                // legacy commands

                NamedCommands.registerCommand("AutoAimRotate", robot.getDrivebase().driveCommand(
                                () -> 0, () -> 0,
                                () -> -robot.limelight_aim_proportional()));

                double shooterDelay = 0.75;

                NamedCommands.registerCommand("startAuto1", new ParallelRaceGroup(
                                new RevShooter(robot.getShooter(), ShootParams.Auto1.getLeftSpeed(),
                                                ShootParams.Auto1.getRightSpeed()),
                                new WaitCommand(shooterDelay)));
                NamedCommands.registerCommand("startAuto2", new ParallelRaceGroup(
                                new RevShooter(robot.getShooter(), ShootParams.Auto2.getLeftSpeed(),
                                                ShootParams.Auto2.getRightSpeed(), false),
                                new WaitCommand(shooterDelay)));
                NamedCommands.registerCommand("startAuto3", new ParallelRaceGroup(
                                new RevShooter(robot.getShooter(), ShootParams.Auto3.getLeftSpeed(),
                                                ShootParams.Auto3.getRightSpeed()),
                                new WaitCommand(shooterDelay)));
                NamedCommands.registerCommand("startAuto4",
                                new RevShooter(robot.getShooter(), ShootParams.Auto2.getLeftSpeed(),
                                                ShootParams.Auto2.getRightSpeed(), false)
                                                .alongWith(new SetDeckPosition(robot.getDeck(), DeckPositions.podium)));
                NamedCommands.registerCommand("startAuto5", new ParallelRaceGroup(
                                new RevShooter(robot.getShooter(), 1.0,
                                                0.5, false),
                                new WaitCommand(shooterDelay)));

                NamedCommands.registerCommand("extendIntake",
                                new SetDeckPosition(robot.getDeck(), DeckPositions.intake)
                                                .alongWith(new SetElevatorPosition(robot.getElevator(),
                                                                ElevatorPositions.intake)));
                NamedCommands.registerCommand("getPiece",
                                new IntakeSequence(robot.getIntake(), robot.getDeck(), robot.getElevator(), robot.getLeds()));
                NamedCommands.registerCommand("getPieceAuto",
                                new AutoNotePickup(robot.getDeck(), robot.getElevator(), robot.getIntake(),
                                                robot.getDrivebase(), robot.getLeds()));

                NamedCommands.registerCommand("shootAuto2",
                                new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(), ShootParams.Auto2,
                                                false, robot.getLimelight()));
                NamedCommands.registerCommand("shootAuto1",
                                new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(), ShootParams.Auto1,
                                                false, robot.getLimelight()));
                NamedCommands.registerCommand("shootAuto3",
                                new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(), ShootParams.Auto3,
                                                false, robot.getLimelight()));
                NamedCommands.registerCommand("finishAuto", new InstantCommand(() -> robot.getShooter().stop()));

                NamedCommands.registerCommand("shootAutoAim",
                                new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(),
                                                ShootParams.Podium, true, robot.getLimelight()));
                NamedCommands.registerCommand("shootAuto1Aim", 
                        new Auto1Shoot(robot.getDeck(), robot.getIntake(),
                                                ShootParams.Podium, true, robot.getLimelight(), robot.getShooter()));

                NamedCommands.registerCommand("simpleShoot", new SimpleShootNote(robot.getIntake()));

                NamedCommands.registerCommand("deckHome", new SetDeckPosition(robot.getDeck(), DeckPositions.home));
                NamedCommands.registerCommand("setDeckPosAuto1", new SetDeckPosition(robot.getDeck(), DeckPositions.podium));
                NamedCommands.registerCommand("setDeckPosAuto5", new SetDeckPosition(robot.getDeck(), 26));
                NamedCommands.registerCommand("setDeckPosAuto5_1", new SetDeckPosition(robot.getDeck(), 24));
                NamedCommands.registerCommand("setDeckPosAuto5_End", new SetDeckPosition(robot.getDeck(), 30));
                NamedCommands.registerCommand("setDeckPosAuto6_End", new SetDeckPosition(robot.getDeck(), 20));
                
                auto1 = robot.getDrivebase().getAutoCommand("Auto1");
                auto2 = robot.getDrivebase().getAutoCommand("Auto2");
                auto3 = robot.getDrivebase().getAutoCommand("Auto3");
                auto4 = robot.getDrivebase().getAutoCommand("Auto4");
                auto5 = robot.getDrivebase().getAutoCommand("Auto5");
                auto6 = robot.getDrivebase().getAutoCommand("Auto6");
                auto7 = robot.getDrivebase().getAutoCommand("Auto7");
                
                
        }

        public Command SelectAuto() {
                switch (robot.getThumbwheel().getValue()) {

                        case 1:
                                return auto1;
                        case 2:
                                return auto2;
                        case 3:
                                return auto3;
                        case 4:
                                return auto4;
                        case 5:
                                return auto5;
                        case 6:
                                return auto6;
                        case 7:
                                return auto7;

                        default:
                                return new InstantCommand();

                }
        }

}
