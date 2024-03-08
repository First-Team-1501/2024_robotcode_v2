package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShootParams;
import frc.robot.commands.sequential.IntakeSequence;
import frc.robot.commands.sequential.AutoShoot;
import frc.robot.commands.shooter.RevShooter;

public class AutoCommands {

    private RobotContainer robot;

    public AutoCommands(RobotContainer robot) {
        this.robot = robot;

        // legacy commands

        NamedCommands.registerCommand("AutoAimRotate", robot.getDrivebase().driveCommand(
                () -> 0, () -> 0,
                () -> -robot.limelight_aim_proportional()));

        double shooterDelay = 0.75;

        NamedCommands.registerCommand("startAuto1", new ParallelRaceGroup(
                new RevShooter(robot.getShooter(), ShootParams.Auto1.getLeftSpeed(), ShootParams.Auto1.getRightSpeed()),
                new WaitCommand(shooterDelay)));
        NamedCommands.registerCommand("startAuto2", new ParallelRaceGroup(
                new RevShooter(robot.getShooter(), ShootParams.Auto2.getLeftSpeed(), ShootParams.Auto2.getRightSpeed()),
                new WaitCommand(shooterDelay)));
        NamedCommands.registerCommand("startAuto3", new ParallelRaceGroup(
                new RevShooter(robot.getShooter(), ShootParams.Auto3.getLeftSpeed(), ShootParams.Auto3.getRightSpeed()),
                new WaitCommand(shooterDelay)));

        NamedCommands.registerCommand("getPiece",
                new IntakeSequence(robot.getIntake(), robot.getDeck(), robot.getElevator()));

        NamedCommands.registerCommand("shootAuto2",
                new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(), ShootParams.Auto2, false, robot.getLimelight()));
        NamedCommands.registerCommand("shootAuto1",
                new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(), ShootParams.Auto1, false, robot.getLimelight()));
        NamedCommands.registerCommand("shootAuto3",
                new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(), ShootParams.Auto3, false, robot.getLimelight()));
        NamedCommands.registerCommand("finishAuto", new InstantCommand(() -> robot.getShooter().stop()));

        NamedCommands.registerCommand("shootAutoAim",
                new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(), ShootParams.Podium, true, robot.getLimelight()));

    }

    public Command SelectAuto() {
        switch (robot.getThumbwheel().getValue() % 8) {

            case 1:
                return robot.getDrivebase().getAutoCommand("Auto1");
            case 2:
                return robot.getDrivebase().getAutoCommand("Auto2");
            case 3:
                return robot.getDrivebase().getAutoCommand("Auto3");

            case 4: // 3 piece
                return robot.getDrivebase().getAutoCommand("Auto4");

            case 5: // 4 piece
                return robot.getDrivebase().getAutoCommand("Auto5");

            case 6: // 3 piece
                return robot.getDrivebase().getAutoCommand("Auto6");

            default:
                return new InstantCommand();

        }
    }

}
