// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.intake.AmpDeckCommand;
import frc.robot.commands.swervedrive.drivebase.AmpAutoAim;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.deck.DeckSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAmpSequence extends SequentialCommandGroup {
  /** Creates a new AutoAmpSequence. */
  Command ampPipeline;

  public AutoAmpSequence(Limelight limelight, DeckSubsystem deck, IntakeSubsystem intake, SwerveSubsystem drivebase, CommandJoystick driveStick, CommandJoystick rotStick, ElevatorSubsystem elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /*var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      LimelightHelpers.setPipelineIndex("limelight-intake", 1);
    } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      LimelightHelpers.setPipelineIndex("limelight-intake", 2);
    }*/

    addCommands(
        new SetDeckPosition(deck, DeckPositions.amp)
            .alongWith(new AmpDeckCommand(intake))
            .alongWith(new AmpAutoAim(drivebase, driveStick, rotStick))
            .alongWith(new SetElevatorPosition(elevator, 4))
    );
  }
}
