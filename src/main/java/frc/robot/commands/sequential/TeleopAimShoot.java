// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.deck.AutoDeckAim;
import frc.robot.commands.intake.ShootNote;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.commands.swervedrive.drivebase.SpeakerAutoAim;
import frc.robot.subsystems.deck.DeckSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterConfig;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopAimShoot extends SequentialCommandGroup {
  /** Creates a new TeleopAimShoot. */
  public TeleopAimShoot(ShooterSubsystem shooter, DeckSubsystem deck, Limelight limelight, SwerveSubsystem drivebase,
      CommandJoystick driveJoystick, CommandJoystick rotateJoystick, IntakeSubsystem intake) {
    
    addCommands(
        new ParallelRaceGroup(
            new RevShooter(shooter, ShooterConfig.podiumLeftSpeed, ShooterConfig.podiumRightSpeed),
            new SpeakerAutoAim(drivebase, driveJoystick, rotateJoystick),
            new AutoDeckAim(deck, limelight),
            new ShootNote(intake, limelight)
        )    
    );
          
  }
}
