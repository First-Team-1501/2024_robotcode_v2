// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShootParams;
import frc.robot.commands.deck.AutoDeckAim;
import frc.robot.commands.intake.ShootNote;
import frc.robot.commands.intake.SimpleShootNote;
import frc.robot.subsystems.deck.DeckSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  public AutoShoot(ShooterSubsystem shooter, DeckSubsystem deckSubsystem, IntakeSubsystem intakeSubsystem, ShootParams param, boolean autoAim, Limelight limelight ) {
   
    if(autoAim)
    {
       addCommands(
        new ParallelRaceGroup(
          new AutoDeckAim(deckSubsystem, limelight),
          new ShootNote(intakeSubsystem, limelight))
          );
    }
    else{
      addCommands(new SimpleShootNote(intakeSubsystem));
    }

    //addCommands(new SetDeckPosition(deckSubsystem, DeckPositions.home));
  
  }
}
