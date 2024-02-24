// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShootParams;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.intake.ShootNote;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.deck.DeckSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  public AutoShoot(ShooterSubsystem shooter, DeckSubsystem deckSubsystem, IntakeSubsystem intakeSubsystem, ShootParams param ) {
    // Add your commands in the addCommands() call, e.g.
    //addRequirements(null);

    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new SetDeckPosition(deckSubsystem, param.getDeckPosition()),
    new ParallelRaceGroup(
      new RevShooter(shooter, param.getLeftSpeed(), param.getRightSpeed()),
      new WaitCommand(1).andThen(new ShootNote(intakeSubsystem)),
      new InstantCommand(()->{System.out.println("started shoot!!!!");}
        ).andThen(new WaitCommand(3))),
      new SetDeckPosition(deckSubsystem, DeckPositions.home));
  }
}
