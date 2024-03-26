// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.intake.StopIntake;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.deck.DeckSubsystem;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractIntakeSequence extends SequentialCommandGroup {
  /** Creates a new RetractIntakeSequence. */
  public RetractIntakeSequence(DeckSubsystem deckSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetDeckPosition(deckSubsystem, DeckPositions.home)
        .alongWith(new SetElevatorPosition(elevatorSubsystem, ElevatorPositions.zero)
        .alongWith(new StopIntake(intake)))
    );
  }
}
