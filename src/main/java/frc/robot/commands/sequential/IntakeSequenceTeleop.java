// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.elevator.SetElevatorAmpLimit;
import frc.robot.commands.elevator.SetElevatorMaxOutput;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.deck.DeckSubsystem;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.Leds;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSequenceTeleop extends SequentialCommandGroup {
  /** Creates a new IntakeSequenceTeleop. */
  public IntakeSequenceTeleop(IntakeSubsystem intake, DeckSubsystem deck, ElevatorSubsystem elevator, Leds leds) {
    addCommands(
      new SetElevatorPosition(elevator, ElevatorPositions.intake)
      .alongWith
      (new SetDeckPosition(deck, DeckPositions.intake))
      .alongWith
      (new RunIntakeCommand(intake, leds).raceWith(new WaitCommand(3))),
      new SetElevatorAmpLimit(elevator, 3, 3)
      .alongWith(new SetElevatorMaxOutput(elevator, 0.2)),

      new SetElevatorAmpLimit(elevator, 30, 40)
      .andThen(new SetElevatorMaxOutput(elevator, 1.0)),

      new SetDeckPosition(deck, DeckPositions.home)
      .alongWith(new SetElevatorPosition(elevator, ElevatorPositions.zero))
    );
  }
}
