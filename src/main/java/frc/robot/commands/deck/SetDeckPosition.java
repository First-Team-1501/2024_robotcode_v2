// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.deck;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.deck.DeckSubsystem;

public class SetDeckPosition extends Command {

  private DeckSubsystem DECK_SUBSYSTEM;
  private double deckPosition;

  /** Creates a new SetDeckPosition. */
  public SetDeckPosition(DeckSubsystem deck, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DECK_SUBSYSTEM = deck;
    deckPosition = position;

    addRequirements(DECK_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("Starting SetDeckPosition Command - Target Position = " + deckPosition);
    DECK_SUBSYSTEM.set(deckPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("Ending SetDeckPosition Command - Deck at Position = " + deckPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(DECK_SUBSYSTEM.get() - deckPosition) < DeckPositions.tolerance;
  }
}
