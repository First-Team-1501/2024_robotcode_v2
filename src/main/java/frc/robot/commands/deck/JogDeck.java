// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.deck;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.deck.DeckSubsystem;

public class JogDeck extends Command {
  /** Creates a new JogDeck. */

  private double increment;
  private DeckSubsystem DECK;

  public JogDeck(DeckSubsystem deck, double increment) {
    // Use addRequirements() here to declare subsystem dependencies.
    DECK = deck;
    this.increment = increment;
    addRequirements(DECK);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DECK.set(DECK.get()+increment);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
