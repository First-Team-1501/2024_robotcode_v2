// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.deck;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.deck.DeckSubsystem;

public class SetDeckMaxOutput extends Command {
  /** Creates a new SetDeckMaxOutput. */
  DeckSubsystem DECK_SUBSYSTEM;
  double OUTPUT;
  boolean done;
  public SetDeckMaxOutput(DeckSubsystem deck, double output) {
    // Use addRequirements() here to declare subsystem dependencies.
    OUTPUT = output;
    DECK_SUBSYSTEM = deck;
    done = false;

    addRequirements(DECK_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    DECK_SUBSYSTEM.setMaxOutput(OUTPUT);
    done = true;
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
