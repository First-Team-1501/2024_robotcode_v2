// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class SetClimberPosition extends Command {

  private ClimberSubsystem CLIMBER_SUBSYSTEM;
  private double climberPosition;

  /** Creates a new SetClimberPosition. */
  public SetClimberPosition(ClimberSubsystem climber, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.CLIMBER_SUBSYSTEM = climber;
    climberPosition = position;

    addRequirements(CLIMBER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting SetClimberPosition Command - Target Positon = " + climberPosition);
    CLIMBER_SUBSYSTEM.set(climberPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending SetClimberPosition Command - Climber at Position = " + climberPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(CLIMBER_SUBSYSTEM.get() - climberPosition) < 0.1;
  }
}
