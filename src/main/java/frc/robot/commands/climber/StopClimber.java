// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class StopClimber extends Command {
  /** Creates a new StopClimber. */
  ClimberSubsystem CLIMBER_SUBSYSTEM;
  public StopClimber(ClimberSubsystem climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    CLIMBER_SUBSYSTEM = climber;
    addRequirements(CLIMBER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {CLIMBER_SUBSYSTEM.set(CLIMBER_SUBSYSTEM.get());}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
