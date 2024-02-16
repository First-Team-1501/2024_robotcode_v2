// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class JogClimberDown extends Command {
  /** Creates a new JogClimberDown. */
  ClimberSubsystem CLIMBER_SUBSYSTEM;
  public JogClimberDown(ClimberSubsystem CLIMBER_SUBSYSTEM) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.CLIMBER_SUBSYSTEM = CLIMBER_SUBSYSTEM;
    addRequirements(CLIMBER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    CLIMBER_SUBSYSTEM.set(CLIMBER_SUBSYSTEM.get()-0.5);
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
