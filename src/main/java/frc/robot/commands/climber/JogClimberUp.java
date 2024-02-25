// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class JogClimberUp extends Command {
  
  private ClimberSubsystem CLIMBER_SUBSYSTEM;
  
  /** Creates a new JogClimberUp. */
  public JogClimberUp(ClimberSubsystem climber) {
    this.CLIMBER_SUBSYSTEM = climber;

    addRequirements(CLIMBER_SUBSYSTEM);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("Starting JogClimberUp Command");
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    CLIMBER_SUBSYSTEM.set(CLIMBER_SUBSYSTEM.get() + 50);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    System.out.println("Ending JogClimberUp Command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
