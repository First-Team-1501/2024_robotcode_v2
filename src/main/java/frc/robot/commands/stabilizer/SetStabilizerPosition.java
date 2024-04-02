// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stabilizer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.stabilizer.StabilizerPositions;
import frc.robot.subsystems.stabilizer.StabilizerSubsystem;

public class SetStabilizerPosition extends Command {
  
  private StabilizerSubsystem STABILIZER_SUBSYSTEM;
  private double stabilizerPosition;
  
  /** Creates a new SetStabilizerPosition. */
  public SetStabilizerPosition(StabilizerSubsystem stabilizer, double position) {
    
    this.STABILIZER_SUBSYSTEM = stabilizer;
    stabilizerPosition = position;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(STABILIZER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("Starting SetStabilizerPosition Command - Target Position: " + stabilizerPosition);
    STABILIZER_SUBSYSTEM.set(stabilizerPosition);
    if (stabilizerPosition > 5){
      Leds.setClimbingStatus(true);
    } else {
      Leds.setClimbingStatus(false);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    System.out.println("Ending SetStabilizerPosition Command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(STABILIZER_SUBSYSTEM.get() - stabilizerPosition) < StabilizerPositions.tolerance;
  }
}
