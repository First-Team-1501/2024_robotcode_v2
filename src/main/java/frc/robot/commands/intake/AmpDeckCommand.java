// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class AmpDeckCommand extends Command {
  
  private IntakeSubsystem INTAKE_SUBSYSTEM;

  private boolean hasSeenPiece;
  private boolean stageTwo;
  private boolean stageThree;
  private int counter;
  
  /** Creates a new AmpDeckCommand. */
  public AmpDeckCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.INTAKE_SUBSYSTEM = intake;

    addRequirements(INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("Starting AmpDeckCommand");
    INTAKE_SUBSYSTEM.set(-.5, -.5);
    counter = 0;
    hasSeenPiece = false;
    stageTwo = false;
    stageThree = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(INTAKE_SUBSYSTEM.readyToScoreTrap())
    {
      hasSeenPiece = true;
    }
    if(hasSeenPiece && !INTAKE_SUBSYSTEM.readyToScoreTrap())
    {
      stageTwo = true;
      INTAKE_SUBSYSTEM.set(-.5, 0);
    }
    if(stageTwo && INTAKE_SUBSYSTEM.readyToScoreTrap())
    {
      stageThree = true;
      
    }

    if(stageThree)
    {
      counter++;
    } 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    INTAKE_SUBSYSTEM.stop();
    System.out.println("Ending AmpDeckommand");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (counter > 5);
  }
}
