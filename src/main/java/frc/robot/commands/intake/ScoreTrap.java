// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ScoreTrap extends Command {

  private IntakeSubsystem INTAKE_SUBSYSTEM;
  private boolean hitSensor;
  private int counter;

  /** Creates a new RunOuttakeCommand. */
  public ScoreTrap(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.INTAKE_SUBSYSTEM = intake;
    

    addRequirements(INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("Starting RunOuttakeCommand");
    INTAKE_SUBSYSTEM.set(-0.3, -0.8);
    hitSensor = false;
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(!hitSensor){
      hitSensor = INTAKE_SUBSYSTEM.readyToScoreTrap();
    }
    else if(hitSensor){
      counter++;
    }
    
    if (counter > 28){
      INTAKE_SUBSYSTEM.set(1.0, -1.0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    INTAKE_SUBSYSTEM.stop();
    //System.out.println("Ending RunOuttakeCommand");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter > 200;
  }
}
