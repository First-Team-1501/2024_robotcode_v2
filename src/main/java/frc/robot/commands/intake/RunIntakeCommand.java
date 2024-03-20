// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConfig;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.Leds;

public class RunIntakeCommand extends Command {

  private IntakeSubsystem INTAKE_SUBSYSTEM;

  /** Creates a new RunIntakeCommand. */
  public RunIntakeCommand(IntakeSubsystem intake, Leds leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.INTAKE_SUBSYSTEM = intake;

    addRequirements(INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Leds.setIntakeStatus(true);
    //System.out.println("Starting RunIntakeCommand");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(!isFinished())
    {
      INTAKE_SUBSYSTEM.set( IntakeConfig.runningSpeed, IntakeConfig.runningSpeed);   
    }
    else
    {
      INTAKE_SUBSYSTEM.stop();
    }
    
    if(INTAKE_SUBSYSTEM.readyToScoreTrap())
    {
     Leds.setIntakeStatus(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    INTAKE_SUBSYSTEM.stop();
    Leds.setIntakeStatus(false);
    //System.out.println("Ending RunIntakeCommand");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return INTAKE_SUBSYSTEM.readyToScoreTrap();
  }
}
