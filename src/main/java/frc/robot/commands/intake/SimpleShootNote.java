// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConfig;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class SimpleShootNote extends Command {
  
  private IntakeSubsystem INTAKE_SUBSYSTEM;
  private int counter;

  /** Creates a new SimpleShootNote. */
  public SimpleShootNote(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.INTAKE_SUBSYSTEM = intake;

    addRequirements(INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {counter = 0;}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    INTAKE_SUBSYSTEM.set( IntakeConfig.runningSpeed, IntakeConfig.runningSpeed);
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter > 20;
  }
}
