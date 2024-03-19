// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class SetElevatorMaxOutput extends Command {
  /** Creates a new SetElevatorMaxOutput. */
  ElevatorSubsystem ELEVATOR_SUBSYSTEM;
  double OUTPUT;
  boolean done;
  public SetElevatorMaxOutput(ElevatorSubsystem elevator, double output) {
    // Use addRequirements() here to declare subsystem dependencies.
    ELEVATOR_SUBSYSTEM = elevator;
    OUTPUT = output;
    done = false;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    ELEVATOR_SUBSYSTEM.setMaxOutput(OUTPUT);
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
