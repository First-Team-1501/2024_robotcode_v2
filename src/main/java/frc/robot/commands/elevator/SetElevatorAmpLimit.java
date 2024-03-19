// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class SetElevatorAmpLimit extends Command {
  /** Creates a new SetElevatorAmpLimit. */
  ElevatorSubsystem ELEVATOR_SUBSYSTEM;
  int STALL_CURRENT;
  int FREE_CURRENT;
  public SetElevatorAmpLimit(ElevatorSubsystem elevator, int stallCurrent, int freeCurrent) {
    ELEVATOR_SUBSYSTEM = elevator;
    STALL_CURRENT = stallCurrent;
    FREE_CURRENT = freeCurrent;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    ELEVATOR_SUBSYSTEM.changeAmpLimits(STALL_CURRENT, FREE_CURRENT);
  }

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
