// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;

public class SetElevatorPosition extends Command {

  private ElevatorSubsystem ELEVATOR_SUBSYSTEM;
  private double elevatorPosition;

  /** Creates a new SetElevatorPosition. */
  public SetElevatorPosition(ElevatorSubsystem elevator, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ELEVATOR_SUBSYSTEM = elevator;
    elevatorPosition = position;

    addRequirements(ELEVATOR_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("Starting SetElevatorPosition Command - Target Position = " + elevatorPosition);
    ELEVATOR_SUBSYSTEM.set(elevatorPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    System.out.println("Ending SetElevatorPosition Command - Elevator at Position = " + elevatorPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(ELEVATOR_SUBSYSTEM.get() - elevatorPosition) < 0.1;
  }
}
