// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.Leds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ToggleForward extends Command {
  /** Creates a new ToggleForward. */
  public ToggleForward() {
    // Use addRequirements() here to declare subsystem dependencies.
    // * This is for Shuffleboard
    GenericEntry forwardingOn;
    shuffleboardInit();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Leds.setForwardingStatus(false);
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
    return false;
  }

  public void shuffleboardInit() {
    // * Put what you want on Shuffleboard here.
  }

  public void updateShuffleboard() {
    // * Update Shuffleboard
  }
}
