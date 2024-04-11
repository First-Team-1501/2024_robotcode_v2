// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.Leds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.shooter.ShooterConfig;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ToggleForward extends Command {
  /** Creates a new ToggleForward. */
  ShooterSubsystem SHOOTER_SUBSYSTEM;
  boolean forwardingStatus;
  public ToggleForward(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    // * This is for Shuffleboard
    GenericEntry forwardingOn;
    shuffleboardInit();
    SHOOTER_SUBSYSTEM = shooter;
    forwardingStatus = false;
    addRequirements(SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Leds.setForwardingStatus(false);
    forwardingStatus = !forwardingStatus;
    Shuffleboard.getTab("Info").add("Forwarding", forwardingStatus);
   if(forwardingStatus)
   {
    // When True 
    SHOOTER_SUBSYSTEM.set(ShooterConfig.forwardingLeftSpeed, ShooterConfig.forwardingRightSpeed);
   }
   else
   {
    // When False
     SHOOTER_SUBSYSTEM.stop();
   }
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

  public void shuffleboardInit() {
    // * Put what you want on Shuffleboard here.
  }

  public void updateShuffleboard() {
    // * Update Shuffleboard
  }
}
