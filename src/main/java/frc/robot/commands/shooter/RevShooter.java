// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.ShooterConfig;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;

public class RevShooter extends Command {

  private ShooterSubsystem SHOOTER_SUBSYSTEM;

  /** Creates a new RevShooter. */
  public RevShooter(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.SHOOTER_SUBSYSTEM = shooter;

    addRequirements(SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting RevShooter Command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SHOOTER_SUBSYSTEM.set(ShooterConfig.leftSpeed, ShooterConfig.rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SHOOTER_SUBSYSTEM.stop();
    System.out.println("Ending RevShooterCommand");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
