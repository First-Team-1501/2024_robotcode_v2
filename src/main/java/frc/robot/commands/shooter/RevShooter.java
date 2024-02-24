// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RevShooter extends Command {

  private ShooterSubsystem SHOOTER_SUBSYSTEM;
  private double left;
  private double right;
  private boolean stop;

  public RevShooter(ShooterSubsystem shooter, double leftSpeed, double rightSpeed)
  {
    this(shooter, leftSpeed, rightSpeed, true);
  }


  /** Creates a new RevShooter. */
  public RevShooter(ShooterSubsystem shooter, double leftSpeed, double rightSpeed, boolean stop) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.SHOOTER_SUBSYSTEM = shooter;
    left = leftSpeed;
    right = rightSpeed;
    this.stop = stop;
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
    SHOOTER_SUBSYSTEM.set(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(stop)
    {
      SHOOTER_SUBSYSTEM.stop();
    }
    System.out.println("Ending RevShooterCommand");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
