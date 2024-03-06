// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.Limelight;

public class ShootNote extends Command {

  private IntakeSubsystem INTAKE_SUBSYSTEM;
  private Limelight LIMELIGHT;
  private int counter;

  /** Creates a new ShootNote. */
  public ShootNote(IntakeSubsystem intake, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.INTAKE_SUBSYSTEM = intake;
    LIMELIGHT = limelight;
    counter = 0;

    addRequirements(INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Starting ShootNote Command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LIMELIGHT.tX() < 0.5 && LIMELIGHT.tY() < 0.5)
      INTAKE_SUBSYSTEM.set(1, 1);
      counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    INTAKE_SUBSYSTEM.stop();
    // System.out.println("Ending ShootNote Command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter > 20;
  }
}
