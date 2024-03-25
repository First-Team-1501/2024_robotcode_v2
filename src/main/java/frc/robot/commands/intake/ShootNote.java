// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightHelpers;

public class ShootNote extends Command {

  private IntakeSubsystem INTAKE_SUBSYSTEM;
  private int counter;

  /** Creates a new ShootNote. */
  public ShootNote(IntakeSubsystem intake, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.INTAKE_SUBSYSTEM = intake;

    addRequirements(INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    // System.out.println("Starting ShootNote Command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if ((LimelightHelpers.getTX("limelight") < 3) &&
        (LimelightHelpers.getTY("limelight") < 3) &&
        (LimelightHelpers.getTX("limelight") > -3) &&
        (LimelightHelpers.getTY("limelight") > -3) &&
        (LimelightHelpers.getTV("limelight")) &&
        (LimelightHelpers.getTA("limelight") > 0.2) ) {
      if(counter>1)INTAKE_SUBSYSTEM.set(1, 1);
      counter++;
    }
    else if(counter > 1) counter++;
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
    return counter > 25;
  }

  public boolean targetLocked() {
    return (LimelightHelpers.getTX("limelight") < 2) &&
        (LimelightHelpers.getTY("limelight") < 2) &&
        (LimelightHelpers.getTX("limelight") > 2) &&
        (LimelightHelpers.getTY("limelight") > 2);
  }
}
