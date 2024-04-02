// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.limelight.Limelight;

public class ChangePipeline extends Command {
  /** Creates a new ChangePipeline. */
  Limelight LIMELIGHT;
  String PIPELINE_NAME;
  int PIPELINE;
  public ChangePipeline(Limelight limelight, String pipelineName, int pipeline) {
    // Use addRequirements() here to declare subsystem dependencies.
    LIMELIGHT = limelight;
    PIPELINE_NAME = pipelineName;
    PIPELINE = pipeline;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LIMELIGHT.changePipeline(PIPELINE_NAME, PIPELINE);
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
