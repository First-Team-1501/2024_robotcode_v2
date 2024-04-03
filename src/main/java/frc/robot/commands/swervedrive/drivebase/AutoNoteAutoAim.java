// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoNoteAutoAim extends Command {
  /** Creates a new AutoNoteAutoAim. */
  SwerveSubsystem DRIVEBASE;
  IntakeSubsystem INTAKE;
  Translation2d translation;
  boolean stopDriving;
  public AutoNoteAutoAim(SwerveSubsystem drivebase, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    DRIVEBASE = drivebase;
    INTAKE = intake;
    addRequirements(DRIVEBASE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopDriving = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (INTAKE.readyToScoreTrap()) stopDriving = true;
    if(LimelightHelpers.getTV("limelight-intake") && !stopDriving)
    {
      translation = new Translation2d(drive_speed(),0);
      DRIVEBASE.drive(translation, -limelight_aim_proportional_note(),false);
    }
    else{
      translation = new Translation2d(0,0);
      DRIVEBASE.drive(translation, 0,false);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return INTAKE.readyToScoreTrap();
  }

  public double drive_speed()
  {
    double tY = LimelightHelpers.getTY("limelight-intake");
    double outputSpeed = (26 + tY) / 12;
    return outputSpeed;
  }

  public double limelight_aim_proportional_note() {

    double kP = 0.05;
    double kI = 0;
    double kD = 0.00000000000000000001;
    double maxTolerance = 3;


    try (PIDController pidCont = new PIDController(kP, kI, kD)) {
      double targetingAngularVelocity = pidCont.calculate(-LimelightHelpers.getTX("limelight-intake"));


      if (targetingAngularVelocity > maxTolerance) {
        targetingAngularVelocity = maxTolerance;
      } else if (targetingAngularVelocity < -maxTolerance) {
        targetingAngularVelocity = -maxTolerance;
      }

      SmartDashboard.putNumber("NOTE PID CALC", targetingAngularVelocity);

      return targetingAngularVelocity;
    }
  }
}
