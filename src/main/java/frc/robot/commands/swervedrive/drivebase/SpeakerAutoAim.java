// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;



public class SpeakerAutoAim extends Command {
  /** Creates a new SpeakerAutoAim. */
  SwerveSubsystem DRIVEBASE;
  CommandJoystick DRIVE_JOYSTICK;
  CommandJoystick ROTATION_JOYSTICK;
  Translation2d translation;
  public SpeakerAutoAim(SwerveSubsystem drivebase, CommandJoystick drivejoystick, CommandJoystick rotationjoystick) {
    // Use addRequirements() here to declare subsystem dependencies.

    DRIVEBASE = drivebase;
    DRIVE_JOYSTICK = drivejoystick;
    ROTATION_JOYSTICK = rotationjoystick;
    addRequirements(DRIVEBASE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(LimelightHelpers.getTV("limelight"))
    {
      translation = new Translation2d(-MathUtil.applyDeadband(DRIVE_JOYSTICK.getY(), OperatorConstants.LEFT_Y_DEADBAND)*3,-MathUtil.applyDeadband(DRIVE_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3);
      DRIVEBASE.drive(translation, -limelight_aim_proportional(),false);
    }
    else
    {
      translation = new Translation2d(-MathUtil.applyDeadband(DRIVE_JOYSTICK.getY(), OperatorConstants.LEFT_Y_DEADBAND)*3,-MathUtil.applyDeadband(DRIVE_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3);
      DRIVEBASE.drive(translation, -MathUtil.applyDeadband(ROTATION_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3,true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double limelight_aim_proportional() {

    double kP = 0.04;
    double kI = 0;
    double kD = 0.00000000000001;
    double maxTolerance = 3;

    try (PIDController pidCont = new PIDController(kP, kI, kD)) {
      double targetingAngularVelocity = pidCont.calculate(-LimelightHelpers.getTX("limelight"));

      if (targetingAngularVelocity > maxTolerance) {
        targetingAngularVelocity = maxTolerance;
      } else if (targetingAngularVelocity < -maxTolerance) {
        targetingAngularVelocity = -maxTolerance;
      }

      SmartDashboard.putNumber("Aim PID Calc", targetingAngularVelocity);

      return targetingAngularVelocity;
    }
  }
}
