// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class NormalDrive extends Command {
  /** Creates a new NormalDrive. */
  SwerveSubsystem DRIVEBASE;
  CommandJoystick DRIVE_STICK;
  CommandJoystick ROT_STICK;
  Translation2d translation;
  public NormalDrive(SwerveSubsystem drivebase, CommandJoystick drivejoystick, CommandJoystick rotationjoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    DRIVEBASE = drivebase;
    DRIVE_STICK = drivejoystick;
    ROT_STICK = rotationjoystick;

    addRequirements(DRIVEBASE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    translation = new Translation2d(MathUtil.applyDeadband(DRIVE_STICK.getY(), OperatorConstants.LEFT_Y_DEADBAND)*3,MathUtil.applyDeadband(DRIVE_STICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3);
    DRIVEBASE.drive(translation, -MathUtil.applyDeadband(DRIVE_STICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3,true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    translation = new Translation2d(MathUtil.applyDeadband(DRIVE_STICK.getY(), OperatorConstants.LEFT_Y_DEADBAND)*3,MathUtil.applyDeadband(DRIVE_STICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3);
    DRIVEBASE.drive(translation, -MathUtil.applyDeadband(DRIVE_STICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
