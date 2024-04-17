// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class NoteAutoAimRobot extends Command {
  /** Creates a new NoteAutoAim. */
  SwerveSubsystem DRIVEBASE;
  CommandJoystick DRIVE_JOYSTICK;
  CommandJoystick ROTATION_JOYSTICK;
  Translation2d translation;
  IntakeSubsystem INTAKE_SUBSYSTEM;
  boolean robotOriented;
  public NoteAutoAimRobot(SwerveSubsystem drivebase, CommandJoystick drivejoystick, CommandJoystick rotationjoystick, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    DRIVEBASE = drivebase;
    INTAKE_SUBSYSTEM = intake;
    DRIVE_JOYSTICK = drivejoystick;
    ROTATION_JOYSTICK = rotationjoystick;
    addRequirements(DRIVEBASE);
    robotOriented = DRIVEBASE.getRobotOriented();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    var alliance = DriverStation.getAlliance();
    if(alliance.get() == Alliance.Red){
      translation = new Translation2d(MathUtil.applyDeadband(DRIVE_JOYSTICK.getY(), OperatorConstants.LEFT_Y_DEADBAND)*3,MathUtil.applyDeadband(DRIVE_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3);
      DRIVEBASE.drive(translation, -MathUtil.applyDeadband(ROTATION_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3,false);
    }
    else{
      translation = new Translation2d(-MathUtil.applyDeadband(DRIVE_JOYSTICK.getY(), OperatorConstants.LEFT_Y_DEADBAND)*3,-MathUtil.applyDeadband(DRIVE_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3);
      DRIVEBASE.drive(translation, -MathUtil.applyDeadband(ROTATION_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3,false);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    var alliance = DriverStation.getAlliance();
    robotOriented = DRIVEBASE.getRobotOriented();
    

    if(LimelightHelpers.getTV("limelight-intake") && alliance.get() == Alliance.Blue)

    {
      translation = new Translation2d(-MathUtil.applyDeadband(DRIVE_JOYSTICK.getY(), OperatorConstants.LEFT_Y_DEADBAND)*3,-MathUtil.applyDeadband(DRIVE_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3);
      DRIVEBASE.drive(translation, -limelight_aim_proportional_note(),!robotOriented);
    }

    else if (!LimelightHelpers.getTV("limelight-intake")&& alliance.get() == Alliance.Blue)

    {
      translation = new Translation2d(-MathUtil.applyDeadband(DRIVE_JOYSTICK.getY(), OperatorConstants.LEFT_Y_DEADBAND)*3,-MathUtil.applyDeadband(DRIVE_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3);
      DRIVEBASE.drive(translation, -MathUtil.applyDeadband(ROTATION_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3,true);
    }

    else if(LimelightHelpers.getTV("limelight-intake")&& alliance.get() == Alliance.Red && !robotOriented)
    {
      translation = new Translation2d(MathUtil.applyDeadband(DRIVE_JOYSTICK.getY(), OperatorConstants.LEFT_Y_DEADBAND)*3,MathUtil.applyDeadband(DRIVE_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3);
      DRIVEBASE.drive(translation, -limelight_aim_proportional_note(),!robotOriented);
    }
    else if (!LimelightHelpers.getTV("limelight-intake")&& alliance.get() == Alliance.Red)
    {

      translation = new Translation2d(MathUtil.applyDeadband(DRIVE_JOYSTICK.getY(), OperatorConstants.LEFT_Y_DEADBAND)*3,MathUtil.applyDeadband(DRIVE_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3);
      DRIVEBASE.drive(translation, -MathUtil.applyDeadband(ROTATION_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3,true);
    }
    else if (LimelightHelpers.getTV("limelight-intake")&& alliance.get() == Alliance.Red && robotOriented)
    {
      translation = new Translation2d(MathUtil.applyDeadband(-DRIVE_JOYSTICK.getY(), OperatorConstants.LEFT_Y_DEADBAND)*3,MathUtil.applyDeadband(-DRIVE_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3);
      DRIVEBASE.drive(translation, -limelight_aim_proportional_note(),!robotOriented);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    translation = new Translation2d(MathUtil.applyDeadband(DRIVE_JOYSTICK.getY(), OperatorConstants.LEFT_Y_DEADBAND)*3,MathUtil.applyDeadband(DRIVE_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3);
      DRIVEBASE.drive(translation, -MathUtil.applyDeadband(ROTATION_JOYSTICK.getX(), OperatorConstants.LEFT_X_DEADBAND)*3,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
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
