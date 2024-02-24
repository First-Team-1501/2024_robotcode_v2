// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.deck.DeckSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.stabilizer.StabilizerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetRobot extends InstantCommand {
  BooleanSupplier isRed;
  RobotContainer robot;

  public ResetRobot(RobotContainer robot, BooleanSupplier isRed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.isRed = isRed;
    this.robot = robot;
    addRequirements(
      robot.getClimber(), 
      robot.getDeck(),
      robot.getDrivebase(), 
      robot.getElevator(), 
      //robot.getIntake(), 
      //robot.getShooter(), 
      robot.getStabilizer()
     );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    
  }

  @Override
  public void execute()
  {
    System.err.println("Reset!!!!!!!!!!!!!!!!!!!!!");
     robot.getDrivebase().resetOdometry(new Pose2d());
     robot.getStabilizer().resetEncoder();
     //robot.getShooter() // shooter doesn't do position control
     //robot.getIntake(). // also doesn't
     robot.getElevator().resetEncoder();
     robot.getDeck().resetEncoder();
     robot.getClimber().resetEncoder();
    LimelightHelpers.setPipelineIndex("limelight", isRed.getAsBoolean()?1:0);
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}