// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.deck;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.deck.DeckSubsystem;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.limelight.LimelightHelpers.LimelightTarget_Fiducial;

public class AutoDeckAim extends Command {
  private DeckSubsystem DECK_SUBSYSTEM;

  LimelightHelpers limelight = new LimelightHelpers(); // Create a new instance of the LimelightHelpers class
  LimelightTarget_Fiducial m_Fiducial = new LimelightTarget_Fiducial(); // Create a new instance of the LimelightTarget_Fiducial class 

  double kP = 0.8;
  double kI = 0;
  double kD = 0;

  PIDController deckPIDController = new PIDController(kP, kI, kD);

  double pidOut;
 
  

  /** Creates a new AutoDeckAim. */
  public AutoDeckAim(DeckSubsystem deck) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DECK_SUBSYSTEM = deck;
    

    addRequirements(DECK_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    pidOut = deckPIDController.calculate(limelight_aim_proportional());
    SmartDashboard.putNumber("Deck PID Cont", pidOut);
    DECK_SUBSYSTEM.set(DECK_SUBSYSTEM.get() + pidOut);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }

  double limelight_aim_proportional()
  {
    double targetingPosition = LimelightHelpers.getTY("limelight");

    return targetingPosition;
  }
}
