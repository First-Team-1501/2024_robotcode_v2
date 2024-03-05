// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;


public class Limelight extends SubsystemBase {
  LimelightHelpers limelight = new LimelightHelpers();
  /** Creates a new Limelight. */
  public Limelight() {
    
  }

  @Override
  public void periodic() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        // Put what you want it to do here.
        LimelightHelpers.setPipelineIndex("limelight", 1); 
        
      }
      if (ally.get() == Alliance.Blue) {
        // Put what you want it to do here.
        LimelightHelpers.setPipelineIndex("limelight", 0); 
    }
    // This method will be called once per scheduler run
  }
  }
}
