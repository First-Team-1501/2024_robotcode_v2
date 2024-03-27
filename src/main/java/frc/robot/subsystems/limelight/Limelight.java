// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class Limelight extends SubsystemBase {

  Optional<Alliance> alliance;
  // * This is for the Limelight going to Shuffleboard.
  GenericEntry limelightTX;
  GenericEntry limelightTY;
  GenericEntry limelightTA;
  GenericEntry limelightPipelineIndex;
  GenericEntry limelightTV;
  GenericEntry limelightCL;

  // ! This is for the Limelight-Intake going to Shuffleboard.
  GenericEntry limelightIntakeTX;
  GenericEntry limelightIntakeTY;
  GenericEntry limelightIntakeTA;
  GenericEntry limelightIntakePipelineIndex;
  GenericEntry limelightIntakeTV;
  GenericEntry limelightIntakeCL;

  /** Creates a new Limelight. */
  public Limelight() {

    setupLimelight();
    /*updateShuffleboardLimelight();
    updateShuffleboardLimelightIntake();*/

  }

  @Override
  public void periodic() {
    setPipelineUsingAllianceColor();
    updateShuffleboardLimelight();
    updateShuffleboardLimelightIntake();
  }

  private void setupLimelight() {

    setPipelineUsingAllianceColor();

    LimelightHelpers.getLimelightNTTable(null);
    LimelightHelpers.getLimelightNTTableEntry(null, "tid");


    LimelightHelpers.setLEDMode_PipelineControl("limelight");

    shuffleboardInit();
    shuffleboardintakeInit();
    

  }

  public void shuffleboardintakeInit() {
    // This adds the Limelight-Intake tx value to Shuffleboard
    limelightIntakeTX = Shuffleboard.getTab("Limelight")
      .add("Limelight Intake TX", LimelightHelpers.getTX("limelight-intake"))
      .getEntry();

    // This adds the Limelight-Intake ty value to Shuffleboard
    limelightIntakeTY = Shuffleboard.getTab("Limelight")
      .add("Limelight Intake TY", LimelightHelpers.getTY("limelight-intake"))
      .getEntry();

    // This adds the Limelight-Intake ta value to Shuffleboard
    limelightIntakeTA = Shuffleboard.getTab("Limelight")
      .add("Limelight Intake TA", LimelightHelpers.getTA("limelight-intake"))
      .getEntry();

    // This adds the Limelight-Intake pipeline index value to Shuffleboard
    limelightIntakePipelineIndex = Shuffleboard.getTab("Limelight")
      .add("Limelight Intake Pipeline Index", LimelightHelpers.getCurrentPipelineIndex("limelight-intake"))
      .getEntry();

    // This adds the Limelight-Intake tv value to Shuffleboard
    limelightIntakeTV = Shuffleboard.getTab("Limelight")
      .add("Limelight Intake TV", LimelightHelpers.getTV("limelight-intake"))
      .withWidget("Boolean Box")
      .getEntry();

    // This adds the Limelight-Intake cl value to Shuffleboard
    limelightIntakeCL = Shuffleboard.getTab("Limelight")
      .add("Limelight Intake CL", LimelightHelpers.getLimelightNTDouble("limelight-intake", "cl"))
      .getEntry();
  }

  public void shuffleboardInit() {
    // This adds the Limelight tx value to Shuffleboard
    limelightTX = Shuffleboard.getTab("Limelight")
      .add("Limelight TX", LimelightHelpers.getTX("limelight"))
      .getEntry();

    // This adds the Limelight ty value to Shuffleboard
    limelightTY = Shuffleboard.getTab("Limelight")
      .add("Limelight TY", LimelightHelpers.getTY("limelight"))
      .getEntry();

    // This adds the Limelight ta value to Shuffleboard
    limelightTA = Shuffleboard.getTab("Limelight")
      .add("Limelight TA", LimelightHelpers.getTA("limelight"))
      .getEntry();

    // This adds the Limelight pipeline index value to Shuffleboard
    limelightPipelineIndex = Shuffleboard.getTab("Limelight")
      .add("Limelight Pipeline Index", LimelightHelpers.getCurrentPipelineIndex("limelight"))
      .getEntry();

    // This adds the Limelight tv value to Shuffleboard
    limelightTV = Shuffleboard.getTab("Limelight")
      .add("Limelight TV", LimelightHelpers.getTV("limelight"))
      .withWidget("Boolean Box")
      .getEntry();

    // This adds the Limelight cl value to Shuffleboard
    limelightCL = Shuffleboard.getTab("Limelight")
      .add("Limelight CL", LimelightHelpers.getLimelightNTDouble("limelight", "cl"))
      .getEntry();
    
      // ! This is the default tab that the shuffleboard will open to.
    Shuffleboard.selectTab("Drive Tab");
  }

  public void setPipelineUsingAllianceColor() {
    alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        LimelightHelpers.setPipelineIndex("limelight", 1);

      }
      if (alliance.get() == Alliance.Blue) {
        // Put what you want it to do here.
        LimelightHelpers.setPipelineIndex("limelight", 0);
      }
    }
   
  }

  public double tX() {
    return limelightTX.getDouble(0.0);
  }
  public double tY() {
    return limelightTY.getDouble(0.0);
  }
  public double tA() {
    return limelightTA.getDouble(0.0);
  }
  public boolean hasTarget() {
    return limelightTV.getBoolean(false);
  }

  public boolean isLocked()
  {
    return (tX() < 0.5 && tX() > -0.5) && (tY() < 0.5 && tY() > -0.5);
  }

  // * This is for the Limelight going to Shuffleboard.
  public void updateShuffleboardLimelight()  {
    limelightTX.setDouble(LimelightHelpers.getTX("limelight"));
    limelightTY.setDouble(LimelightHelpers.getTY("limelight"));
    limelightTA.setDouble(LimelightHelpers.getTA("limelight"));
    limelightPipelineIndex.setDouble(LimelightHelpers.getCurrentPipelineIndex("limelight"));
    limelightTV.setBoolean(LimelightHelpers.getTV("limelight"));
    limelightCL.setDouble(LimelightHelpers.getLimelightNTDouble("limelight", "cl"));
  }

  // ! This is for the Limelight-Intake going to Shuffleboard.
  public void updateShuffleboardLimelightIntake() {
    limelightIntakeTX.setDouble(LimelightHelpers.getTX("limelight-intake"));
    limelightIntakeTY.setDouble(LimelightHelpers.getTY("limelight-intake"));
    limelightIntakeTA.setDouble(LimelightHelpers.getTA("limelight-intake"));
    limelightIntakePipelineIndex.setDouble(LimelightHelpers.getCurrentPipelineIndex("limelight-intake"));
    limelightIntakeTV.setBoolean(LimelightHelpers.getTV("limelight-intake"));
    limelightIntakeCL.setDouble(LimelightHelpers.getLimelightNTDouble("limelight-intake", "cl"));
  }

}
