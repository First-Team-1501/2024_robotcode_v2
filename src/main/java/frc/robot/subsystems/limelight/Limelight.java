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
  /* 
  private double tx; //Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
  private double ty; //Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
  private double ta; //Target Area (0% of image to 100% of image)

  private double pipelineIndex;
  private boolean tv; //Whether the limelight has any valid targets (0 or 1)
  private double cl; //Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.

  private double tx2; //Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
  private double ty2; //Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
  private double ta2; //Target Area (0% of image to 100% of image)

  private double pipelineIndex2;
  private boolean tv2; //Whether the limelight has any valid targets (0 or 1)
  private double cl2; //Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.
*/
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
    //tx = LimelightHelpers.getTX("limelight");
    //ty = LimelightHelpers.getTY("limelight");
    //tx = LimelightHelpers.getTX("limelight-intake");
    //ty = LimelightHelpers.getTY("limelight-intake");
    updateShuffleboardLimelight();
    updateShuffleboardLimelightIntake();
  }

  private void setupLimelight() {

    setPipelineUsingAllianceColor();

    //tx = LimelightHelpers.getTX("limelight");
    //ty = LimelightHelpers.getTY("limelight");
    //ta = LimelightHelpers.getTA("limelight");
    //pipelineIndex = LimelightHelpers.getCurrentPipelineIndex("limelight");
    //tv = LimelightHelpers.getTV("limelight");
    //cl = LimelightHelpers.getLimelightNTDouble(null, "cl");

    /*tx2 = LimelightHelpers.getTX("limelight-intake");
    ty2 = LimelightHelpers.getTY("limelight-intake");
    ta2 = LimelightHelpers.getTA("limelight-intake");
    pipelineIndex2 = LimelightHelpers.getCurrentPipelineIndex("limelight-intake");
    tv2 = LimelightHelpers.getTV("limelight-intake");
    cl2 = LimelightHelpers.getLimelightNTDouble(null, "cl");*/


    

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
    // why did I do this???? For the intake limelight the pipeline is the same all the time.
    /*alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        LimelightHelpers.setPipelineIndex("limelight-intake", 1);

      }
      if (alliance.get() == Alliance.Blue) {
        // Put what you want it to do here.
        LimelightHelpers.setPipelineIndex("limelight-intake", 0);
      }
    }*/
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
    limelightTX.setDouble(LimelightHelpers.getTX("Limelight"));
    limelightTY.setDouble(LimelightHelpers.getTY("Limelight"));
    limelightTA.setDouble(LimelightHelpers.getTA("Limelight"));
    limelightPipelineIndex.setDouble(LimelightHelpers.getCurrentPipelineIndex("Limelight"));
    limelightTV.setBoolean(LimelightHelpers.getTV("Limelight"));
    limelightCL.setDouble(LimelightHelpers.getLimelightNTDouble("Limelight", "cl"));
  }

  // ! This is for the Limelight-Intake going to Shuffleboard.
  public void updateShuffleboardLimelightIntake() {
    limelightIntakeTX.setDouble(LimelightHelpers.getTX("Limelight-Intake"));
    limelightIntakeTY.setDouble(LimelightHelpers.getTY("Limelight-Intake"));
    limelightIntakeTA.setDouble(LimelightHelpers.getTA("Limelight-Intake"));
    limelightIntakePipelineIndex.setDouble(LimelightHelpers.getCurrentPipelineIndex("Limelight-Intake"));
    limelightIntakeTV.setBoolean(LimelightHelpers.getTV("Limelight-Intake"));
    limelightIntakeCL.setDouble(LimelightHelpers.getLimelightNTDouble("Limelight-Intake", "cl"));
  }

}