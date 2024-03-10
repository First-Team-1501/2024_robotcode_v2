// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class Limelight extends SubsystemBase {

  Optional<Alliance> alliance;

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

  /** Creates a new Limelight. */
  public Limelight() {

    setupLimelight();

  }

  @Override
  public void periodic() {
    setPipelineUsingAllianceColor();
    tx = LimelightHelpers.getTX("limelight");
    ty = LimelightHelpers.getTY("limelight");
    tx = LimelightHelpers.getTX("limelight2");
    ty = LimelightHelpers.getTY("limelight2");
  }

  private void setupLimelight() {

    setPipelineUsingAllianceColor();

    tx = LimelightHelpers.getTX("limelight");
    ty = LimelightHelpers.getTY("limelight");
    ta = LimelightHelpers.getTA("limelight");
    pipelineIndex = LimelightHelpers.getCurrentPipelineIndex("limelight");
    tv = LimelightHelpers.getTV("limelight");
    cl = LimelightHelpers.getLimelightNTDouble(null, "cl");

    tx2 = LimelightHelpers.getTX("limelight2");
    ty2 = LimelightHelpers.getTY("limelight2");
    ta2 = LimelightHelpers.getTA("limelight2");
    pipelineIndex2 = LimelightHelpers.getCurrentPipelineIndex("limelight2");
    tv2 = LimelightHelpers.getTV("limelight2");
    cl2 = LimelightHelpers.getLimelightNTDouble(null, "cl");


    

    LimelightHelpers.getLimelightNTTable(null);
    LimelightHelpers.getLimelightNTTableEntry(null, "tid");


    LimelightHelpers.setLEDMode_PipelineControl("limelight");

    smartDashboardInit();
    

  }

  public void smartDashboardInit() {

    SmartDashboard.putNumber("Limelight tX", tx);
    SmartDashboard.putNumber("Limelight tY", ty);
    SmartDashboard.putNumber("Limelight tA", ta);
    SmartDashboard.putNumber("Pipeline Index", pipelineIndex);
    SmartDashboard.putNumber("Limelight CL", cl);
    SmartDashboard.putBoolean("Has Target", tv);
    SmartDashboard.putBoolean("TargetLocked", isLocked());

    SmartDashboard.putNumber("Limelight tX", tx2);
    SmartDashboard.putNumber("Limelight tY", ty2);
    SmartDashboard.putNumber("Limelight tA", ta2);
    SmartDashboard.putNumber("Pipeline Index", pipelineIndex2);
    SmartDashboard.putNumber("Limelight CL", cl2);
    SmartDashboard.putBoolean("Has Target", tv2);
    //SmartDashboard.putBoolean("TargetLocked", isLocked());

    Shuffleboard.getTab("Drive Tab")
    .add("Target Locked", isLocked())
    .withWidget("Boolean Box")
    .withPosition(2, 2)
    .getEntry();

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

    alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        LimelightHelpers.setPipelineIndex("limelight2", 1);

      }
      if (alliance.get() == Alliance.Blue) {
        // Put what you want it to do here.
        LimelightHelpers.setPipelineIndex("limelight2", 0);
      }
    }
  }

  public double tX() {
    return tx;
  }
  public double tY() {
    return ty;
  }
  public double tA() {
    return ta;
  }
  public boolean hasTarget() {
    return tv;
  }

  public boolean isLocked()
  {
    return (tX() < 0.5 && tX() > -0.5) && (tY() < 0.5 && tY() > -0.5);
  }

}
