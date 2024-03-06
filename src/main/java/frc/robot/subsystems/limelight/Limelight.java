// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  /** Creates a new Limelight. */
  public Limelight() {

    setupLimelight();

  }

  @Override
  public void periodic() {

  }

  private void setupLimelight() {

    tx = LimelightHelpers.getTX("limelight");
    ty = LimelightHelpers.getTY("limelight");
    ta = LimelightHelpers.getTA("limelight");
    pipelineIndex = LimelightHelpers.getCurrentPipelineIndex("limelight");
    tv = LimelightHelpers.getTV("limelight");
    cl = LimelightHelpers.getLimelightNTDouble(null, "cl");

    alliance = DriverStation.getAlliance();

    LimelightHelpers.setLEDMode_PipelineControl("limelight");

    smartDashboardInit();
    setPipelineUsingAllianceColor();

  }

  public void smartDashboardInit() {

    SmartDashboard.putNumber("Limelight tX", tx);
    SmartDashboard.putNumber("Limelight tY", ty);
    SmartDashboard.putNumber("Limelight tA", ta);
    SmartDashboard.putNumber("Pipeline Index", pipelineIndex);
    SmartDashboard.putNumber("Limelight CL", cl);
    SmartDashboard.putBoolean("Has Target", tv);
  }

  public void setPipelineUsingAllianceColor() {

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

}
