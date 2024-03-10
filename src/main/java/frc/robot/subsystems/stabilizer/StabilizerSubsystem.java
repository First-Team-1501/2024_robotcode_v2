// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.stabilizer;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StabilizerSubsystem extends SubsystemBase {
  
  // Create motor, encoder, and PID instances
  private CANSparkMax stabilizerMotor;
  private RelativeEncoder stabilizerEncoder;
  private SparkPIDController stabilizerPID;
  
  /** Creates a new StabilizerSubsystem. */
  public StabilizerSubsystem() 
  {

    // Initialize motor
    stabilizerMotor = new CANSparkMax(StabilizerConfig.ID, StabilizerConfig.motorType);

    // Initialize encoder instance
    stabilizerEncoder = stabilizerMotor.getEncoder();

    // Initialize PID instance
    stabilizerPID = stabilizerMotor.getPIDController();

    // PID Configuration
    stabilizerPID.setP(StabilizerConfig.p);
    stabilizerPID.setI(StabilizerConfig.i);
    stabilizerPID.setD(StabilizerConfig.d);
    stabilizerPID.setIZone(StabilizerConfig.IZone);
    stabilizerPID.setDFilter(StabilizerConfig.DFilter);

    // Set output range
    stabilizerPID.setOutputRange(StabilizerConfig.outputMin, StabilizerConfig.outputMax);

    // Set ramp rate
    stabilizerMotor.setOpenLoopRampRate(StabilizerConfig.openRampRate);
    stabilizerMotor.setClosedLoopRampRate(StabilizerConfig.closedRampRate);

    // Set inversion
    stabilizerMotor.setInverted(StabilizerConfig.kInverted);

    // Set current limits
    stabilizerMotor.setSmartCurrentLimit(StabilizerConfig.smartCurrentStallLimit, StabilizerConfig.smartCurrentFreeLimit);

    // Set position conversion factor
    stabilizerEncoder.setPositionConversionFactor(StabilizerConfig.positionConversionFactor);

    // Set forward soft limit
    stabilizerMotor.enableSoftLimit(SoftLimitDirection.kForward, StabilizerConfig.softLimitFwdEnabled);
    stabilizerMotor.setSoftLimit(SoftLimitDirection.kForward, StabilizerConfig.softLimitFwd);

    // Set reverse soft limit
    stabilizerMotor.enableSoftLimit(SoftLimitDirection.kReverse, StabilizerConfig.softLimitRevEnabled);
    stabilizerMotor.setSoftLimit(SoftLimitDirection.kReverse, StabilizerConfig.softLimitRev);

    // Set position to zero
    stabilizerEncoder.setPosition(0);

    // Burn Flash
    stabilizerMotor.burnFlash();

    set(StabilizerPositions.zero);

    ShuffleBoardInit();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Stabilizer Position", get());
  }

  // Get position
  public double get() {
    return stabilizerEncoder.getPosition();
  }

  // Set position
  public void set(double position) {
    stabilizerPID.setReference(position, StabilizerConfig.controlType);
  }

  public void resetEncoder()
  {
    stabilizerEncoder.setPosition(0);
  }

  public void ShuffleBoardInit()
  {
    //SmartDashboard.putNumber("Stabilizer Position", get());

    Shuffleboard.getTab("Stabilizer")
      .add("Stabilizer Position", stabilizerEncoder.getPosition())
      .getEntry();
  }

}
