// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import java.util.Map;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  // Create motors, encoder, and PID
  private CANSparkMax climberMasterMotor;
  private CANSparkMax climberSlaveMotor;
  private RelativeEncoder climberEncoder;
  private SparkPIDController climberPID;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // Initialize motors
    climberMasterMotor = new CANSparkMax(ClimberConfig.ID, ClimberConfig.motorType);
    climberSlaveMotor = new CANSparkMax(ClimberConfig.followerID, ClimberConfig.motorType);

    climberMasterMotor.setIdleMode(ClimberConfig.idleMode);
    climberSlaveMotor.setIdleMode(ClimberConfig.idleMode);

    // Initialize encoder instance
    climberEncoder = climberMasterMotor.getEncoder();

    // Initialize PID instance
    climberPID = climberMasterMotor.getPIDController();

    // PID Config
    climberPID.setP(ClimberConfig.p);
    climberPID.setI(ClimberConfig.i);
    climberPID.setD(ClimberConfig.d);
    climberPID.setIZone(ClimberConfig.IZone);
    climberPID.setDFilter(ClimberConfig.DFilter);

    // Set output range
    climberPID.setOutputRange(ClimberConfig.outputMin, ClimberConfig.outputMax);

    // Set ramp rate
    climberMasterMotor.setOpenLoopRampRate(ClimberConfig.openRampRate);
    climberMasterMotor.setClosedLoopRampRate(ClimberConfig.closedRampRate);

    // Set inversion
    climberMasterMotor.setInverted(ClimberConfig.kInverted);

    // Set current limits
    climberMasterMotor.setSmartCurrentLimit(ClimberConfig.smartCurrentStallLimit, ClimberConfig.smartCurrentFreeLimit);

    // Set position conversion factor
    climberEncoder.setPositionConversionFactor(ClimberConfig.positionConversionFactor);

    // Set forward soft limit
    climberMasterMotor.enableSoftLimit(SoftLimitDirection.kForward, ClimberConfig.softLimitFwdEnabled);
    climberMasterMotor.setSoftLimit(SoftLimitDirection.kForward, ClimberConfig.softLimitFwd);

    // Set reverse soft limit
    climberMasterMotor.enableSoftLimit(SoftLimitDirection.kReverse, ClimberConfig.softLimitRevEnabled);
    climberMasterMotor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConfig.softLimitRev);

    // Configure slave motor
    climberSlaveMotor.follow(climberMasterMotor, ClimberConfig.follow_isInverted);

    // Set position to zero
    climberEncoder.setPosition(0);

    // Burn Flash
    climberMasterMotor.burnFlash();
    climberSlaveMotor.burnFlash();

    // Set to position mode
    climberPID.setReference(get(), ClimberConfig.controlType);

    ShuffleBoardInit();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Climber Position", get());
  }

  // Get position
  public double get() {
    return climberEncoder.getPosition();
  }

  // Set position
  public void set(double position) {
    climberPID.setReference(position, ClimberConfig.controlType);
  }

  public void resetEncoder()
  {
    climberEncoder.setPosition(0);
  }

  public void ShuffleBoardInit() {
    // Add to Shuffleboard
    //SmartDashboard.putNumber("Climber Position", get());

    Shuffleboard.getTab("Drive Tab")
      .add("Climber Position", climberEncoder.getPosition())
      .withWidget(BuiltInWidgets.kNumberBar)
      .withProperties(Map.of("min", 0, "max", 290))
      .withSize(2, 1)
      .withPosition(0, 0)
      .getEntry();

      
  }

}
