// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.thumbwheel.Thumbwheel;

public class ClimberSubsystem extends SubsystemBase {

  // Create motors, encoder, and PID
  private CANSparkMax climberMasterMotor;
  private CANSparkMax climberSlaveMotor;
  private RelativeEncoder climberEncoder;
  private SparkPIDController climberPID;
  private int thumbwheelValue;
  Thumbwheel THUMBWHEEL;
  

  //This is for Shuffleboard.
  GenericEntry climberPosition;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(Thumbwheel thumbwheel) {

    THUMBWHEEL = thumbwheel;

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

    shuffleBoardInit();

  }

  @Override
  public void periodic() {
    
    thumbwheelValue = THUMBWHEEL.getValue();

    if(thumbwheelValue == 15)
    {
      climberMasterMotor.setIdleMode(IdleMode.kCoast);
      climberSlaveMotor.setIdleMode(IdleMode.kCoast);
    }
    else
    {
      climberMasterMotor.setIdleMode(ClimberConfig.idleMode);
      climberSlaveMotor.setIdleMode(ClimberConfig.idleMode);
    }

    updateShuffleboard();
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

  public void shuffleBoardInit() {
    // Add to Shuffleboard
    //SmartDashboard.putNumber("Climber Position", get());

    climberPosition = Shuffleboard.getTab("Info")
      .add("Climber Position", climberEncoder.getPosition())
      .withWidget("Number Bar")
      .withProperties(Map.of("min", 0, "max", 290))
      .withSize(2, 1)
      .withPosition(0, 0)
      .getEntry();

    }

    public void updateShuffleboard()
    {
      climberPosition.setDouble(climberEncoder.getPosition());
    }

}
