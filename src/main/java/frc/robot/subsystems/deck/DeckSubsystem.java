// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.deck;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeckSubsystem extends SubsystemBase {

  // Create motor, encoder, and PID instances
  private CANSparkMax deckMotor;
  private RelativeEncoder deckEncoder;
  private SparkPIDController deckPID;

  /** Creates a new DeckSubsystem. */
  public DeckSubsystem() {
    // Initialize motor
    deckMotor = new CANSparkMax(DeckConfig.ID, DeckConfig.motorType);

    // Initialize encoder instance
    deckEncoder = deckMotor.getEncoder();

    // Initialize PID instance
    deckPID = deckMotor.getPIDController();

    // PID Configuration
    deckPID.setP(DeckConfig.p);
    deckPID.setI(DeckConfig.i);
    deckPID.setD(DeckConfig.d);
    deckPID.setIZone(DeckConfig.IZone);
    deckPID.setDFilter(DeckConfig.DFilter);

    // Set output range
    deckPID.setOutputRange(DeckConfig.outputMin, DeckConfig.outputMax);

    // Set ramp rate
    deckMotor.setOpenLoopRampRate(DeckConfig.openRampRate);
    deckMotor.setClosedLoopRampRate(DeckConfig.closedRampRate);

    // Set inversion
    deckMotor.setInverted(DeckConfig.kInverted);

    // Set current limits
    deckMotor.setSmartCurrentLimit(DeckConfig.smartCurrentStallLimit, DeckConfig.smartCurrentFreeLimit);

    // Set position conversion factor
    deckEncoder.setPositionConversionFactor(DeckConfig.positionConversionFactor);

    // Set forward soft limit
    deckMotor.enableSoftLimit(SoftLimitDirection.kForward, DeckConfig.softLimitFwdEnabled);
    deckMotor.setSoftLimit(SoftLimitDirection.kForward, DeckConfig.softLimitFwd);

    // Set reverse soft limit
    deckMotor.enableSoftLimit(SoftLimitDirection.kReverse, DeckConfig.softLimitRevEnabled);
    deckMotor.setSoftLimit(SoftLimitDirection.kReverse, DeckConfig.softLimitRev);

    // Set position to zero
    deckEncoder.setPosition(0);

    // Burn Flash
    deckMotor.burnFlash();



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Deck Position", get());
  }

  // Get position
  public double get() {
    return deckEncoder.getPosition();
  }

  // Set position
  public void set(double position) {
    deckPID.setReference(position, DeckConfig.controlType);
  }

}
