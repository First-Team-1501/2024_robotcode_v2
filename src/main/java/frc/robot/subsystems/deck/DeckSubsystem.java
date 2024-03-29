// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.deck;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeckSubsystem extends SubsystemBase {

  GenericEntry deckPosition;
  
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

    //SmartDashboard.putNumber("Deck Position", get());
    shuffleBoardInit();
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleboard();
    
  }

  // Get position
  public double get() {
    return deckEncoder.getPosition();
  }

  // Set position
  public void set(double position) {
    deckPID.setReference(position, DeckConfig.controlType);
  }

  public void resetEncoder()
  {
    deckEncoder.setPosition(0);
  }


  public void setMaxOutput(double output)
  {
    deckPID.setOutputRange(-output, output);
  }

  public void shuffleBoardInit()
  {
    // ? Added the deckEncoder.getPostion() to the line of code we will see if it works.
    //SmartDashboard.putNumber("Deck Position", deckEncoder.getPosition());

    //SmartDashboard.putNumber("Deck Position", get());

    deckPosition = Shuffleboard.getTab("Info")
    .add("Deck Position2", deckEncoder.getPosition())
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", 0, "max", 160))
    .withPosition(0, 2)
    .getEntry();
  }

  public void updateShuffleboard()
  {
    deckPosition.setDouble(get());
  }

}
