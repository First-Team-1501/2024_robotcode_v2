// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.Map;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.deck.DeckConfig;

public class ElevatorSubsystem extends SubsystemBase {

  // Create motor, encoder, and PID instances
  private CANSparkMax elevatorMotor;
  private RelativeEncoder elevatorEncoder;
  private SparkPIDController elevatorPID;

  //This is for Shuffleboard.
  GenericEntry elevatorPosition;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    // Initilize motor
    elevatorMotor = new CANSparkMax(ElevatorConfig.ID, ElevatorConfig.motorType);

    // Initialize encoder instance
    elevatorEncoder = elevatorMotor.getEncoder();

    // Initialize PID instance
    elevatorPID = elevatorMotor.getPIDController();

    // PID Configuration
    elevatorPID.setP(ElevatorConfig.p);
    elevatorPID.setI(ElevatorConfig.i);
    elevatorPID.setD(ElevatorConfig.d);
    elevatorPID.setIZone(ElevatorConfig.IZone);
    elevatorPID.setDFilter(ElevatorConfig.DFilter);

    // Set output range
    elevatorPID.setOutputRange(ElevatorConfig.outputMin, ElevatorConfig.outputMax);

    // Set ramp rate
    elevatorMotor.setOpenLoopRampRate(ElevatorConfig.openRampRate);
    elevatorMotor.setClosedLoopRampRate(ElevatorConfig.closedRampRate);

    // Set inversion
    elevatorMotor.setInverted(ElevatorConfig.kInverted);

    // Set current limits
    elevatorMotor.setSmartCurrentLimit(ElevatorConfig.smartCurrentStallLimit, ElevatorConfig.smartCurrentFreeLimit);

    // Set position conversion factor
    elevatorEncoder.setPositionConversionFactor(ElevatorConfig.positionConversionFactor);

    // Set forward soft limit
    elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, ElevatorConfig.softLimitFwdEnabled);
    elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, ElevatorConfig.softLimitFwd);

    // Set reverse soft limit
    elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, ElevatorConfig.softLimitRevEnabled);
    elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, ElevatorConfig.softLimitRev);

    // Set position to zero
    elevatorEncoder.setPosition(0);

    // Burn Flash
    elevatorMotor.burnFlash();

    // Set to position mode
    elevatorPID.setReference(get(), ElevatorConfig.controlType);

    shuffleBoardInit();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Elevator Position", get());
    updateShuffleboard();
  }

  // Get position
  public double get() {
    return elevatorEncoder.getPosition();
  }

  // Set position
  public void set(double position) {
    elevatorPID.setReference(position, DeckConfig.controlType);
  }

  public void resetEncoder()
  {
    elevatorEncoder.setPosition(0);
  }

  public void setMaxOutput(double output)
  {
    elevatorPID.setOutputRange(-output, output);
  }

  public void changeAmpLimits(int current)
  {
    elevatorMotor.setSmartCurrentLimit(current, ElevatorConfig.smartCurrentFreeLimit);
  }

  public void shuffleBoardInit()
  {
    // Add to Shuffleboard
    //SmartDashboard.putNumber("Elevator Position", get());

   elevatorPosition = Shuffleboard.getTab("Info")
      .add("Elevator Position", elevatorEncoder.getPosition())
      .withWidget("Number Bar")
      .withProperties(Map.of("min", 0, "max", 50))
      .withSize(2, 1)
      .withPosition(1, 1)
      .getEntry();
  }

  public void updateShuffleboard()
  {
    // Update Shuffleboard
    elevatorPosition.setDouble(get());
  }
}
