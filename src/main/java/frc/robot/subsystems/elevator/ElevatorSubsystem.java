// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.deck.DeckConfig;

public class ElevatorSubsystem extends SubsystemBase {

  // Create motor, encoder, and PID instances
  private CANSparkMax elevatorMotor;
  private RelativeEncoder elevatorEncoder;
  private SparkPIDController elevatorPID;

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

    // Set to position mode
    elevatorPID.setReference(get(), ElevatorConfig.controlType);

    // Burn Flash
    elevatorMotor.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", get());
  }

  // Get position
  public double get() {
    return elevatorEncoder.getPosition();
  }

  // Set position
  public void set(double position) {
    elevatorPID.setReference(position, DeckConfig.controlType);
  }

}
