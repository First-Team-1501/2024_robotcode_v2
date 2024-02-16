// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterSubsystem;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  // Create the motors
  private CANSparkFlex leftMotor;
  private CANSparkFlex rightMotor;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // Initialize motors
    leftMotor = new CANSparkFlex(ShooterConfig.left_ID, ShooterConfig.left_motorType);
    rightMotor = new CANSparkFlex(ShooterConfig.right_ID, ShooterConfig.right_motorType);

    // Set idle mode
    leftMotor.setIdleMode(ShooterConfig.left_idleMode);
    rightMotor.setIdleMode(ShooterConfig.right_idleMode);

    // PID Configuration
    leftMotor.getPIDController().setP(ShooterConfig.left_p);
    leftMotor.getPIDController().setI(ShooterConfig.left_i);
    leftMotor.getPIDController().setD(ShooterConfig.left_d);
    leftMotor.getPIDController().setIZone(ShooterConfig.left_IZone);
    leftMotor.getPIDController().setDFilter(ShooterConfig.left_DFilter);

    rightMotor.getPIDController().setP(ShooterConfig.right_p);
    rightMotor.getPIDController().setI(ShooterConfig.right_i);
    rightMotor.getPIDController().setD(ShooterConfig.right_d);
    rightMotor.getPIDController().setIZone(ShooterConfig.right_IZone);
    rightMotor.getPIDController().setDFilter(ShooterConfig.right_DFilter);

    // Set output range
    leftMotor.getPIDController().setOutputRange(ShooterConfig.left_outputMin, ShooterConfig.left_outputMax);
    rightMotor.getPIDController().setOutputRange(ShooterConfig.right_outputMin, ShooterConfig.right_outputMax);

    // Set ramp rate
    leftMotor.setOpenLoopRampRate(ShooterConfig.left_openRampRate);
    rightMotor.setOpenLoopRampRate(ShooterConfig.right_openRampRate);

    // Set inversion
    leftMotor.setInverted(ShooterConfig.left_kInverted);
    rightMotor.setInverted(ShooterConfig.right_kInverted);

    // Set current limits
    leftMotor.setSmartCurrentLimit(ShooterConfig.left_smartCurrentStallLimit, ShooterConfig.left_smartCurrentFreeLimit);
    rightMotor.setSmartCurrentLimit(ShooterConfig.right_smartCurrentStallLimit,
        ShooterConfig.right_smartCurrentFreeLimit);

    // Burn flash
    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Set speed of left and right motors
  public void set(double leftSpeed, double rightSpeed) {
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }

  // Stop motors
  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

}
