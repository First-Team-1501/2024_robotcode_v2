// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;




import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {

  // Create the motors
  private CANSparkMax topMotor;
  private CANSparkMax bottomMotor;

  private DigitalInput intakeSensor;
  private DigitalInput outtakeSensor;



  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    //candle1 = candle;

    // Initialize motors
    topMotor = new CANSparkMax(IntakeConfig.top_ID, IntakeConfig.top_motorType);
    bottomMotor = new CANSparkMax(IntakeConfig.bottom_ID, IntakeConfig.bottom_motorType);

    // Set idle mode
    topMotor.setIdleMode(IntakeConfig.top_idleMode);
    bottomMotor.setIdleMode(IntakeConfig.bottom_idleMode);

    // PID Configuration
    topMotor.getPIDController().setP(IntakeConfig.top_p);
    topMotor.getPIDController().setI(IntakeConfig.top_i);
    topMotor.getPIDController().setD(IntakeConfig.top_d);
    topMotor.getPIDController().setIZone(IntakeConfig.top_IZone);
    topMotor.getPIDController().setDFilter(IntakeConfig.top_DFilter);

    bottomMotor.getPIDController().setP(IntakeConfig.bottom_p);
    bottomMotor.getPIDController().setI(IntakeConfig.bottom_i);
    bottomMotor.getPIDController().setD(IntakeConfig.bottom_d);
    bottomMotor.getPIDController().setIZone(IntakeConfig.bottom_IZone);
    bottomMotor.getPIDController().setDFilter(IntakeConfig.bottom_DFilter);

    // Set output range
    topMotor.getPIDController().setOutputRange(IntakeConfig.top_outputMin, IntakeConfig.top_outputMax);
    bottomMotor.getPIDController().setOutputRange(IntakeConfig.bottom_outputMin, IntakeConfig.bottom_outputMax);

    // Set ramp rate
    topMotor.setOpenLoopRampRate(IntakeConfig.top_openRampRate);
    bottomMotor.setOpenLoopRampRate(IntakeConfig.bottom_openRampRate);

    // Set inversion
    topMotor.setInverted(IntakeConfig.top_kInverted);
    bottomMotor.setInverted(IntakeConfig.bottom_kInverted);

    // Set current limits
    topMotor.setSmartCurrentLimit(IntakeConfig.top_smartCurrentStallLimit, IntakeConfig.top_smartCurrentFreeLimit);
    bottomMotor.setSmartCurrentLimit(IntakeConfig.bottom_smartCurrentStallLimit,
        IntakeConfig.bottom_smartCurrentFreeLimit);

    // Burn flash
    topMotor.burnFlash();
    bottomMotor.burnFlash();

    // Initialize digital inputs
    intakeSensor = new DigitalInput(IntakeConfig.intakeSensorID);
    outtakeSensor = new DigitalInput(IntakeConfig.outtakeSensorID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Has Note", hasNote());
    SmartDashboard.putBoolean("Outtake Sensor", readyToScoreTrap());

  

  }

  // Function to set the motor speeds
  public void set(double topSpeed, double bottomSpeed) {
    topMotor.set(topSpeed);
    bottomMotor.set(bottomSpeed);
  }

  // Function to stop the motors
  public void stop() {
    topMotor.set(0);
    bottomMotor.set(0);
  }

  // Function to see if note intake is complete
  public boolean hasNote() {
    return intakeSensor.get();
  }

  // Function to see if note is ready to outtake (to score in the trap)
  public boolean readyToScoreTrap() {
    return (outtakeSensor.get());
  }

}
