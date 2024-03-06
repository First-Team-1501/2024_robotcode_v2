// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.deck.DeckSubsystem;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.stabilizer.StabilizerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.thumbwheel.Thumbwheel;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Swerve subsystem
  private final SwerveSubsystem drivebase;

  // Our subsystems
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private DeckSubsystem deck;
  private ElevatorSubsystem elevator;
  private ClimberSubsystem climber;
  private StabilizerSubsystem stabilizer;
  private Thumbwheel thumb;
  private Limelight limelight;

  // Stuff for the CANdle
  RainbowAnimation rAnimation = new RainbowAnimation(1, 1, 8);
  CANdle candle1 = new CANdle(48, "canivore");
  FireAnimation fireAnimation = new FireAnimation(01, 0, 90, 1, 0);
  LarsonAnimation larsonAnimation = new LarsonAnimation(52, 155, 235);
  RgbFadeAnimation rgbAnimation = new RgbFadeAnimation();
  StrobeAnimation strobeAnimation = new StrobeAnimation(255, 255, 255, 255, 1, 8);
  TwinkleAnimation twinklewhiteAnimation = new TwinkleAnimation(255, 255, 255);
  TwinkleAnimation twinkleblueAnimation = new TwinkleAnimation(0, 0, 255);
  TwinkleAnimation twinkleredAnimation = new TwinkleAnimation(255, 0, 0);
  TwinkleOffAnimation twinkleOffAnimation = new TwinkleOffAnimation(255, 255, 255);
  SingleFadeAnimation singleFadeAnimation = new SingleFadeAnimation(255, 255, 255, 255, 1, 8);

  // private TeleopCommands teleop;
  private AutoCommands auto;
  private TeleopCommands teleop;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    thumb = new Thumbwheel();

    drivebase = new SwerveSubsystem(
        new File(Filesystem.getDeployDirectory(), "swerve/neo") // DO NOT UNDER ANY CIRCUMSTANCE CHANGE THIS FROM //
                                                                // "SWERVE/NEO"!!!!!!!!!!!!!!

    );

    intake = new IntakeSubsystem(candle1);
    shooter = new ShooterSubsystem();
    deck = new DeckSubsystem();
    elevator = new ElevatorSubsystem();
    climber = new ClimberSubsystem();
    stabilizer = new StabilizerSubsystem();
    limelight = new Limelight();

    teleop = new TeleopCommands(this);
    auto = new AutoCommands(this);

  }

  // Getters
  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }

  public IntakeSubsystem getIntake() {
    return intake;
  }

  public ShooterSubsystem getShooter() {
    return shooter;
  }

  public DeckSubsystem getDeck() {
    return deck;
  }

  public ElevatorSubsystem getElevator() {
    return elevator;
  }

  public ClimberSubsystem getClimber() {
    return climber;
  }

  public StabilizerSubsystem getStabilizer() {
    return stabilizer;
  }

  public Thumbwheel getThumbwheel() {
    return thumb;
  }

  public CANdle getCandle() {
    return candle1;
  }

  public Limelight getLimelight() {
    return limelight;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto.SelectAuto();
  }

  public double limelight_aim_proportional() {

    double kP = 0.013;
    double kI = 0;
    double kD = 0.0000001;
    double maxTolerance = 0.6;
    double baseValue = 0.28;

    try (PIDController pidCont = new PIDController(kP, kI, kD)) {
      double targetingAngularVelocity = pidCont.calculate(-limelight.tX());

      if (targetingAngularVelocity > 0) {
        targetingAngularVelocity += baseValue;
      } else {
        targetingAngularVelocity -= baseValue;
      }

      if (targetingAngularVelocity < 0.3 && targetingAngularVelocity > -0.3)
        return 0;

      if (targetingAngularVelocity > maxTolerance) {
        targetingAngularVelocity = maxTolerance;
      } else if (targetingAngularVelocity < -maxTolerance) {
        targetingAngularVelocity = -maxTolerance;
      }

      SmartDashboard.putNumber("Aim PID Calc", targetingAngularVelocity);

      return targetingAngularVelocity;
    }
  }

  // Default commands - these are setting the default positions for the elevator
  // and the deck
  public void defaultCommands() {
    deck.setDefaultCommand(new SetDeckPosition(deck, DeckPositions.home));
    elevator.setDefaultCommand(new SetElevatorPosition(elevator, ElevatorPositions.zero));
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public Command onTeleopInit() {
    return teleop.onTeleopInit();
  }

}
