// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.climber.JogClimberDown;
import frc.robot.commands.climber.JogClimberUp;
import frc.robot.commands.climber.SetClimberPosition;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.RunOuttakeCommand;
import frc.robot.commands.intake.ShootNote;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.subsystems.Thumbwheel;
import frc.robot.subsystems.climber.ClimberPositions;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.deck.DeckSubsystem;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterConfig;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

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
  private Thumbwheel thumb;


  private TeleopCommands teleop;
  private AutoCommands auto;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() 
  {
    
    thumb = new Thumbwheel();

    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    deck = new DeckSubsystem();
    elevator =  new ElevatorSubsystem();
    climber = new ClimberSubsystem();
 

    teleop = new TeleopCommands(this);
    auto =  new AutoCommands(this);

  }

  // what follows is a bunch of getters that I couldn't possibly care about.

  SwerveSubsystem getDrivebase(){return drivebase;}
  IntakeSubsystem getIntake(){ return intake;}
  ShooterSubsystem getShooter(){return shooter;}
  DeckSubsystem getDeck(){return deck;}
  ElevatorSubsystem getElevator(){return elevator;}
  ClimberSubsystem getClimber(){return climber;}
  Thumbwheel getThumbwheel(){return getThumbwheel();}

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  { 
    return auto.SelectAuto();
  }

  // Default commands - these are setting the default positions for the elevator
  // and the deck
  public void defaultCommands() 
  {
    deck.setDefaultCommand(new SetDeckPosition(deck, DeckPositions.home));
    elevator.setDefaultCommand(new SetElevatorPosition(elevator, ElevatorPositions.zero));
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

}
