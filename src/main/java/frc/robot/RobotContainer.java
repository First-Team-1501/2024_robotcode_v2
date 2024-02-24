// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.subsystems.Thumbwheel;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.deck.DeckSubsystem;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.stabilizer.StabilizerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.limelight.LimelightHelpers; // Import the LimelightHelpers class
import frc.robot.limelight.LimelightHelpers.LimelightResults; // Import the LimelightResults class
import frc.robot.limelight.LimelightHelpers.LimelightTarget_Fiducial; // Import the LimelightTarget_Fiducial class
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
  //Imports for the Limelight
  LimelightHelpers limelight = new LimelightHelpers(); // Create a new instance of the LimelightHelpers class
  LimelightTarget_Fiducial m_Fiducial = new LimelightTarget_Fiducial(); // Create a new instance of the LimelightTarget_Fiducial class 
  
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


  //private TeleopCommands teleop;
  private AutoCommands auto;
  private TeleopCommands teleop;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() 
  {
    
    thumb = new Thumbwheel();
  
    //Stuff for the Limelight
    //This is the value from the Limelight.
    double tx = LimelightHelpers.getTX("tx");
    double ty = LimelightHelpers.getTY("ty");
    double ta = LimelightHelpers.getTA("ta");
    double pipelineIndex = LimelightHelpers.getCurrentPipelineIndex("getpipe");
    double id = LimelightHelpers.getFiducialID("id");
    boolean tv = LimelightHelpers.getTV("tv");
    LimelightHelpers.setLEDMode_PipelineControl("");
    LimelightHelpers.getLimelightNTTable(null);
    LimelightHelpers.getLimelightNTTableEntry(null, "tid");
    LimelightHelpers.LimelightResults results = new LimelightResults();
    LimelightHelpers.getLimelightNTDouble(null, "cl");

    double cl = LimelightHelpers.getLimelightNTDouble(null, "cl");
    double ts = LimelightHelpers.getLimelightNTDouble(null, "tid");

    //This sends the value to the SmartDashboard.
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("ta", ta);
    SmartDashboard.putNumber("pipelineIndex", pipelineIndex);
    SmartDashboard.putNumber("id", id);
    SmartDashboard.putNumber("cl", cl);
    SmartDashboard.putNumber("ts", ts);
    SmartDashboard.putBoolean("Has Target", tv);

    drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"),  // DO NOT UNDER ANY CIRCUMSTANCE CHANGE THIS FROM
                                                                // "SWERVE/NEO"!!!!!!!!!!!!!!
      ()->{return thumb.getValue()>=8;}
    );

    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    deck = new DeckSubsystem();
    elevator =  new ElevatorSubsystem();
    climber = new ClimberSubsystem();
    stabilizer = new StabilizerSubsystem();
 

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
  StabilizerSubsystem getStabilizer(){return stabilizer;}
  Thumbwheel getThumbwheel(){return thumb;}

  
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
