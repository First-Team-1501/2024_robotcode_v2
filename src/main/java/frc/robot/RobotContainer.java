// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Climber.Climber;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Deck.Deck;
import frc.robot.Deck.PositionList;
import frc.robot.Deck.Commands.AdoptSetAngle;
import frc.robot.Elevator.DistanceList;
import frc.robot.Elevator.Elevator;
import frc.robot.Elevator.Commands.AdoptTargetDistance;
import frc.robot.Intake.Intake;
import frc.robot.Intake.Commands.NoteIntake;
import frc.robot.Intake.Commands.NoteOuttake;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.Commands.Shoot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;


import frc.robot.subsystems.leds.CANdleSystem;
import frc.robot.subsystems.leds.CANdleSystem.AnimationTypes;
import frc.robot.commands.leds.CANdleConfigCommands;
import frc.robot.commands.leds.CANdlePrintCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  Climber s_CLIMBER;
  Deck s_DECK;
  Elevator s_ELEVATOR;
  Intake s_INTAKE;
  Shooter s_SHOOTER;
  
//CHANGED SwerveSystem function from child:"swerve/neo" to "swerve"

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase;
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController;
  CommandJoystick rotationController;

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController operatorXbox;
  GenericHID buttonBoard;
  

    // LED lights
  //private final CANdleSystem m_candleSubsystem = new CANdleSystem(operatorXbox);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    
        //Initialize Subsystems
    // The robot's subsystems and commands are defined here...
   drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
   System.out.println("Drivebase Initialized");
                                                                      
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  driverController = new CommandJoystick(0);
  rotationController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  operatorXbox = new XboxController(2);
  buttonBoard = new GenericHID(3);

    /* We will not be using "Angluar Velocity for this robot" 
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
          () -> MathUtil.applyDeadband(driverXbox.getLeftY(),  OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(driverXbox.getLeftX(),  OperatorConstants.LEFT_X_DEADBAND),
          () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
                driverXbox::getYButtonPressed,
                driverXbox::getAButtonPressed,
                driverXbox::getXButtonPressed,
                driverXbox::getBButtonPressed);
    */

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation


      // The MINUS Sign is REQUIRED to FIX Inversion Issue - This is the JoyStick Control for the 2024 Robot
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(

        () -> -MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(rotationController.getRawAxis(0), OperatorConstants.RIGHT_X_DEADBAND));

    
         

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot

       /* We will not be using "Angluar Velocity for this robot"
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> rotationController.getRawAxis(0));
      */
        
    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(((operatorXbox.getLeftY() < .5) ? (operatorXbox.getLeftY()*2) : 1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(((operatorXbox.getLeftX() < .5) ? (operatorXbox.getLeftY()*2) : 1), OperatorConstants.LEFT_X_DEADBAND),
        () -> operatorXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    
    
    /* //Initialize & set default CANdle settings
    CANdleSystem candleLEDs = new CANdleSystem(operatorXbox);
    candleLEDs.changeAnimation(AnimationTypes.Rainbow);
    candleLEDs.configBrightness(100); */

    s_CLIMBER = new Climber();
    s_DECK = new Deck();
    s_ELEVATOR = new Elevator();
    s_INTAKE = new Intake();
    s_SHOOTER = new Shooter();
    System.out.println("Subsystems initialized");

    // Configure the trigger bindings
    configureBindings();
    System.out.println("Triggers Configured");

    //First code for the Camera server.
    CameraServer.startAutomaticCapture();
    //CvSink cvSink = CameraServer.getVideo();
    //CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);




  }
          
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    driverController.button(3).onTrue(new InstantCommand(drivebase::zeroGyro));
    new JoystickButton(operatorXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

  //  **OPERATOR BUTTONS** 
  //Deck Buttons
    Trigger shooter_RevCloseup = new Trigger( () -> operatorXbox.getXButton() )
      .whileTrue(new Shoot(s_SHOOTER, AdoptSetAngle.finished));

    Trigger intake = new Trigger( () -> operatorXbox.getRightBumper())
      .whileTrue(new NoteIntake(s_INTAKE));

    Trigger deck_SetCloseup = new Trigger( () -> operatorXbox.getYButton() )
      .whileTrue(new AdoptSetAngle(s_DECK, PositionList.CLOSEUP));
    
      Trigger deck_SetHome = new Trigger(()-> operatorXbox.getAButton())
    .whileTrue(new AdoptSetAngle(s_DECK, PositionList.HOME));

    Trigger elevatorOut = new Trigger( () -> buttonBoard.getRawButton(4))
    .whileTrue(new AdoptTargetDistance(s_ELEVATOR, DistanceList.INTAKE));

    Trigger elevatorIn = new Trigger( () -> buttonBoard.getRawButton(2))
    .whileTrue(new AdoptTargetDistance(s_ELEVATOR, DistanceList.ZERO));
    //Trigger deck_Intake = new Trigger( ()-> operatorXbox.)
    Trigger outtake = new Trigger( () -> operatorXbox.getLeftBumper())
    .whileTrue(new NoteOuttake(s_INTAKE));

    Trigger deck_SetIntake = new Trigger(()-> operatorXbox.getBButton())
    .whileTrue(new AdoptSetAngle(s_DECK, PositionList.INTAKE));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Path1", true);
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  
  
  public void autonomousInit()
  {
    //CANdleSystem candleLEDs = new CANdleSystem(operatorXbox);
    //candleLEDs.configBrightness(100);
    //candleLEDs.changeAnimation(AnimationTypes.Rainbow);
  

  }
}
