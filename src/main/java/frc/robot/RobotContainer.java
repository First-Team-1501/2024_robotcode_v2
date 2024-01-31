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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;

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
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(0);
  CommandJoystick rotationController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(2);

    // LED lights
  private final CANdleSystem m_candleSubsystem = new CANdleSystem(driverXbox);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

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

        /* USE THIS FOR NEW JOYSTICKS
        () -> MathUtil.applyDeadband(((driverController.getY() < .5) ? (driverController.getY()*2) : 1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(((driverController.getX() < .5) ? (driverController.getX()*2) : 1), OperatorConstants.LEFT_X_DEADBAND),
         */

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
        () -> MathUtil.applyDeadband(((driverXbox.getLeftY() < .5) ? (driverXbox.getLeftY()*2) : 1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(((driverXbox.getLeftX() < .5) ? (driverXbox.getLeftY()*2) : 1), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    
    
    //Initialize & set default CANdle settings
    CANdleSystem candleLEDs = new CANdleSystem(driverXbox);
    candleLEDs.changeAnimation(AnimationTypes.Rainbow);
    candleLEDs.configBrightness(100);


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
    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

   // LED Buttons to try
    //new JoystickButton(driverXbox, Constants.BlockButton).onTrue(new RunCommand(m_candleSubsystem::setColors, m_candleSubsystem));
    new JoystickButton(driverXbox, Constants.IncrementAnimButton).onTrue(new RunCommand(m_candleSubsystem::incrementAnimation, m_candleSubsystem));
    new JoystickButton(driverXbox, Constants.DecrementAnimButton).onTrue(new RunCommand(m_candleSubsystem::decrementAnimation, m_candleSubsystem));
    //This seems to work the best. Ross wrote this.
    //new POVButton(driverXbox, Constants.MaxBrightnessAngle).onTrue(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 1.0));
    //new POVButton(driverXbox, Constants.MidBrightnessAngle).onTrue(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0.3));
    //new POVButton(driverXbox, Constants.ZeroBrightnessAngle).onTrue(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0));

    new JoystickButton(driverXbox, Constants.VbatButton).onTrue(new CANdlePrintCommands.PrintVBat(m_candleSubsystem));
    new JoystickButton(driverXbox, Constants.V5Button).onTrue(new CANdlePrintCommands.Print5V(m_candleSubsystem));
    new JoystickButton(driverXbox, Constants.CurrentButton).onTrue(new CANdlePrintCommands.PrintCurrent(m_candleSubsystem));
    new JoystickButton(driverXbox, Constants.TemperatureButton).onTrue(new CANdlePrintCommands.PrintTemperature(m_candleSubsystem));

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
    CANdleSystem candleLEDs = new CANdleSystem(driverXbox);
    candleLEDs.configBrightness(100);
    candleLEDs.changeAnimation(AnimationTypes.Rainbow);
  

  }
}
