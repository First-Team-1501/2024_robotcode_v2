// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.deck.DeckConfig;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.shooter.ShooterConfig;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
  public enum ShootParams
  {
      CloseUp(DeckPositions.closeup, ShooterConfig.closeLeftSpeed, ShooterConfig.closeRightSpeed),
      Auto1(DeckPositions.closeup, ShooterConfig.closeLeftSpeed, ShooterConfig.closeRightSpeed),
      Auto2(DeckPositions.closeup, ShooterConfig.auto2LeftSpeed, ShooterConfig.auto2RightSpeed),
      Auto3(DeckPositions.closeup, ShooterConfig.closeRightSpeed, ShooterConfig.closeLeftSpeed),

      Podium(DeckPositions.podium, ShooterConfig.podiumLeftSpeed, ShooterConfig.podiumRightSpeed),
      Far(DeckPositions.backline, ShooterConfig.farLeftSpeed, ShooterConfig.farRightSpeed);


      
      private double deckPosition;
      private double leftSpeed;
      private double rightSpeed;

      private ShootParams(double deck, double left, double right)
      {
          deckPosition = deck;
          leftSpeed = left;
          rightSpeed = right;
      }

      public double getDeckPosition()
      {
        return deckPosition;
      }

      public double getLeftSpeed()
      {
        return leftSpeed;
      }

      public double getRightSpeed()
      {
        return rightSpeed;
      }
  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class Auton

  {
    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.6, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  // LED Constants
  public static final int CANdleID = 50;
  public static final int JoystickId = 2;
  public static final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
  public static final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;
  public static final int BlockButton = XboxController.Button.kStart.value;
  public static final int MaxBrightnessAngle = 90;
  public static final int MidBrightnessAngle = 180;
  public static final int ZeroBrightnessAngle = 270;
  public static final int VbatButton = XboxController.Button.kA.value;
  public static final int V5Button = XboxController.Button.kB.value;
  public static final int CurrentButton = XboxController.Button.kX.value;
  public static final int TemperatureButton = XboxController.Button.kY.value;

}
