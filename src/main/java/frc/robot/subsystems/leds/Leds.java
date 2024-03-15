// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limelight.LimelightHelpers;
import java.util.Optional;

public class Leds extends SubsystemBase {

  Optional<Alliance> alliance;
  private TwinkleAnimation twinkleblueAnimation;
  private TwinkleAnimation twinkleredAnimation;
  private StrobeAnimation strobeAnimation;
  private StrobeAnimation strobeintakeAnimation;

  private final double STROBE_SPEED = 98.0 / 256.0;

  private CANdle candle1;
  private static boolean intaking;

  /** Creates a new CANdle */
  public Leds() {
    setupLeds();
  }

  private void setupLeds() {
    twinkleblueAnimation = new TwinkleAnimation(0, 0, 255);
    twinkleredAnimation = new TwinkleAnimation(255, 0, 0);
    candle1 = new CANdle(48, "canivore");
    strobeAnimation = new StrobeAnimation(0, 255, 0, 255, 98.0 / 256.0, 68, 0);
    strobeintakeAnimation = new StrobeAnimation(255, 255, 255, 255, STROBE_SPEED, 148, 0);
    intaking = false;

  }

  @Override
  public void periodic() {
    if ((LimelightHelpers.getTX("limelight") < 3) &&
        (LimelightHelpers.getTY("limelight") < 3) &&
        (LimelightHelpers.getTX("limelight") > -3) &&
        (LimelightHelpers.getTY("limelight") > -3) &&
        LimelightHelpers.getTV("limelight")) {
      setLedsToStrobe();
    } else if (intaking) {
      setLedsIntake();
    } else {
      setLedsUsingAllianceColor();
    }
  }

  public void setLedsToStrobe() {
    candle1.animate(strobeAnimation);
  }

  public void setLedsIntake() {
    candle1.animate(strobeintakeAnimation);
  }

  public void setLedsUsingAllianceColor() {
    alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        candle1.animate(twinkleredAnimation);
      }
      if (alliance.get() == Alliance.Blue) {
        candle1.animate(twinkleblueAnimation);
      }
    }
  }

  public static void setIntakeStatus(boolean status) {
    intaking = status;
  }
}