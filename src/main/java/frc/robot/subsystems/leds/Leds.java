// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limelight.LimelightHelpers;

public class Leds extends SubsystemBase {

  Optional<Alliance> alliance;
  private TwinkleAnimation twinkleblueAnimation;
  private TwinkleAnimation twinkleredAnimation;
  //private StrobeAnimation pickupAnimation;
  private StrobeAnimation lockedonAnimation;

  private final int TOTAL = 148;
  private final int GUITAR = 80;
  private final int THUNDERSTRUCK = 68;
  private final double STROBE_SPEED = 98.0 / 256.0;

  private CANdle candle1;

  /** Creates a new CANdle */
  public Leds() {
    setupLeds();
  }

  private void setupLeds() {
    twinkleblueAnimation = new TwinkleAnimation(0, 0, 255);
    //twinkleredAnimation = new TwinkleAnimation(255, 0, 0);
    twinkleredAnimation = new TwinkleAnimation(255, 0, 0, 0, 0, TOTAL, TwinklePercent.Percent88, GUITAR);
    candle1 = new CANdle(48, "canivore");
    lockedonAnimation = new StrobeAnimation(0, 255, 0, 0, STROBE_SPEED, THUNDERSTRUCK);
    //pickupAnimation = new StrobeAnimation(0, 0, 0, 255, STROBE_SPEED, TOTAL);

    setLedsUsingAllianceColor();

  }

  @Override
  public void periodic() {
    if ((LimelightHelpers.getTX("limelight") < 3) &&
        (LimelightHelpers.getTY("limelight") < 3) &&
        (LimelightHelpers.getTX("limelight") > -3) &&
        (LimelightHelpers.getTY("limelight") > -3) &&
        LimelightHelpers.getTV("limelight"))  {

      setLedsToStrobe();
    } else {
      setLedsUsingAllianceColor();
    }

    // This method will be called once per scheduler run
  }

  public void setLedsToStrobe() {
    candle1.animate(lockedonAnimation);
  }

  public void setLedsUsingAllianceColor() {
    alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        candle1.animate(twinkleredAnimation);

      }
      if (alliance.get() == Alliance.Blue) {
        // Put what you want it to do here.
        candle1.animate(twinkleblueAnimation);
      }
    }
  }
}