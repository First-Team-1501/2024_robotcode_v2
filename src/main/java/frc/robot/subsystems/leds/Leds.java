// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import java.util.Optional;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.TwinkleAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  private CANdle candle1;
  TwinkleAnimation twinkleblueAnimation = new TwinkleAnimation(0, 0, 255);
  TwinkleAnimation twinkleredAnimation = new TwinkleAnimation(255, 0, 0);
  TwinkleAnimation twinklewhiteAnimation = new TwinkleAnimation(255, 255, 255);
  /** Creates a new Leds. */
  public Leds() {
  }

  @Override
  public void periodic() {

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        // Put what you want it to do here.
        candle1.animate(twinkleredAnimation);
      }
      if (ally.get() == Alliance.Blue) {
        // Put what you want it to do here.
        candle1.animate(twinkleblueAnimation);
      }
      else {
        // Put what you want it to do here.
        candle1.animate(twinklewhiteAnimation);
      }
    }
    // This method will be called once per scheduler run
  }
}
