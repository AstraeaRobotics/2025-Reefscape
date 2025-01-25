// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
  public static class LedConstants{
    public static final int ledLength = 0;
    public static final int kPwmPort = 0;
        public static final int kLength = 144;
        public static final double kInterval = Units.secondsToMilliseconds(0.1);
        public static final double kIntervalTrail = Units.secondsToMilliseconds(0.05);
        public static final int kRainbowSaturation = 255;
        public static final int kRainbowValue = 150;
        public static final int kRainbowIncrement = 150;
        public static final int kMaxHue = 180;
        public enum LEDStates {
          kOff,
          kGlowRed,
          kGlowYellow,
          kGlowPurple,
          kRainbow,
          kintaking,
          kPlacing,
          kTrailYellow,
          kTrailPurple,
          kFlashYellow,
          kFlashPurple,
          kFlashGreen,
          kBigInterval,
        }
  }

}
