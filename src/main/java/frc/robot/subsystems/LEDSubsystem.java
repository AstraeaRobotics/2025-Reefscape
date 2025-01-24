// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import java.awt.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  boolean enabled = false;
  private boolean isOn = true;
  private AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private final AddressableLEDSim m_ledSim;
  private long m_previousTimeFlash=0;
  private int m_rainbowFirstPixelHue = 30;
  private int currentTrailIndex = 0;
  int currentIndex = 0;
  int futureIndex = 0;
  public LEDSubsystem() {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    m_ledSim = new AddressableLEDSim(m_led);
    Shuffleboard.getTab("DashBoard").addRaw("Addressable Led", this::getLedData).withWidget("Addressable LED");
    setState(LEDStates.kTrailPurple);
  }
  public void setState(LEDStates state) {
  }
public enum LEDStates {
  kOff,
  kGlowRed,
  kGlowYellow,
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
public void setOutPut(LEDStates state){
  switch (state) {
    case kGlowRed:
    glow(Color.kRed);
  }
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
