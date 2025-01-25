// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  boolean enabled = false;
  private boolean isOn = true;
  private AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private final AddressableLEDSim ledSim;
  private long previousTimeFlash=0;
  private int rainbowFirstPixelHue = 30;
  private int currentTrailIndex = 0;
  int currentIndex = 0;
  int futureIndex = 0;
  public LEDSubsystem() {
    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
    ledSim = new AddressableLEDSim(led);
    Shuffleboard.getTab("DashBoard").addRaw("Addressable Led", this::getLedData).withWidget("Addressable LED");
    setState(LEDStates.kTrailPurple);
  }
  public void setState(LEDStates state) {
  }
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
public void setOutPut(LEDStates state){
  switch (state) {
    case kGlowRed:
    glow(Color.kRed);
    break;
    case kPlacing:
    break;
    case kRainbow:
    rainbow();
    break;
    case kTrailPurple:
    trail(Color.kRed, Color.kYellow, 20, 6);
    break;
    case kTrailYellow:
    trail(Color.kYellow, Color.kPurple, 20, 6);
    break;
    case kGlowYellow:
    bigTrail(Color.kYellow, 10, 4);
    break;
    case kGlowPurple:
    bigTrail(Color.kPurple, 10, 4);
    break;
    case kFlashYellow:
    flash(Color.kYellow);
    break;
    case kFlashPurple:
    flash(Color.kPurple);
    break;
    case kFlashGreen:
    flash(Color.kGreen);
    break;
    case kBigInterval:
    bigTrail(Color.kRed, 10, 4);
    break;
    default:
    break;
  }
  led.setData(ledBuffer);
}
private byte[] getLedData(){
  return ledSim.getData();
}
public void glowRGB(int r, int g, int b){
  for (int v=0; v < ledBuffer.getLength(); v++){
    ledBuffer.setRGB(v, r, g, b);
  }

}
public void setLEDRGB(int i, int r, int g, int b, double brightness){
  ledBuffer.setRGB(i, (int)brightness*r, (int)brightness*g, (int)brightness*b);

}
public void setLED(int i, Color color, double brightness){
  setLEDRGB(i, (int)(color.red*255), (int)(color.green*255), (int)(color.blue*255), brightness);
}
public void glow(Color color){
  glowRGB((int)(color.red*64), (int)color.blue*64, (int)color.green*64);
}
public void rainbow() {
for (int i=0; i < ledBuffer.getLength(); i++){
  int hue = 
  (rainbowFirstPixelHue + (i*180 / ledBuffer.getLength())) % 180;
  ledBuffer.setHSV(i, hue, 255, 128);}
  rainbowFirstPixelHue += 2;
  rainbowFirstPixelHue %= 180;
}

public void flashRGB(int r, int g, int b) {
long currentTime= System.currentTimeMillis();
if ((currentTime - previousTimeFlash) >= LedConstants.kInterval){
  previousTimeFlash=currentTime;
  isOn = !isOn;
}
if (!isOn){
  r=0;
  g=0;
  b=0;
}
for (int i = 0; i < ledBuffer.getLength(); i++){
  ledBuffer.setRGB(i, r, g, b);
}
}
public void flash(Color color){
  flashRGB((int)color.red*64, (int)color.green*64, (int)color.blue*64);
}
public void trail(Color bgColor, Color movingColor, int trailLength, int speed) {
  glow(bgColor);
  for (int i = currentTrailIndex; i < currentTrailIndex + trailLength; i++) {
    ledBuffer.setLED(i % ledBuffer.getLength(), movingColor);
  }
  currentTrailIndex += speed;
}
public void fadeRed(){
  double t = Timer.getFPGATimestamp();
  for (int i = 0; i < ledBuffer.getLength(); i++) {
    ledBuffer.setRGB(i, (int) (255*(Math.cos(1.5*t)*0.5+0.5)), 0, 0);
  }
}
public void fadeBlue(){
  double t = Timer.getFPGATimestamp();
  for (int i = 0; i < ledBuffer.getLength(); i++) {
    ledBuffer.setRGB(i, 0, 0, (int)(255*(Math.cos(1.5*t)*0.5+0.5)));
  }
}
public void bigTrail(Color color, double interval, int length) {
  long currentTime = System.currentTimeMillis();

  if (currentTime - previousTimeFlash >= interval) {
    previousTimeFlash = currentTime;
    
    for (int i = 0; i < LedConstants.kLength; i++) {
      int targetLED = (i + currentIndex) % LedConstants.kLength;
      setLED(targetLED, color, Math.max((double)i / (LedConstants.kLength * 4) - 0.01, 0.01));
    }
    currentIndex++;
  }
}

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()){
      setOutPut(LEDStates.kBigInterval);
    }
  }
}
