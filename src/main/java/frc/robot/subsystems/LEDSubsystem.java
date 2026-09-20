// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

  private static LEDSubsystem instance;

  public static LEDSubsystem getInstance(){
    if(instance == null){
      instance = new LEDSubsystem();
    }
    return instance;
  }

  //LED IO
  private AddressableLED leds = new AddressableLED(LEDConstants.LEDPort);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.length);
  private Alliance alliance;

  //Robot state tracking
  public boolean autoFinished = false;
  public boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private double autoFinishedTime = 0.0;
  private double autoTimerError = 0.0;
  public boolean teleopStatue = false;


  public LEDSubsystem() {
    try{
      alliance = DriverStation.getAlliance().get();
    } catch(Exception e){
      alliance = DriverStation.Alliance.Red;
    }
    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    leds.setData(buffer);
  }



  public enum LEDState{
    INITIAL,
    ENABLE,
    DRIVING,
    LOADED,
    UNLOADED,
    AUTO,
    Climb,
    TO_AMP
  }

  public LEDState ledState = LEDState.INITIAL;

  public void update(){
    update(this.ledState);
    // SmartDashboard.putString("LEDStatue", ledState.toString());
  }

  public void update(LEDState state){
    this.ledState = state;
    switch (state){
      case INITIAL:
      if(alliance==Alliance.Blue){
        solidColor(Color.kBlue);
      }else{
        solidColor(Color.kRed);
      }
      break;
    case LOADED:
      solidColor(Color.kGreen);
      break;
    case UNLOADED:
      if(alliance==Alliance.Blue){
        solidColor(Color.kBlue);
      }else{
        solidColor(Color.kRed);
      }
      break;
    case AUTO:
      breath(Color.kAzure, Color.kBlack, Timer.getFPGATimestamp(), 0.3);
      break;
    case TO_AMP:
      solidColor(Color.kPurple);
      break;
    case Climb:
      rainbow(5, 0.4);
      break;
    default:
      wave(Color.kYellow, Color.kRed,LEDConstants.DefaultwaveLength, LEDConstants.DefaultwaveDuration);
      break;
    }


  }

 
  public void stop(){
    if(alliance==Alliance.Blue){
      breath(Color.kBlue, Color.kBlack, Timer.getFPGATimestamp(), LEDConstants.EnablebreathDuration);
    }else{
      breath(Color.kRed, Color.kBlack, Timer.getFPGATimestamp(), LEDConstants.EnablebreathDuration);
    };
  }

  private void solidColor(Color color) {
    if (color != null) {
      for (int i = LEDConstants.initializelength; i < buffer.getLength(); i++) {
        buffer.setLED(i, color);
      }
    }
  }

  private void solidTwoColor(double percent, Color color) {
    // for (int i = 0; i < MathUtil.clamp(LEDConstants.length * percent, 0, LEDConstants.length); i++) {
    //   buffer.setLED(i, color);
    // }
    for (int i = 0; i < MathUtil.clamp(LEDConstants.length, 0, LEDConstants.length); i++) {
      if(i>LEDConstants.length * percent){
        buffer.setLED(i, Color.kBlack);
      }else{
        buffer.setLED(i, color);
      }
    }
  }

  private void strobe(Color c1, Color c2,double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solidColor(on ? c1 : c2);
  }

  private void rainbow(double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < buffer.getLength(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i <= buffer.getLength()) {
        buffer.setHSV(i, (int) x, 255, 255);
      }
    }
  }

  private void breath(Color c1, Color c2, double timestamp, double duration) {
    double x = ((timestamp % LEDConstants.breathDuration) / LEDConstants.breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0)/2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solidColor(new Color(red, green, blue));
  }

  private void wave( Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < buffer.getLength(); i++) {
      x += xDiffPerLed;
      if (i >= 0) {
        double ratio = (Math.pow(Math.sin(x), LEDConstants.waveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), LEDConstants.waveExponent) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  private void stripes(List<Color> colors, int length, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = 0; i < buffer.getLength(); i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }


}
