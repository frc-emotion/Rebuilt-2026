// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  /** Creates a new LED in terms of CANdle */

  private CANdle ledCANdle;
  private CANdleConfiguration ledSetter;

  private static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
  private static final RGBWColor kBlue = new RGBWColor(Color.kBlue).scaleBrightness(0.5);
  private static final RGBWColor kRed = new RGBWColor(Color.kRed).scaleBrightness(0.5);
  private static final RGBWColor kYellow = new RGBWColor(Color.kYellow).scaleBrightness(0.5);
  private static final RGBWColor kGreen = new RGBWColor(Color.kGreen).scaleBrightness(0.5);

  private enum AnimationType {
    None,
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
  }

  //initialization
  
  public LED(CANBus mechanisms) {
    ledCANdle = new CANdle(0, mechanisms);
    ledSetter = new CANdleConfiguration();

    ledSetter.LED.StripType = StripTypeValue.GRB;
    ledSetter.LED.BrightnessScalar = 1.0;
    ledCANdle.getConfigurator().apply(ledSetter);
    ledCANdle.setControl(new SolidColor(kSlot0StartIndex, kSlot0EndIndex).withColor(kWhite));
  }
  
//things we want the leds to do
  public void disableStatusLights() {
    ledSetter.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
  }
  public void enableStatusLights() {
    ledSetter.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;
  }

//get rid of everything
  public void clearEverything(){
    for (int i = kSlot0StartIndex; i < kSlot0EndIndex; ++i) {
      ledCANdle.setControl(new EmptyAnimation(i));
    }
    ledCANdle.setControl(new SolidColor(8, 40).withColor(kWhite));
  }
// all da colors
  public void setYellow(){
    ledCANdle.setControl(new SolidColor(8, 40).withColor(kYellow));
  }

  public void setRed(){
    ledCANdle.setControl(new SolidColor(8, 40).withColor(kRed));
  }

  public void setBlue(){
    ledCANdle.setControl(new SolidColor(8, 40).withColor(kYellow));
  }

    public void setWhite(){
    ledCANdle.setControl(new SolidColor(8, 40).withColor(kWhite));
  }

    public void setGreen(){
    ledCANdle.setControl(new SolidColor(8, 40).withColor(kGreen));
  }


//   public void setRed() {
//     // int r = 255;
//     // int g = 0;
//     // int b = 0;

//     LEDPattern red = LEDPattern.solid(Color.kRed);

//     red.applyTo(ledBuffer);
//     led.setData(ledBuffer);

//   }

//   public void setBlue() {
//     LEDPattern blue = LEDPattern.solid(Color.kBlue);

//     blue.applyTo(ledBuffer);
//     led.setData(ledBuffer);

//   }
// //team color 
//   public void setYellow() {
//     LEDPattern yellow = LEDPattern.solid(Color.kYellow);

//     yellow.applyTo(ledBuffer);
//     led.setData(ledBuffer);

//   }

// //aura farm
//   public void setRainbow() {
//     LEDPattern rainbow = LEDPattern.rainbow(255, 128);

//     rainbow.applyTo(ledBuffer);
//     led.setData(ledBuffer);

//   }
// //set alliance
//   public void setLedAlliance() {

//     var alliance = DriverStation.getAlliance();
        
//     if (alliance.isPresent()) { 
//       if (alliance.get() == DriverStation.Alliance.Red) {
//         setRed();
//       } else if (alliance.get() == DriverStation.Alliance.Blue) {
//         setBlue();
//       }
//     } else {
//       setRainbow();
//     }
//   }

}
