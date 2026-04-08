// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.util.function.BooleanSupplier;

// import com.ctre.phoenix6.CANBus;
// import com.ctre.phoenix6.configs.CANdleConfiguration;
// import com.ctre.phoenix6.controls.EmptyAnimation;
// import com.ctre.phoenix6.controls.SolidColor;
// import com.ctre.phoenix6.controls.StrobeAnimation;
// import com.ctre.phoenix6.hardware.CANdle;
// import com.ctre.phoenix6.signals.RGBWColor;
// import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
// import com.ctre.phoenix6.signals.StripTypeValue;


// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants.LEDConstants;


// public class LED extends SubsystemBase {
//   /** Creates a new LED in terms of CANdle */
//   public CANdle ledCANdle;
//   private CANdleConfiguration ledSetter;

//   private static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
//   private static final RGBWColor kBlue = new RGBWColor(Color.kBlue).scaleBrightness(0.5);
//   private static final RGBWColor kRed = new RGBWColor(Color.kRed).scaleBrightness(0.5);
//   private static final RGBWColor kYellow = new RGBWColor(Color.kYellow).scaleBrightness(0.5);
//   private static final RGBWColor kGreen = new RGBWColor(Color.kGreen).scaleBrightness(0.5);
//   private static final RGBWColor kOrange = new RGBWColor(Color.kOrange).scaleBrightness(0.5);

// //animation selectors
//   public AnimationType m_animHopperState = AnimationType.None;
//   public AnimationType m_animShooterState = AnimationType.None;

//   private final SendableChooser<AnimationType> m_animHopperChooser = new SendableChooser<AnimationType>();
//   private final SendableChooser<AnimationType> m_animShooterChooser = new SendableChooser<AnimationType>();

//   public final StrobeAnimation hopperIntakeOut;
//   public final SolidColor hopperIntakeRed;
//   public final SolidColor hopperIntakeBlue;
//   public final SolidColor hopperIntakeWhite;
//   public final SolidColor shooterIndexerNotRunning;
//   public final SolidColor shooterIndexerRunning;
//   public BooleanSupplier isIntakeDeployed, isIndexerRunning;

//   public final int[] hopperBounds;
//   public final int[] shooterBounds;

  
//   //the list of animations   
//     private enum AnimationType {
//       None,
//       ColorFlow,
//       Fire,
//       Larson,
//       Rainbow,
//       RgbFade,
//       SingleFade,
//       Strobe,
//       Twinkle,
//       TwinkleOff,
//     }
    
//     //initialization
//   public LED(CANBus mechanisms, BooleanSupplier isIntakeDeployed, BooleanSupplier isIndexerRunning, int[] hopperBounds, int[] shooterBounds) {
//     ledCANdle = new CANdle(0, mechanisms);
//     ledSetter = new CANdleConfiguration();
    
//     ledSetter.LED.StripType = StripTypeValue.GRB;
//     ledSetter.LED.BrightnessScalar = 1.0;
//     ledCANdle.getConfigurator().apply(ledSetter);
//     ledSetter.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
  
//   //add animations to chooser for slot 0 (NOT a big fish to fry yet)
//     m_animHopperChooser.setDefaultOption("Solid Color", AnimationType.None);
//     m_animHopperChooser.addOption("Rainbow", AnimationType.Rainbow);
//     m_animHopperChooser.addOption("Strobe", AnimationType.Strobe);
    
//   /* add animations to chooser for slot 1 */
//     m_animShooterChooser.setDefaultOption("Solid Color", AnimationType.None);
//     m_animShooterChooser.addOption("Rainbow", AnimationType.Rainbow);
//     m_animShooterChooser.addOption("Strobe", AnimationType.Strobe);
  
//     this.isIntakeDeployed = isIntakeDeployed;
//     this.isIndexerRunning = isIndexerRunning;
//     this.hopperBounds = hopperBounds;
//     this.shooterBounds = shooterBounds;

//     shooterIndexerNotRunning = new SolidColor(shooterBounds[49], shooterBounds[77])
//       .withColor(kYellow);

//     shooterIndexerRunning = new SolidColor(shooterBounds[49], shooterBounds[77])
//       .withColor(kGreen);
//     //  .withSlot(0);
  
//     hopperIntakeOut = new StrobeAnimation(hopperBounds[8], hopperBounds[48]) // lower, upper bounds
//       .withColor(kOrange) //color
//       .withFrameRate(5.0) // Number of times to strobe per second
//       .withSlot(0); // Slot allows animations to run at once (range [0,7])

//     hopperIntakeRed = new SolidColor(hopperBounds[8], hopperBounds[48])
//       .withColor(kRed);

//     hopperIntakeBlue = new SolidColor(hopperBounds[8], hopperBounds[48])
//     .withColor(kBlue);

//     hopperIntakeWhite = new SolidColor(hopperBounds[8], hopperBounds[48])
//     .withColor(kWhite);
//   }

//   public void disableStatusLights() {
//     ledSetter.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
//   }
//   public void enableStatusLights() {
//     ledSetter.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;
//   }

// //get rid of everything
//   public void clearEverything(){
//     for (int i = 8; i < 77; ++i) {
//       ledCANdle.setControl(new EmptyAnimation(i));
//     }
//     ledCANdle.setControl(new SolidColor(LEDConstants.kHopperLEDStartIndex, LEDConstants.kShooterLEDEndIndex).withColor(kWhite));
//   }
// }
//   /* what does simeon want
//    * 
//    * HOPPER LEDS (40 LEDS)
//    * for both alliances (field management system)
//    * intake out- orange (let's make it blink using strobe animation)
//    * intake in- alliance colors
//    * DONE!!!!
//    * 
//    * SHOOTER LEDS (28 LEDS)
//    * indexer not running- yellow
//    * indexer running- green
//    * 
//    * Aura farming leds for whatever purposes (a smaller fish to fry)
//    */