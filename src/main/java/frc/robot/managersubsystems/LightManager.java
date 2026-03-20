package frc.robot.managersubsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.enums.Modes.FlywheelMode;
import frc.robot.enums.Modes.HoodMode;
import frc.robot.enums.Modes.IntakeSpinMode;
import frc.robot.enums.Modes.TurretMode;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.VirtualSubsystem;

public class LightManager extends VirtualSubsystem {

  private static LightManager instance;
  private final String key;
  private final CANdle candle;

  private final int numLights = 50;

  private final RGBWColor orangeColor = new RGBWColor(243, 118, 33);
  private final RGBWColor blueColor = new RGBWColor(255, 0, 0);
  private final RGBWColor redColor = new RGBWColor(0, 0, 255);
  private final RGBWColor greenColor = new RGBWColor(0, 255, 0);
  private final RGBWColor yellowColor = new RGBWColor(255, 245, 0);

  private final RGBWColor pinkColor = new RGBWColor(255, 192, 203);
  private final RGBWColor purpleColor = new RGBWColor(215, 0, 255);

  private LightManager(String key) {
    this.key = key;

    this.candle = new CANdle(1);
  }

  public static LightManager instance() {
    if (instance == null) {
      instance = new LightManager("lights");
    }
    return instance;
  }

  @Override
  public void periodic() {

    if (DriverStation.isFMSAttached()) {
      if (DriverStation.isEnabled()) {
        enabledChecks();
        return;
      } else {
        if (RobotContainer.isRed()) {
          patternSetIdleRed();
          return;
        } else {
          patternSetIdleBlue();
          return;
        }
      }
    } else {
      patternSetIdleOrange();
      return;
    }
  }

  private void enabledChecks() {
    // Highest priority is hood checks
    if (RobotState.instance().getHoodMode() == HoodMode.DOWN) {
      candle.setControl(new LarsonAnimation(0, numLights).withColor(pinkColor).withFrameRate(750)
          .withBounceMode(LarsonBounceValue.Front));
      return;
    }

    // Then verify quest connected
    if(!RobotState.instance().getQuestConnected()) {
      candle.setControl(new TwinkleAnimation(0, numLights).withColor(purpleColor).withFrameRate(400));
      return;
    }

    // Are limelights disconnected
    // TODO do later

    // Weird logger error
    if(Logger.getReceiverQueueFault()) {
      candle.setControl(new SolidColor(0, numLights).withColor(redColor));
      return;
    }

    // Are we in tracking mode
    if(RobotState.instance().getTurretMode() != TurretMode.TRACKING_HUB) {
      candle.setControl(new SolidColor(0, numLights).withColor(yellowColor));
      return;
    }

    // Shooting feedback
    if(RobotState.instance().getFlywheelMode() == FlywheelMode.SHOOTING || RobotState.instance().getFlywheelMode() == FlywheelMode.FRENZY) {
      candle.setControl(new StrobeAnimation(0, numLights).withColor(ShotCalculator.instance().getReadyToFeed() ? greenColor : redColor).withFrameRate(500));
      return;
    }

    // Are we intaking
    if(RobotState.instance().getIntakeSpinMode() == IntakeSpinMode.INTAKING) {
      candle.setControl(new ColorFlowAnimation(0, numLights).withColor(blueColor).withFrameRate(300));
      return;
    }

    // Nothing requires attention
    candle.setControl(new EmptyControl());
  }

  private void patternSetIdleOrange() {
    candle.setControl(new LarsonAnimation(0, numLights).withColor(orangeColor).withFrameRate(250)
        .withBounceMode(LarsonBounceValue.Front).withSize(10));
  }

  private void patternSetIdleBlue() {
    candle.setControl(new LarsonAnimation(0, numLights).withColor(blueColor).withUpdateFreqHz(750)
        .withBounceMode(LarsonBounceValue.Front).withSize(10));
  }

  private void patternSetIdleRed() {
    candle.setControl(new LarsonAnimation(0, numLights).withColor(redColor).withUpdateFreqHz(750)
        .withBounceMode(LarsonBounceValue.Front).withSize(10));
  }
}
