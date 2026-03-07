package frc.robot.managersubsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.VirtualSubsystem;

public class LightManager extends VirtualSubsystem {

  private static LightManager instance;
  private final String key;

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private Timer timer;

  // First 23: intake

  private LightManager(String key) {
    this.key = key;

    this.led = new AddressableLED(0);
    this.buffer = new AddressableLEDBuffer(119);
    this.led.setLength(buffer.getLength());

    // Set the data
    this.led.setData(buffer);
    this.led.start();

    this.timer = new Timer();
  }

  public static LightManager instance() {
    if (instance == null) {
      instance = new LightManager("lights");
    }
    return instance;
  }

  @Override
  public void periodic() {
    patternSetIdleOrange();
  }

  private void patternSetIdleOrange() {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the GRB values for red

      double intensity = (Math.sin((i / 5.0) + (Timer.getTimestamp() / 1.0)));

      if (intensity < 0) {
        intensity = 0;
      }

      buffer.setRGB(i, (int) (80.0 * intensity), (int) (255.0 * intensity), 0);
    }
    led.setData(buffer);
  }
}
