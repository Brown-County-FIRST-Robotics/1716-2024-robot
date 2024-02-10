package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.utils.PeriodicRunnable;

public class LEDs extends PeriodicRunnable {
  AddressableLED leds;
  AddressableLEDBuffer ledBuff;
  int counter = 0;

  public LEDs() {
    super(); // Super call adds it to the registry, which calls the periodic method every tick
    leds = new AddressableLED(5);
    ledBuff = new AddressableLEDBuffer(10);
    leds.setLength(ledBuff.getLength());
    leds.setData(ledBuff);
    leds.start();
  }

  @Override
  public void periodic() {

    SetColor(counter, 255, 5);
    counter += 5;
  }

  private void SetColor(int hue, int saturation, int value) {
    for (var i = 0; i < ledBuff.getLength(); i++) {
      ledBuff.setHSV(i, (hue - (20 * i)) % 180, saturation, value);
    }
    leds.setData(ledBuff);
  }
}
