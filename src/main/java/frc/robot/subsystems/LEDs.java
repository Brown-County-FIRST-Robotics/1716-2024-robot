package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.utils.PeriodicRunnable;

public class LEDs extends PeriodicRunnable {
  AddressableLED leds;
  AddressableLEDBuffer ledBuff;
  int counter = 0;
  int brightness = 25;
  int x = 5;

  public LEDs() {
    super(); // Super call adds it to the registry, which calls the periodic method every tick
    leds = new AddressableLED(5);
    ledBuff = new AddressableLEDBuffer(50); //something around 280 length for LED
    leds.setLength(ledBuff.getLength());
    leds.setData(ledBuff);
    leds.start();
    resetLeds();
  }

  @Override
  public void periodic() { 
    brightness = 25;
   x = x + 1 % ledBuff.getLength();
    
   /*  for (var i = 0; i < 50; i++) { //mode 1
      ledBuff.setHSV(i, (counter + i * 2) % 180, 255, brightness);

    }
    counter += 2; */

    ledBuff.setHSV((x - 5), 7 % 180, 255, 0);
  
  ledBuff.setHSV(x, 7 % 180, 255, brightness);
   

    counter += 2;

    leds.setData(ledBuff);
  }

  private void resetLeds() {
    for(int i = 0; i < ledBuff.getLength(); i++){
brightness = 0;
    }
    }

  }

