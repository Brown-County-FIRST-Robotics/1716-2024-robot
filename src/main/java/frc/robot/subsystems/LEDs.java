package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.utils.PeriodicRunnable;

public class LEDs extends PeriodicRunnable {
  AddressableLED leds;
  AddressableLEDBuffer ledBuff;
  int counter = 0;
  int colour = 0;
  int brightness = 25;
  int x = 5;
  int x2 = 45;
  int y = 0;
  int a = 15;
  int time = 0;
  int timespeed = 2;
  boolean mode1 = false;
  boolean mode2 = false;

  public LEDs() {
    super(); // Super call adds it to the registry, which calls the periodic method every tick
    leds = new AddressableLED(5);
    ledBuff = new AddressableLEDBuffer(50); // something around 280 length for LED
    leds.setLength(ledBuff.getLength());
    leds.setData(ledBuff);
    leds.start();
    // resetLeds();
  }

  @Override
  public void periodic() {
    timespeed = 3; //bigger value makes stuff slower

    time++;
    time = time % timespeed;

    brightness = (a - 15) * (a - 15);

    if(time == 1)
    {
    a++;
    a = a % 30;
    }

    if (time == 1) {
      x = x + y % ledBuff.getLength();
      x2 = x + -y % ledBuff.getLength();
    }
    if (x == 44) {
      y = -1;
    }

    if (x == 5) {
      y = 1;
    }

    if(mode1 == true)
    {  for (var i = 0; i < 50; i++) { //mode 1
      ledBuff.setHSV(i, colour, 255, brightness);

    }
    colour = (colour + 1) % 180;

  }



    if(mode2 == true)
    {
     ledBuff.setHSV((x - (y * 5)), 7, 255, 0);

     ledBuff.setHSV(x, colour, 255, brightness);

   // ledBuff.setHSV((50 + -x) + (-y * 5), colour, 255, 255);
    
  //  ledBuff.setHSV(50 + -x, colour, 255, 0);

      ledBuff.setHSV((x2 - (-y * 5)), 7, 255, 0);

     ledBuff.setHSV(x2, colour, 255, brightness);
    

    colour = (colour + 1) % 180;
  }

    leds.setData(ledBuff);
  }

  public void mode1() {
    mode1 = true;
    mode2 = false;
  }
  public void mode2() {
    mode2 = true;
    mode1 = false;
  }

  public void intakelight() {
    colour = 1;
  }

  public void shooterlight() {
    colour = 60;
  }

  /* private void resetLeds() {
    for(int i = 0; i < ledBuff.getLength(); i++){
      brightness = 0;
    }
  } */

}
