package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.utils.PeriodicRunnable;
import java.util.Random;

public class LEDs extends PeriodicRunnable {
  AddressableLED leds;
  AddressableLEDBuffer ledBuff;
  int counter = 0;
  int colour = 0;
  int brightness = 25;
  int x = 5;
  int x2 = 44;
  int y = 0;
  int y2 = 0;
  int a = 15;
  int time = 0;
  int timespeed = 3; // bigger value makes stuff slower
  int raindrop[] = new int[180];
  boolean mode1 = false;
  boolean mode2 = false;
  boolean mode3 = true;
  Random random = new Random();
  private static LEDs globalInst;

  public static LEDs getInstance() {
    if (globalInst == null) {
      globalInst = new LEDs();
    }
    return globalInst;
  }

  private LEDs() {
    super(); // Super call adds it to the registry, which calls the periodic method every tick
    // leds = new AddressableLED(5);
    // ledBuff = new AddressableLEDBuffer(100); // something around 280 length for LED
    // leds.setLength(ledBuff.getLength());
    // leds.setData(ledBuff);
    // leds.start();
    // // resetLeds();

    // for (var i = 0; i != ledBuff.getLength(); i++) {
    //   raindrop[i] = random.nextInt(255);
    // }
  }

  @Override
  public void periodic() {

    // time++;
    // time = time % timespeed;

    // brightness = (a - 15) * (a - 15);

    // if (time == 1) {
    //   a++;
    //   a = a % 30;
    // }

    // if (time == 1) {
    //   x = x + y % ledBuff.getLength();
    //   x2 = x2 + y2 % ledBuff.getLength();
    // }
    // if (x == 44) {
    //   y = -1;
    //   y2 = 1;
    // }

    // if (x == 5) {
    //   y = 1;
    //   y2 = -1;
    // }

    // if (mode1) // MODE 1
    // {
    //   for (var i = 0; i < ledBuff.getLength(); i++) {
    //     ledBuff.setHSV(i, colour, 255, brightness);
    //   }
    //   colour = (colour + 1) % 180;
    // }

    // if (mode2) // MODE 2
    // {
    //   ledBuff.setHSV((x - (y * 5)), 7, 255, 0);

    //   ledBuff.setHSV(x, colour, 255, brightness);

    //   ledBuff.setHSV((x2 - (y2 * 5)), 7, 255, 0);

    //   ledBuff.setHSV(x2, colour + 50, 255, brightness);

    //   colour = (colour + 1) % 180;
    // }

    // if (mode3) // MODE 3
    // {

    //   if (time == 1) {
    //     for (var i = 0; i < 100; i++) {
    //       ledBuff.setHSV(i, colour, 255, raindrop[i]);
    //       raindrop[i]--;
    //       // brightness = raindrop[i];
    //       /*  if(time == 0)
    //       {
    //         colour = random.nextInt(180);
    //       } */

    //     }
    //   }
    // }

    // leds.setData(ledBuff);
  }

  public void mode1() {
    mode1 = true;
    mode2 = false;
    mode3 = false;
  }

  public void mode2() {
    mode2 = true;
    mode1 = false;
    mode3 = false;
  }

  public void mode3() {
    mode3 = true;
    mode1 = false;
    mode2 = false;
  }

  public void intakelight() {
    colour = 31;
  }

  public void errorLight() {
    colour = 1;
  }

  public void shooterlight() {
    colour = 60;
  }

  public void separateClimberLight() {
    colour = 120;
  }

  /* private void resetLeds() {
    for(int i = 0; i < ledBuff.getLength(); i++){
      brightness = 0;
    }
  } */

  // public void getControllerInput(double left, double right) {
  //   timespeed+= left;
  //   if(timespeed != 0)
  //   {
  //   timespeed-= right;
  // }
  // }

  public void increaseSpeed() {
    timespeed -= 2;
  }

  public void decreaseSpeed() {
    timespeed += 2;
  }
}
