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
  int color[] = new int[180];
  int brightness = 25;
  int x = 5;
  int x2 = 44;
  int y = 0;
  int y2 = 0;
  int a = 15;
  int time = 0;
  int timespeed = 3;   //bigger value makes stuff slower
  int raindrop[] = new int[180];
  boolean mode1 = false;
  boolean mode2 = false;
  boolean mode3 = false;
  boolean gamemode = false;
  Random random = new Random();

  int p1 = 0;
  int p1score = 0;
  int p2 = 0;
  int p2score = 26;
  public LEDs() {
    super(); // Super call adds it to the registry, which calls the periodic method every tick
    leds = new AddressableLED(5);
    ledBuff = new AddressableLEDBuffer(100); // something around 280 length for LED
    leds.setLength(ledBuff.getLength());
    leds.setData(ledBuff);
    leds.start();
    // resetLeds();

    for(var i = 0; i != ledBuff.getLength(); i++)
    {
      raindrop[i] = random.nextInt(255);
      //color[i] = random.nextInt(15);
      color[i] = 2;
    }

  }

  @Override
  public void periodic() {
  

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
      x2 = x2 + y2 % ledBuff.getLength();
    }
    if (x == 44) {
     y = -1;
      y2 = 1;
    }

    if (x == 5) {
      y = 1;
      y2 = -1;
    }


    

    if(mode1 == true)   //MODE 1
    {  for (var i = 0; i < ledBuff.getLength(); i++) { 
      ledBuff.setHSV(i, colour, 255, brightness);

    }
    colour = (colour + 1) % 180;
  }




    if(mode2 == true)  //MODE 2
    {
     ledBuff.setHSV((x - (y * 5)), 7, 255, 0);

     ledBuff.setHSV(x, colour, 255, brightness);

      ledBuff.setHSV((x2 - (y2 * 5)), 7, 255, 0);

     ledBuff.setHSV(x2, colour + 50, 255, brightness);
    
     colour = (colour + 1) % 180;
  }





  if(mode3 == true)    //MODE 3 
  {
   
    if(time == 1)
    {
    for(var i = 0; i < 100; i++)
    {
    ledBuff.setHSV(i, color[i], 255, raindrop[i]);
    raindrop[i] -=4;
    if (raindrop[i] < 1)
    {
      color[i] +=10;
      raindrop[i] = 80;
    }
    if (color[i] > 240)
    {
      color[i] = 10;
    }
  }
}
  }





  if(gamemode == true) //Gamemode
  {
  
    ledBuff.setHSV(13, 1, 255, 150);
    ledBuff.setHSV(14, 1, 255, 150);
  
  
  p1score = (p1 / 10);
  p2score = p2 / 10;
  
  if((p1score != 13) && (p2score != 14))
  {
  ledBuff.setHSV(p1score, 175, 255, 150);
  ledBuff.setHSV(p2score, 10, 255, 150);
  }
  }

  if((p1score > 12) && (p1score != ledBuff.getLength()))
  {
   p1score++;
  }

    leds.setData(ledBuff);
  }

  public void mode1() { //Button B
    mode1 = true;
    mode2 = false;
    mode3 = false;
    gamemode = false;
  }
  public void mode2() { //Button A
    mode2 = true;
    mode1 = false;
    mode3 = false;
    gamemode = false;
  }

public void mode3() { //Button X
mode3 = true;
mode1 = false;
mode2 = false;
gamemode = false;
}

public void gamemode() { //Button Start button

  for(int i = 0; i != ledBuff.getLength(); i++)
  {
    ledBuff.setHSV(i, 1, 255, 0);
  }
 p1score = 0;
 p2score = 26;
mode1 = false;
mode2 = false;
mode3 = false;
gamemode = true;

}

public void y2(){
p2++;
}

public void y1(){
p1++;
}

  public void intakelight() {
    mode1 = false;
    mode2 = false; 
    mode3 = false;
    gamemode = false;
    colour =  50;
   for(int i = 0; i != ledBuff.getLength(); i++)
   {
     ledBuff.setHSV(i, colour, 255, 255);
   }
  }

  public void setColor(int colour) {
    mode1 = false;
    mode2 = false; 
    mode3 = false;
   for(int i = 0; i != ledBuff.getLength(); i++)
   {
     ledBuff.setHSV(i, colour, 255, 255);
   }
  }

  

  public void shooterlight() {
    colour = 60;
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
    timespeed-=2;
  }
  public void decreaseSpeed() {
timespeed+=2;

  }

}
