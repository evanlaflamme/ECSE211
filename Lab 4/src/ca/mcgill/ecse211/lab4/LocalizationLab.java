// Lab2.java

package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;

public class LocalizationLab {

  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  private static final Port usPort = LocalEV3.get().getPort("S4");

  private static Odometer odometer = null;

  public static final double WHEEL_RADIUS = 2.07;
  public static final double TRACK = 15.1;
  public static final double SQUARE_LENGTH = 30.48;

  public static void main(String[] args) {
    int buttonChoice;
    
    final TextLCD t = LocalEV3.get().getTextLCD();
    odometer = new Odometer(leftMotor, rightMotor);
    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);
        
    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(usPort);
    SampleProvider usDistance = usSensor.getMode("Distance");
    SampleProvider usFiltered = new MedianFilter(usDistance, 5);
    float[] usData = new float[usDistance.sampleSize()];
    UltrasonicPoller usPoller = null;
    
    UltrasonicLocalizer usLoc = null;

    do {
      // clear the display
      t.clear();

      // ask the user whether the motors should drive in a square or float
      t.drawString("< Left | Right >", 0, 0);
      t.drawString("       |        ", 0, 1);
      t.drawString("Rising | Falling", 0, 2);
      t.drawString(" edge  |  edge  ", 0, 3);
      t.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    odometer.start();
    odometryDisplay.start();
    
    Navigation navigation = new Navigation(odometer, leftMotor, rightMotor);
    
    if (buttonChoice == Button.ID_LEFT) { //Rising edge
      usLoc = new UltrasonicLocalizer(UltrasonicLocalizer.Method.RISING_EDGE, navigation, odometer);
    } else { //Falling edge
      usLoc = new UltrasonicLocalizer(UltrasonicLocalizer.Method.FALLING_EDGE, navigation, odometer);
    }
    
    usPoller = new UltrasonicPoller(usFiltered, usData, usLoc);

    usPoller.start();
    
    try { // Wait 5 seconds for ultrasonic sensor to activate
      Thread.sleep(5000);
    } catch (Exception e) {
    }
    
    usLoc.start();
    
/*    try{
    	usLoc.join();
    }catch (InterruptedException e){
    }
    */
    Button.waitForAnyPress();
    
    LightLocalizer lightlocalizer = new LightLocalizer(odometer, navigation);
    lightlocalizer.start();
    
/*    try{
    	lightlocalizer.join();
    }catch (InterruptedException e){
    }*/
    
    Button.waitForAnyPress();
    System.exit(0);
  }
}
