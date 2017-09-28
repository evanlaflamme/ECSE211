// Lab2.java

package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;

public class NavigationLab {

  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  
  private static final Port usPort = LocalEV3.get().getPort("S4");
  
  private static Odometer odometer = null;

  public static final double WHEEL_RADIUS = 2.2;
  public static final double TRACK = 15.9;
  public static final double SQUARE_LENGTH = 30.48;

  public static void main(String[] args) {
    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(usPort);
    SampleProvider usDistance = usSensor.getMode("Distance");
    SampleProvider usFiltered = new MedianFilter(usDistance, 20);
    float[] usData = new float[usFiltered.sampleSize()];
    UltrasonicPoller usPoller = new UltrasonicPoller(usFiltered, usData);

    final TextLCD t = LocalEV3.get().getTextLCD();
    odometer = new Odometer(leftMotor, rightMotor);
    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);
    OdometryCorrection odometryCorrection = new OdometryCorrection(odometer);

    do {
      // clear the display
      t.clear();
      
      t.drawString("Press any button to start.", 0, 0);
      Button.waitForAnyPress();
      
      odometer.start();
      odometryDisplay.start();
  
      odometryCorrection.start();      
      
      usPoller.start();
      
      // spawn a new Thread to avoid SquareDriver.drive() from blocking
      Thread navigationThread = (new Thread() {
        public void run() {
          Navigation n = new Navigation(odometer, leftMotor, rightMotor);
          n.travelTo(2, 2);
        }
      });
      navigationThread.start();
      
      try {
        Thread.sleep(10000);
      } catch (InterruptedException e) {
         //Do nothing
      }
      
      navigationThread.interrupt();
      System.out.println("Stopping drive thread");
      
      
    } while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
