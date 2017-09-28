// Lab2.java

package ca.mcgill.ecse211.lab3;

import java.util.ArrayList;
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
  
  private static ArrayList<int[]> points = new ArrayList<int[]>();
  private static int pointsIndex = 0;

  public static final double WHEEL_RADIUS = 2.2;
  public static final double TRACK = 15.9;
  public static final double SQUARE_LENGTH = 30.48; 

  public static void main(String[] args) {
    //Map 1
    points.add(new int[] {0, 2});
    points.add(new int[] {1, 1});
    points.add(new int[] {2, 2});
    points.add(new int[] {2, 1});
    points.add(new int[] {1, 0});
    
    //Map 2
    /*points.add(new int[] {1, 1});
    points.add(new int[] {0, 2});
    points.add(new int[] {2, 2});
    points.add(new int[] {2, 1});
    points.add(new int[] {1, 0});*/
    
    //Map 3
    /*points.add(new int[] {1, 0});
    points.add(new int[] {2, 1});
    points.add(new int[] {2, 2});
    points.add(new int[] {0, 2});
    points.add(new int[] {1, 1});*/
    
    //Map 4
    /*points.add(new int[] {0, 1});
    points.add(new int[] {1, 2});
    points.add(new int[] {1, 0});
    points.add(new int[] {2, 1});
    points.add(new int[] {2, 2});*/
    
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
    
    // clear the display
    t.clear();
    
    t.drawString("Press any button.", 0, 0);
    Button.waitForAnyPress();
    
    odometer.start();
    odometryDisplay.start();

    odometryCorrection.start();      
    
    usPoller.start();
    
    Thread navigationThread = null;
  
    while (pointsIndex < points.size()) {      
      navigationThread = (new Thread() {
        public void run() {
          Navigation n = new Navigation(odometer, leftMotor, rightMotor);
          n.travelTo(points.get(pointsIndex)[0], points.get(pointsIndex)[1]);
        }
      });
      navigationThread.start();
      
      /*n.travelTo(points.get(pointsIndex)[0], points.get(pointsIndex)[1]);
      pointsIndex++;*/
      
      while (navigationThread.getState() != Thread.State.TERMINATED) {  //TODO: FIX THIS
        //Check for objects...
        int one = 0;
        
        if (one == 1) { //Object detection
          leftMotor.stop(); //Set speed to zero to stop motors
          rightMotor.stop();
          navigationThread.interrupt();
          System.out.println("Stopping drive thread");
          
          pointsIndex--;
        }
      }
      System.out.println("At point");
    }
    System.exit(0);
  }
}
