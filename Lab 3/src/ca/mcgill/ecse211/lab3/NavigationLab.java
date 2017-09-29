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
  
  private static boolean objectDetected = false;

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
  public static final double SQUARE_LENGTH = 30.7; 
  
  private static final int DISTANCE_THRESHOLD = 15;
  
  private synchronized static void setObjectDetection(boolean b) {
    objectDetected = b;
  }
  
  private synchronized static boolean getObjectDetection() {
    return objectDetected;
  }

  public static void main(String[] args) {    
    //Map 1
    /*points.add(new int[] {0, 2}); (-7, -2.4); (4.4, -2.1)(2.3, 0.1)
    points.add(new int[] {1, 1});
    points.add(new int[] {2, 2});
    points.add(new int[] {2, 1});
    points.add(new int[] {1, 0});*/
    
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
    
    //Test Map
    points.add(new int[] {2, 1});
    points.add(new int[] {1, 1});
    points.add(new int[] {1, 2});
    points.add(new int[] {2, 0});

    final TextLCD t = LocalEV3.get().getTextLCD();
    odometer = new Odometer(leftMotor, rightMotor);
    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);
    
    // clear the display
    t.clear();
    
    t.drawString("Press any button.", 0, 0);
    Button.waitForAnyPress();
    
    odometer.start();
    odometryDisplay.start();
    
    Thread pollingThread = (new Thread() {
      public void run() {
        @SuppressWarnings("resource")
        SensorModes usSensor = new EV3UltrasonicSensor(usPort);
        SampleProvider usDistance = usSensor.getMode("Distance");
        SampleProvider usFiltered = new MedianFilter(usDistance, 5);
        float[] usData = new float[usFiltered.sampleSize()];
        
        int distance;
        
        while (true) {
          usFiltered.fetchSample(usData, 0); // acquire data
          distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
          
          t.drawString("Distance: " + distance, 0, 3);
          
          if (distance < DISTANCE_THRESHOLD && distance != 0) { //Ignore zero readings
            setObjectDetection(true);
          }
          
          try {
            Thread.sleep(50);
          } catch (Exception e) {
          } // Poor man's timed sampling
        }
      }
    });
    pollingThread.start();
    
    try { //Wait 5 seconds
      Thread.sleep(5000);
    } catch (Exception e) {
    }
  
    while (pointsIndex < points.size()) {
      Thread navigationThread = (new Thread() {
        public void run() {
          Navigation n = new Navigation(odometer, leftMotor, rightMotor);
          n.travelTo(points.get(pointsIndex)[0], points.get(pointsIndex)[1]);
          
          while (n.isNavigating()) { //While robot is moving
            if (getObjectDetection()) { //If detect obstacle
              leftMotor.stop(); //Stop motors
              rightMotor.stop();
              
              n.avoidObject(); //Do avoidance
              setObjectDetection(false);
              return; //Leave thread
            }
          }
          
          pointsIndex++;
        }
      });
      navigationThread.start();
      
      try {
        navigationThread.join(); //Wait for thread to finish
      } catch (InterruptedException e) {
        //Should never be interrupted
      }
    }
    System.exit(0);
  }
}
