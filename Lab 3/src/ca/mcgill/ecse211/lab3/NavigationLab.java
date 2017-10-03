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

  private static ArrayList<int[]> points = new ArrayList<int[]>(); // Array to hold the points
  private static int pointsIndex = 0; // Holds the current index in the array

  public static final double WHEEL_RADIUS = 2.07;
  public static final double TRACK = 15.5;
  public static final double SQUARE_LENGTH = 30.48;

  private static final int DISTANCE_THRESHOLD = 10; // Distance from the object for detection

  // Bounds of the board
  private static final double MAX_X = 3 * NavigationLab.SQUARE_LENGTH;
  private static final double MIN_X = -1 * NavigationLab.SQUARE_LENGTH;
  private static final double MAX_Y = 3 * NavigationLab.SQUARE_LENGTH;
  private static final double MIN_Y = -1 * NavigationLab.SQUARE_LENGTH;

  // Sets the object detected boolean safely (for multiple threads to access)
  private synchronized static void setObjectDetection(boolean b) {
    objectDetected = b;
  }

  // Gets the object detected boolean safely
  private synchronized static boolean getObjectDetection() {
    return objectDetected;
  }

  public static void main(String[] args) {
    // Map 1
     //points.add(new int[] {0, 2}); 
     //points.add(new int[] {1, 1}); 
     //points.add(new int[] {2, 2});
     //points.add(new int[] {2, 1}); 
     //points.add(new int[] {1, 0});
     
    // Map 2
    /*
     * points.add(new int[] {1, 1}); points.add(new int[] {0, 2}); points.add(new int[] {2, 2});
     * points.add(new int[] {2, 1}); points.add(new int[] {1, 0});
     */

    // Map 3
    
     points.add(new int[] {1, 0}); 
     points.add(new int[] {2, 1}); 
     points.add(new int[] {2, 2});
     points.add(new int[] {0, 2}); 
     points.add(new int[] {1, 1});
     

    // Map 4
    /*
     * points.add(new int[] {0, 1}); points.add(new int[] {1, 2}); points.add(new int[] {1, 0});
     * points.add(new int[] {2, 1}); points.add(new int[] {2, 2});
     */

    final TextLCD t = LocalEV3.get().getTextLCD();
    odometer = new Odometer(leftMotor, rightMotor);
    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);

    // clear the display
    t.clear();

    t.drawString("Press any button.", 0, 0);
    Button.waitForAnyPress();

    odometer.start();
    odometryDisplay.start();

    // Thread for polling
    // Started in main thread so that it has easy access to the object detected global variable
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

          t.drawString("Distance: " + distance, 0, 3); // Show distane on the screen

          if (distance < DISTANCE_THRESHOLD && distance != 0) { // If object detected ignoring zero
                                                                // readings
            setObjectDetection(true); // Set object detected to true
          }

          try {
            Thread.sleep(50);
          } catch (Exception e) {
          } // Poor man's timed sampling
        }
      }
    });
    pollingThread.start();

    try { // Wait 5 seconds for ultrasonic sensor to activate
      Thread.sleep(5000);
    } catch (Exception e) {
    }

    while (pointsIndex < points.size()) { // While there are still points to visit
      // Start navigation thread
      Thread navigationThread = (new Thread() {
        public void run() {
          Navigation n = new Navigation(odometer, leftMotor, rightMotor);
          n.travelTo(points.get(pointsIndex)[0], points.get(pointsIndex)[1]); // Navigate to current
                                                                              // point

          if (detectObject(n)) { // Check for object detection
            avoidObject(n); // Avoid object is detected
            return;
          }

          pointsIndex++; // Only increment points index if object was not detected
        }

        // Returns true if object is detected during navigation
        private boolean detectObject(Navigation n) {
          while (n.isNavigating()) { // While robot is moving
            if (getObjectDetection()) { // If detect obstacle
              leftMotor.stop(); // Stop motors
              rightMotor.stop();

              return true;
            }
          }

          return false;
        }

        // Performs avoidance
        private void avoidObject(Navigation n) {
          double distance = NavigationLab.SQUARE_LENGTH; // The distance to travel
          double avoidanceOffset = NavigationLab.TRACK * 4; // Distance between center of
                                                                    // rotation and end of robot

          double currentX = odometer.getX(); // Get current odometer values
          double currentY = odometer.getY();
          double currentTheta = odometer.getTheta();

          // Components if robot is to turn right
          double rightTheta = 90 - currentTheta;
          double rightXComponent = Math.sin(rightTheta) * distance;
          double rightYComponent = Math.cos(rightTheta) * distance;

          // Components if robot is to turn left
          double leftTheta = 90 + currentTheta;
          double leftXComponent = Math.cos(leftTheta) * distance;
          double leftYComponent = Math.sin(leftTheta) * distance;

          int direction = 0; // 1 signifies turning right; -1 signifies turning left

          if (currentX + rightXComponent + avoidanceOffset < MAX_X
              && currentX + rightXComponent - avoidanceOffset > MIN_X
              && currentY + rightYComponent + avoidanceOffset < MAX_Y
              && currentY + rightYComponent - avoidanceOffset > MIN_Y) { // If possible to turn
                                                                         // right
            direction = 1;
          } else if (currentX + leftXComponent + avoidanceOffset < MAX_X
              && currentX + leftXComponent - avoidanceOffset > MIN_X
              && currentY + leftYComponent + avoidanceOffset < MAX_Y
              && currentY + leftYComponent - avoidanceOffset > MIN_Y) { // If possible to turn left
            direction = -1;
          }

          n.avoidObject_1(direction, distance); // Perform first step of avoidance

          setObjectDetection(false); // Only start detection once first turn complete (don't want to
                                     // detect same object twice)

          if (detectObject(n)) { // Check for object during first step
            avoidObject(n); // Avoid object and stop current avoidance
            return;
          }

          n.avoidObject_2(direction, distance); // Perform second step of avoidance

          if (detectObject(n)) { // Check for object during second step
            avoidObject(n); // Avoid object and stop current avoidance
            return;
          }

          n.avoidObject_3(direction, distance); // Perform third and last step of avoidance

          if (detectObject(n)) { // Check for object during third step
            avoidObject(n); // Avoid object and stop current avoidance
            return;
          }
        }
      });
      navigationThread.start();

      try {
        navigationThread.join(); // Wait for thread to finish before starting next navigation thread
      } catch (InterruptedException e) {
        // Should never be interrupted
      }
    }
    System.exit(0);
  }
}
