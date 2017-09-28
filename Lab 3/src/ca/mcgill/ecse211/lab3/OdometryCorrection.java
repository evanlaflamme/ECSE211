/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.lab3;

import java.util.ArrayList;
import java.util.Arrays;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import lejos.hardware.lcd.TextLCD;


public class OdometryCorrection extends Thread {
  private static final long CORRECTION_PERIOD = 5;
  private Odometer odometer;
  private EV3ColorSensor cSensor;
  private SampleProvider cFiltered;

  private static final int SAMPLE_SIZE = 10;
  private ArrayList<Integer> samples = new ArrayList<Integer>(); // Array that holds previous
                                                                 // samples in order to compare data
  
  private static final int UPPER_THRESHOLD = 6;
  private static final int LOWER_THRESHOLD = -6;

  private int lastBeepCounter = 0; // Holds the counter that counts iterations since last beep

  private double previousX = 0; // Holds previous positions
  private double previousY = 0;
  private double previousT = 0;

  private double offsetX = 0; // Holds offsets for x and y (initial distance from lines)
  private double offsetY = 0;

  // constructor
  public OdometryCorrection(Odometer odometer) {
    this.odometer = odometer;

    Port cPort = LocalEV3.get().getPort("S1");
    cSensor = new EV3ColorSensor(cPort);
    SampleProvider cAmbient = cSensor.getMode(1); // Ambient mode to get light intensity
    cFiltered = new MedianFilter(cAmbient, 5); //Use median filter to remove noise
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();

      float[] cData = new float[cFiltered.sampleSize()];
      cFiltered.fetchSample(cData, 0);

      samples.add((int) (cData[0] * 100)); // Add newest sample to sample array (as integer)

      int[] derivSamples = new int[SAMPLE_SIZE]; // Holds the derivative of samples

      if (samples.size() == SAMPLE_SIZE + 1) { // Ignore sample set if not full (only affects first
                                               // few milliseconds)
        samples.remove(0); // Remove oldest sample from set

        if (lastBeepCounter == 0) { // Only do correction if last beep was 10 iterations away
          Integer[] samplesArray = samples.toArray(new Integer[samples.size()]);

          derivSamples = derivative(samplesArray);

          if (max(derivSamples) > UPPER_THRESHOLD && min(derivSamples) < LOWER_THRESHOLD) {
            double[] position = new double[3];

            odometer.getPosition(position, new boolean[] {true, true, true}); // Get odometer
                                                                              // position

            double theta = getThetaEstimate(position[2]); // Get square theta

            if (offsetY == 0 && theta == 0) { // First line cross, set initial y position and y
                                                   // offset
              previousY = 0;
              offsetY = position[1];
              
              position[1] = previousY; //Set updated position
            } else if (theta == 0) { // Along first vertical path, add square length to y
              previousY += NavigationLab.SQUARE_LENGTH;
              
              position[1] = previousY; //Set updated position
            } else if (previousT == 0 && theta == 90) { // First line after first turn, set initial
                                                        // x and x offset, and add offset to Y
              previousX = 0;
              offsetX = position[0];

              previousY += (NavigationLab.SQUARE_LENGTH - offsetY);
              
              position[0] = previousX; // Set updated positions
              position[1] = previousY;
            } else if (theta == 90) { // Along first horizontal path, add square length to x
              previousX += NavigationLab.SQUARE_LENGTH;
              
              position[0] = previousX; // Set updated position
            } else if (previousT == 90 && theta == 180) { // First line after second turn, add
                                                          // offset to x and subtract offset from y
              previousX += (NavigationLab.SQUARE_LENGTH - offsetX);

              previousY -= (NavigationLab.SQUARE_LENGTH - offsetY);
              
              position[0] = previousX; // Set updated positions
              position[1] = previousY;
            } else if (theta == 180) { // Second vertical path, subtract square length from y (going
                                       // toward zero)
              previousY -= NavigationLab.SQUARE_LENGTH;
              
              position[1] = previousY; //Set updated position
            } else if (previousT == 180 && theta == 270) { // First line after third turn, subtract
                                                           // offsets from x and y
              previousX -= (NavigationLab.SQUARE_LENGTH - offsetX);

              previousY -= offsetY;
              
              position[0] = previousX; // Set updated positions
              position[1] = previousY;
            } else if (theta == 270) { // Last few lines, subtract square lengths from x
              previousX -= NavigationLab.SQUARE_LENGTH;
              
              position[0] = previousX; // Set updated position
            }

            previousT = theta; // Store last theta

            odometer.setPosition(position, new boolean[] {true, true, false}); // Update positions
            
            Sound.setVolume(10);
            Sound.beep();

            lastBeepCounter = 10; // Reset beep counter
          }
        } else {
          lastBeepCounter--;
        }
      }


      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometry correction will be
          // interrupted by another thread
        }
      }
    }
  }

  private double getThetaEstimate(double theta) { // Returns the closest "perfect" angle to theta
    int error = 5;

    // Return if theta is close to "perfect" angle, within error
    if (theta >= 90 - error && theta <= 90 + error) {
      return 90;
    } else if (theta >= 180 - error && theta <= 180 + error) {
      return 180;
    } else if (theta >= 270 - error && theta <= 270 + error) {
      return 270;
    } else {
      return 0;
    }
  }

  private int max(int[] arr) {
    int max = -255;
    
    for (int i = 0; i < arr.length; i++) {
      if (arr[i] > max) {
        max = arr[i];
      }
    }
    
    return max;
  }
  
  private int min(int[] arr) {
    int min = 255;
    
    for (int i = 0; i < arr.length; i++) {
      if (arr[i] < min) {
        min = arr[i];
      }
    }
    
    return min;
  }

  private int[] derivative(Integer[] samples) { // Returns the discrete derivative of the array
    int[] deriv = new int[samples.length];

    for (int i = 0; i < samples.length; i++) {
      if (i == samples.length - 1) { // If first derivative (does not exist), set it to zero
        deriv[i] = 0;
      } else { // Otherwise, set it as equation for discrete derivatives
        deriv[i] = samples[i + 1] - samples[i];
      }
    }

    return deriv;
  }
}
