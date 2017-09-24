/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.lab2;

import java.util.ArrayList;
import java.util.Arrays;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;


public class OdometryCorrection extends Thread {
  private static final long CORRECTION_PERIOD = 5;
  private Odometer odometer;
  private EV3ColorSensor cSensor;
  private SampleProvider cFilter;

  private static final double SQUARE_LENGTH = 30.48;
  private static final int SAMPLE_SIZE = 10;
  private ArrayList<Integer> samples = new ArrayList<Integer>(); // Array that holds previous
                                                                 // samples in order to compare data

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
    // cFilter = new MedianFilter(cAmbient, 15);
    cFilter = cAmbient;
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();

      cSensor.setFloodlight(Color.WHITE); // Set floodlight color to white to try and help with
                                          // black line detection

      float[] cData = new float[cFilter.sampleSize()];
      cFilter.fetchSample(cData, 0);

      samples.add((int) (cData[0] * 100)); // Add newest sample to sample array (as integer)

      int[] derivSamples = new int[SAMPLE_SIZE]; // Holds the derivative of samples

      if (samples.size() == SAMPLE_SIZE + 1) { // Ignore sample set if not full (only affects first
                                               // few milliseconds)
        samples.remove(0); // Remove oldest sample from set

        if (lastBeepCounter == 0) { // Only do correction if last beep was 10 iterations away
          Integer[] samplesArray = samples.toArray(new Integer[samples.size()]);

          derivSamples = derivative(samplesArray);

          // If rate of change of last 5 samples is above threshold, then over line
          if (rateOfChange(
              Arrays.copyOfRange(derivSamples, SAMPLE_SIZE - 4, SAMPLE_SIZE - 1)) > 2.9F) {
            double[] position = new double[3];

            odometer.getPosition(position, new boolean[] {true, true, true}); // Get odometer
                                                                              // position

            double theta = getThetaEstimate(position[2]); // Get square theta

            if (previousY == 0 && theta == 0) { // First line cross, set initial y position and y
                                                // offset
              previousY = position[1];
              offsetY = position[1];
            } else if (theta == 0) { // Along first vertical path, add square length to y
              previousY += SQUARE_LENGTH;
            } else if (previousT == 0 && theta == 90) { // First line after first turn, set initial
                                                        // x and x offset, and add offset to Y
              previousX = position[0];
              offsetX = position[0];

              previousY += (SQUARE_LENGTH - offsetY);
            } else if (theta == 90) { // Along first horizontal path, add square length to x
              previousX += SQUARE_LENGTH;
            } else if (previousT == 90 && theta == 180) { // First line after second turn, add
                                                          // offset to x and subtract offset from y
              previousX += (SQUARE_LENGTH - offsetX);

              previousY -= (SQUARE_LENGTH - offsetY);
            } else if (theta == 180) { // Second vertical path, subtract square length from y (going
                                       // toward zero)
              previousY -= SQUARE_LENGTH;
            } else if (previousT == 180 && theta == 270) { // First line after third turn, subtract
                                                           // offsets from x and y
              previousX -= (SQUARE_LENGTH - offsetX);

              previousY -= offsetY;
            } else if (theta == 270) { // Last few lines, subtract square lengths from x
              previousX -= SQUARE_LENGTH;
            }

            position[0] = previousX; // Set updated positions
            position[1] = previousY;

            previousT = theta; // Store last theta

            odometer.setPosition(position, new boolean[] {true, true, false}); // Update positions

            Sound.setVolume(30);
            Sound.beep(); // Beep to indicate line crossing

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

  private float rateOfChange(int[] arr) { // Returns the rate of change of the array
    return (arr[arr.length - 1] - arr[0]) / arr.length;
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
