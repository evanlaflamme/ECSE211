package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;


public class LightLocalizer extends Thread {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private Navigation navigation;
  private EV3ColorSensor cSensor;
  private SampleProvider cFiltered;

  private static final int THRESHOLD = 25;
  private static final int MIN_DATA_THRESHOLD = 5;
  private static final double DISTANCE_FROM_CENTER = 9.5;

  private int lastBeepCounter = 0; // Holds the counter that counts iterations since last beep

  // constructor
  public LightLocalizer(Odometer odometer, Navigation navigation) {
    this.odometer = odometer;
    this.navigation = navigation;

    Port cPort = LocalEV3.get().getPort("S1");
    cSensor = new EV3ColorSensor(cPort);
    SampleProvider cAmbient = cSensor.getMode(1); // Ambient mode to get light intensity
    cFiltered = new MedianFilter(cAmbient, 5); // Use median filter to remove noise
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    double thetaX1 = 0, thetaX2 = 0, thetaY1 = 0, thetaY2 = 0;

    navigation.travelTo(2, 2); // Travel toward origin

    while (navigation.isNavigating()) { // While robot is turning
      correctionStart = System.currentTimeMillis();

      if (detectLine()) { // If line detected, stop moving
        navigation.stopMotors();

        Sound.setVolume(70);
        Sound.beep();
        break;
      }

      // this ensure the detection occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
        }
      }
    }

    navigation.travelTo((odometer.getX() - DISTANCE_FROM_CENTER) / LocalizationLab.SQUARE_LENGTH,
        (odometer.getY() - DISTANCE_FROM_CENTER) / LocalizationLab.SQUARE_LENGTH); // Return towards
                                                                                   // origin

    while (navigation.isNavigating()) {
      // Wait until we are at the origin
    }

    navigation.turnTo(-odometer.getTheta(), false, true); // Turn back to 0 degrees

    navigation.turnTo(360, true, false); // Perform a full 360 degree turn, returning before
                                         // completion

    while (navigation.isNavigating()) { // While robot is turning
      correctionStart = System.currentTimeMillis();

      int numLinesDetected = 0;

      if (detectLine()) { // If line detected
        numLinesDetected++;
        double currentTheta = odometer.getTheta();

        switch (numLinesDetected) { // Set theta depending on which line is detected
          case 1:
            thetaX1 = currentTheta;
            break;
          case 2:
            thetaY1 = currentTheta;
            break;
          case 3:
            thetaX2 = currentTheta;
            break;
          case 4:
            thetaY2 = currentTheta;
            break;
        }

        // this ensure the detection occurs only once every period
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
          } catch (InterruptedException e) {
          }
        }
      }
    }

    // Calculate theta x and theta y once robot is done turning
    double thetaX = (thetaX2 - thetaX1) / 2;
    double thetaY = (thetaY2 - thetaY1) / 2;

    // Calculate x and y positions
    double positionX = Math.cos(thetaY);
    double positionY = Math.cos(thetaX);

    // Set new positions
    odometer.setX(positionX);
    odometer.setY(positionY);

    navigation.travelTo(0, 0); // Travel to origin

    while (navigation.isNavigating()) {
      // Wait until robot is at the point
    }

    navigation.turnTo(-odometer.getTheta(), true, true); // Turn to 0 degrees
  }

  private boolean detectLine() { // Returns true if a line is detected
    float[] cData = new float[cFiltered.sampleSize()];
    cFiltered.fetchSample(cData, 0); // Get data from sensor

    while ((cData[0] * 100) < MIN_DATA_THRESHOLD) { // Ensure that the sensor is getting usable data
      try {
        Thread.sleep(50);
      } catch (InterruptedException e) {
      }

      cFiltered.fetchSample(cData, 0);
    }

    if (lastBeepCounter == 0) { // Ensures that last line detect was enough time ago
      if ((cData[0] * 100) < THRESHOLD) { // If data is less than threshold, line detected
        Sound.setVolume(70);
        Sound.beep(); // Beep

        lastBeepCounter = 20; // Reset beep counter

        return true;
      }
    } else {
      lastBeepCounter--; // Otherwise, lower the beep counter
    }

    return false;
  }
}
