package ca.mcgill.ecse211.lab4;

import java.util.ArrayList;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;


public class LightLocalizer extends Thread {
  private static final long CORRECTION_PERIOD = 5;
  private Odometer odometer;
  private Navigation navigation;
  private EV3ColorSensor cSensor;
  private SampleProvider cFiltered;

  private static final int SAMPLE_SIZE = 10;
  private ArrayList<Float> samples = new ArrayList<Float>(); // Array that holds previous
                                                                 // samples in order to compare data
  
  private static final float UPPER_THRESHOLD = 1.2F;
  private static final float LOWER_THRESHOLD = -1.2F;
  private static final double DISTANCE_FROM_CENTER = 9.5;

  private int lastBeepCounter = 0; // Holds the counter that counts iterations since last beep
  
  // constructor
  public LightLocalizer(Odometer odometer, Navigation navigation) {
    this.odometer = odometer;
    this.navigation = navigation;

    Port cPort = LocalEV3.get().getPort("S1");
    cSensor = new EV3ColorSensor(cPort);
    SampleProvider cAmbient = cSensor.getMode(1); // Ambient mode to get light intensity
    cFiltered = new MedianFilter(cAmbient, 5); //Use median filter to remove noise
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    double thetaX1 = 0, thetaX2 = 0, thetaY1 = 0, thetaY2 = 0;
    
    navigation.turnTo(360, true, false);

    while (navigation.isNavigating()) {
      correctionStart = System.currentTimeMillis();
      
      int numLinesDetected = 0;

      float[] cData = new float[cFiltered.sampleSize()];
      cFiltered.fetchSample(cData, 0);

      samples.add((cData[0] * 100)); // Add newest sample to sample array (as integer)

      float[] derivSamples = new float[SAMPLE_SIZE]; // Holds the derivative of samples

      if (samples.size() == SAMPLE_SIZE + 1) { // Ignore sample set if not full (only affects first
                                               // few milliseconds)
        samples.remove(0); // Remove oldest sample from set

        if (lastBeepCounter == 0) { // Only do correction if last beep was 10 iterations away
          Float[] samplesArray = samples.toArray(new Float[samples.size()]);

          derivSamples = derivative(samplesArray);

          if (max(derivSamples) > UPPER_THRESHOLD && min(derivSamples) < LOWER_THRESHOLD) { //Line detected
              numLinesDetected++;
              double currentTheta = odometer.getTheta();
              
              switch (numLinesDetected) {
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

        	  
            Sound.setVolume(70);
            Sound.beep();

            lastBeepCounter = 5; // Reset beep counter
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
    
    double thetaX = (thetaX2 - thetaX1)/2;
    double thetaY = (thetaY2 - thetaY1)/2;
    //double thetaX = ((180 - thetaX1) % 360 + 360) % 360;
    //double thetaY = ((270 - thetaY1) % 360 + 360) % 360;
    
    double positionX = (-1 * DISTANCE_FROM_CENTER) * Math.abs(Math.cos(thetaY));
    double positionY = (-1 * DISTANCE_FROM_CENTER) * Math.abs(Math.cos(thetaX));
    
    TextLCD t = LocalEV3.get().getTextLCD();
    t.drawString("New X:          " + positionX, 0, 3);
    t.drawString("New Y:          " + positionY, 0, 4);
    
    odometer.setX(positionX);
    odometer.setY(positionY);
    
    navigation.travelTo(0, 0);
    
    while (navigation.isNavigating()) {
      //Wait to get to point
    }
    
    navigation.turnTo(-odometer.getTheta(), true, true);
  }

  private float max(float[] arr) {
    float max = -255;
    
    for (int i = 0; i < arr.length; i++) {
      if (arr[i] > max) {
        max = arr[i];
      }
    }
    
    return max;
  }
  
  private float min(float[] arr) {
    float min = 255;
    
    for (int i = 0; i < arr.length; i++) {
      if (arr[i] < min) {
        min = arr[i];
      }
    }
    
    return min;
  }

  private float[] derivative(Float[] samples) { // Returns the discrete derivative of the array
    float[] deriv = new float[samples.length];

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
