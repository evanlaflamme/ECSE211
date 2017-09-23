/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.lab2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import lejos.hardware.Sound;
import lejos.hardware.Sounds;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import lejos.robotics.filter.MeanFilter;


public class OdometryCorrection extends Thread {
  private static final long CORRECTION_PERIOD = 5;
  private Odometer odometer;
  private EV3ColorSensor cSensor;
  private SampleProvider cFilter;
  
  private static final double SQUARE_LENGTH = 30.48;
  private static final int SAMPLE_SIZE = 10;
  private ArrayList<Integer> samples = new ArrayList<Integer>(); //Array that holds previous samples in order to compare data
  
  private int lastBeepCounter = 0; //Holds the counter that counts iterations since last beep
  
  private double previousX = 0;
  private double previousY = 0;
  private double previousT = 0;
  
  private double offsetX = 0;
  private double offsetY = 0;

  // constructor
  public OdometryCorrection(Odometer odometer) {
    this.odometer = odometer;
    
    Port cPort = LocalEV3.get().getPort("S1");
    cSensor = new EV3ColorSensor(cPort);
    SampleProvider cAmbient = cSensor.getMode(1); //Ambient mode to get light intensity
    //cFilter = new MedianFilter(cAmbient, 15);
    cFilter = cAmbient;
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();
      
      cSensor.setFloodlight(Color.WHITE); //Set floodlight color to white to try and help with black line detection
      
	  float[] cData = new float[cFilter.sampleSize()];
      cFilter.fetchSample(cData, 0);
      
      samples.add((int) (cData[0] * 100)); //Add newest sample to sample array (as integer)
      
      int[] derivSamples = new int[SAMPLE_SIZE]; //Holds the derivative of samples
      
      if (samples.size() == SAMPLE_SIZE + 1) { //Ignore sample set if not full (only affects first few seconds)
    	  samples.remove(0); //Remove oldest sample from set
    	  
    	  if (lastBeepCounter == 0) { //Only do correction if last beep was 10 iterations away
        	  Integer[] samplesArray = samples.toArray(new Integer[samples.size()]);
        	  
        	  derivSamples = derivative(samplesArray);
        	  
        	  if (rateOfChange(Arrays.copyOfRange(derivSamples, SAMPLE_SIZE - 4, SAMPLE_SIZE - 1)) > 2.9F) {
        	    double[] position = new double[3];
        	    
        	    odometer.getPosition(position, new boolean[] {true, true, true});
        	    
        	    double theta = thetaCloseTo(position[2]);
        	    
        	    if (previousY == 0 && theta == 0) { //First line cross
        	    	previousY = position[1];
        	    	offsetY = position[1];
        	    } else if (theta == 0) { //First vertical path
        	    	previousY += SQUARE_LENGTH;
        	    } else if (previousT == 0 && theta == 90) { //First turn
        	    	previousX = position[0];
        	    	offsetX = position[0];
        	    	
        	    	previousY += (SQUARE_LENGTH - offsetY);
        	    } else if (theta == 90) { //First horizontal path
        	    	previousX += SQUARE_LENGTH;
        	    } else if (previousT == 90 && theta == 180) { //Turn
        	    	previousX += (SQUARE_LENGTH - offsetX);
        	    	
        	    	previousY -= (SQUARE_LENGTH - offsetY);
        	    } else if (theta == 180) { //Second vertical path
        	    	previousY -= SQUARE_LENGTH;
        	    } else if (previousT == 180 && theta == 270) {
        	    	previousX -= (SQUARE_LENGTH - offsetX);
        	    	
        	    	previousY -= offsetY;
        	    } else if (theta == 270) {
        	    	previousX -= SQUARE_LENGTH;
        	    }
        	    
        	    position[0] = previousX;
        	    position[1] = previousY;
        	    
        	    previousT = theta;
        	    
        	    //System.out.println("Corrected X: " + position[0] + "    Corrected Y: " + position[1] + "    Theta: " + theta);
        	    
        	    odometer.setPosition(position, new boolean[] {true, true, false});
        	    
        	    Sound.setVolume(30);
        	    Sound.beep();
        	    
        	    lastBeepCounter = 10;
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
  
  private double thetaCloseTo(double theta) { //Returns the closest "perfect" angle to theta
    int error = 5;
    
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
  
  private float rateOfChange(int[] arr) {
	  return (arr[arr.length - 1] - arr[0]) / arr.length;
  }
  
  private int[] derivative(Integer[] samples) {
	  int[] deriv = new int[samples.length];
	  
	  for (int i = 0; i < samples.length; i++) {
		  if (i == samples.length - 1) {
			  deriv[i] = 0;
		  } else {
			  deriv[i] = samples[i + 1] - samples[i];
		  }
	  }
	  
	  return deriv;
  }
}
