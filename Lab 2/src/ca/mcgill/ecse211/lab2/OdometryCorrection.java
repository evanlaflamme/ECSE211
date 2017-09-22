/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.lab2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import lejos.hardware.Sound;
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
  
  private static final int SAMPLE_SIZE = 10;
  private ArrayList<Integer> samples = new ArrayList<Integer>();
  
  private int lastBeepCounter = 0;

  // constructor
  public OdometryCorrection(Odometer odometer) {
    this.odometer = odometer;
    
    Port cPort = LocalEV3.get().getPort("S1");
    cSensor = new EV3ColorSensor(cPort);
    SampleProvider cAmbient = cSensor.getMode(1); //Ambient mode to get light intensity
    //cFilter = new MedianFilter(cAmbient, 15);
    cFilter = cAmbient;
    
    cSensor.setFloodlight(Color.WHITE);
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();
      
	  float[] cData = new float[cFilter.sampleSize()];
      cFilter.fetchSample(cData, 0);
      
      samples.add((int) (cData[0] * 100));
      
      int[] convSamples = new int[SAMPLE_SIZE];
      
      if (samples.size() == SAMPLE_SIZE + 1) {
    	  samples.remove(0); //Remove first (oldest sample)
    	  
    	  if (lastBeepCounter == 0) {
        	  Integer[] samplesArray = samples.toArray(new Integer[samples.size()]);
        	  
        	  //convSamples = conv1d(samplesArray, new float[] {-0.25F, 0.5F, 0.25F});
        	  convSamples = derivative(samplesArray);
        	  
        	  if (rateOfChange(Arrays.copyOfRange(convSamples, SAMPLE_SIZE - 4, SAMPLE_SIZE - 1)) > 1.0) {
        		  //Line
        	        Sound.setVolume(70);
        	        Sound.beep();
        	        System.out.println("Beep");
        	        
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
  
  private float rateOfChange(int[] arr) {
	  return (arr[arr.length - 1] - arr[0]) / arr.length;
  }
  
  private int maxOfArray(int[] arr) {
	  int max = 0;
	  
	  for (int i = 0; i < arr.length; i++) {
		  if (arr[i] > max) {
			  max = arr[i];
		  }
	  }
	  
	  return max;
  }
  
  private int minOfArray(int[] arr) {
	  int min = 100;
	  
	  for (int i = 0; i < arr.length; i++) {
		  if (arr[i] < min) {
			  min = arr[i];
		  }
	  }
	  
	  return min;
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
  
  private int[] conv1d(Integer[] samples, float[] window) {
	  int[] convSum = new int[samples.length];
	  
	  for (int i = 0; i < samples.length; i++) {
		  convSum[i] = 0;
		  
		  for (int j = 0; j < window.length; j++) {
			  if (i - j > 0) {
				  convSum[i] += (int)(samples[i - j] * window[j]);
			  }			  
		  }
	  }
	  
	  return convSum;
  }
}
