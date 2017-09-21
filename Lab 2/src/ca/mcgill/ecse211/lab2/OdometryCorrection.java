/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.lab2;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;

public class OdometryCorrection extends Thread {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private SampleProvider cFilter;
  
  private float lastIntensityLevel = 1;

  // constructor
  public OdometryCorrection(Odometer odometer) {
    this.odometer = odometer;
    
    Port cPort = LocalEV3.get().getPort("S1");
    SensorModes cSensor = new EV3ColorSensor(cPort);
    SampleProvider cAmbient = cSensor.getMode(3); //Ambient mode to get light intensity
    cFilter = new MedianFilter(cAmbient, 30);
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();

      float[] cData = new float[cFilter.sampleSize()];
      cFilter.fetchSample(cData, 0);
      
      if (cData[0] < lastIntensityLevel) { //Less intense than last check, so over band        
        double[] position = new double[3];
        
        odometer.getPosition(position, new boolean[] {true, true, true});
        
        for (int i = 0; i < position.length; i++) {
          position[i] = 0;
        }
        
        odometer.setPosition(position , new boolean[] {true, true, true});
      }
      
      lastIntensityLevel = cData[0];

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
}
