package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.port.;
import lejos.hardware.sensor.;
import lejos.remote.ev3.;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
