package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;

public class UltrasonicLocalizer extends Thread {
  private int distance = -1;
  
  private static final int RISING_THRESHOLD = 30;
  private static final int FALLING_THRESHOLD = 60;

  public enum Method { RISING_EDGE, FALLING_EDGE };
  private Method method;
  
  private Navigation navigation;
  private Odometer odometer;
  
  private double theta1;
  private double theta2;
  
  public UltrasonicLocalizer(Method method, Navigation navigation, Odometer odometer) {
    this.method = method;
    this.navigation = navigation;
    this.odometer = odometer;
  }
  
  public void run() {
	  navigation.turnTo(360, true, false);
	  
	  while (distance == -1) {
		  //Wait until we have data
	  }
	  
	  if (method == Method.RISING_EDGE) {
		  risingEdge();
	  } else {
		  fallingEdge();
	  }
  }
  
  public void processUSData(int distance) {
	  this.distance = distance;
  }
  
  private void risingEdge() {
	  while (distance < RISING_THRESHOLD) {
		  //Wait until first rising edge
	  }
	  Sound.setVolume(70);
	  Sound.beep();
	  navigation.stopMotors();
	  theta1 = odometer.getTheta();
	  
	  navigation.turnTo(-360, true, false);
	  
	  try {
	      Thread.sleep(2000);
	    } catch (Exception e) {
	    }
	  
	  while (distance < RISING_THRESHOLD) {
		  //Wait until second edge
	  }
	  Sound.beep();
	  navigation.stopMotors();
	  theta2 = odometer.getTheta();
	  
	  correctHeading();
  }
  
  private void fallingEdge() {
	  while (distance > FALLING_THRESHOLD) {
		  //Wait until first rising edge
	  }
	  Sound.setVolume(70);
	  Sound.beep();
	  navigation.stopMotors();
	  theta1 = odometer.getTheta();
	  
	  navigation.turnTo(-360, true, false);
	  
	  try {
	      Thread.sleep(3500);
	    } catch (Exception e) {
	    }
	  
	  while (distance > FALLING_THRESHOLD) {
		  //Wait until second edge
	  }
	  Sound.beep();
	  navigation.stopMotors();
	  theta2 = odometer.getTheta();
	  
	  correctHeading();
  }
  
  private void correctHeading() {
	  double correctionTheta = 0;
	  
	  if (method == Method.RISING_EDGE) {
		  correctionTheta = 45 - ((theta1 + theta2) / 2);
	  } else {
		  correctionTheta = 225 - ((theta1 + theta2) / 2);
	  }
	  
	  correctionTheta += odometer.getTheta();	  
	  correctionTheta = ((correctionTheta % 360) + 360) % 360;
	  
	  odometer.setTheta(correctionTheta);
	  
	  navigation.turnTo(-correctionTheta, false, true);
  }
}
