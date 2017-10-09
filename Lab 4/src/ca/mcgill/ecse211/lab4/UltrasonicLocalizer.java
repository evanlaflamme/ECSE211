package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;

public class UltrasonicLocalizer extends Thread {
  private int distance = -1;

  private static final int RISING_THRESHOLD = 30;
  private static final int FALLING_THRESHOLD = 60;

  public enum Method {
    RISING_EDGE, FALLING_EDGE
  };

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
    navigation.turnTo(360, true, false); // Perform a full turn, return before completion

    while (distance == -1) {
      // Wait until we have data
    }

    if (method == Method.RISING_EDGE) { // Call appropriate method
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
      // Wait until first rising edge
    }
    Sound.setVolume(70);
    Sound.beep();
    navigation.stopMotors(); // Stop motors when rising edge is found
    theta1 = odometer.getTheta(); // Save theta 1

    navigation.turnTo(-360, true, false); // Turn the opposite way

    try {
      Thread.sleep(2000);
    } catch (Exception e) {
    }

    while (distance < RISING_THRESHOLD) {
      // Wait until second edge
    }
    Sound.beep();
    navigation.stopMotors(); // Stop motors after second edge
    theta2 = odometer.getTheta(); // Save theta 2

    correctHeading();
  }

  private void fallingEdge() {
    while (distance > FALLING_THRESHOLD) {
      // Wait until first rising edge
    }
    Sound.setVolume(70);
    Sound.beep();
    navigation.stopMotors(); // Stop motors when rising edge is found
    theta1 = odometer.getTheta(); // Save theta 1

    navigation.turnTo(-360, true, false); // turn the opposite way

    try {
      Thread.sleep(3500);
    } catch (Exception e) {
    }

    while (distance > FALLING_THRESHOLD) {
      // Wait until second edge
    }
    Sound.beep();
    navigation.stopMotors(); // Stop motors after second edge
    theta2 = odometer.getTheta(); // Save theta 2

    correctHeading();
  }

  private void correctHeading() {
    double correctionTheta = 0;

    if (method == Method.RISING_EDGE) { // Correct the theta
      correctionTheta = 45 - ((theta1 + theta2) / 2);
    } else {
      correctionTheta = 225 - ((theta1 + theta2) / 2);
    }

    correctionTheta += odometer.getTheta();
    correctionTheta = ((correctionTheta % 360) + 360) % 360; // Ensure that it is between 0 and 360

    odometer.setTheta(correctionTheta); // Set new theta

    navigation.turnTo(-correctionTheta, false, true); // Turn to 0 degrees
  }
}
