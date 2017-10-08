package ca.mcgill.ecse211.lab4;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
  private static final int FORWARD_SPEED = 190;
  private static final int ROTATE_SPEED = 150;

  private Odometer odometer;

  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor) {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  // Travels to specified x and y from current position
  void travelTo(double x, double y) {
    x = x * LocalizationLab.SQUARE_LENGTH; // Convert to odometer coordinates
    y = y * LocalizationLab.SQUARE_LENGTH;

    double currentX = odometer.getX(); // Get current odometer values
    double currentY = odometer.getY();
    double currentTheta = odometer.getTheta();

    double angle = Math.atan2(x - currentX, y - currentY); // Get angle between points
    angle = angle * 180 / Math.PI; // Convert to degree

    if (currentTheta != angle) {
      double turnTheta = angle - currentTheta;

      turnTo(turnTheta, false, true); // Turn to new angle
    }

    double distance = Math.sqrt(Math.pow(y - currentY, 2) + Math.pow(x - currentX, 2)); // Distance
                                                                                        // to travel

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    // Travel to point
    leftMotor.rotate(convertDistance(LocalizationLab.WHEEL_RADIUS, distance), true);
    rightMotor.rotate(convertDistance(LocalizationLab.WHEEL_RADIUS, distance), true);
  }

  // Turns to theta relative to current angle
  // 'ret' designates if the method should return before completing or not
  // 'small' designates whether the robot should take the shorter turn or not
  void turnTo(double theta, boolean ret, boolean small) {
    if (small) {
      if (theta <= -180 && theta >= -359) { // Ensures that robot takes minimum angle
        theta += 360;
      } else if (theta >= 180 && theta <= 359) {
        theta -= 360;
      } else if (theta >= 360 || theta <= -360) {
        theta = theta % 360;
      }
    }
    
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    // Rotate to new angle
    leftMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, theta), true);
    rightMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, theta), false || ret);
  }
  
  void stopMotors() {
	  leftMotor.setSpeed(0);
	  rightMotor.setSpeed(0);
  }

  // Returns true if the robot is moving
  boolean isNavigating() {
    return leftMotor.isMoving() && rightMotor.isMoving();
  }

  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}
