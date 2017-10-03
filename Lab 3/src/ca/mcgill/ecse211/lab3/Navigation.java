package ca.mcgill.ecse211.lab3;

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
    x = x * NavigationLab.SQUARE_LENGTH; // Convert to odometer coordinates
    y = y * NavigationLab.SQUARE_LENGTH;

    double currentX = odometer.getX(); // Get current odometer values
    double currentY = odometer.getY();
    double currentTheta = odometer.getTheta();

    double angle = Math.atan2(x - currentX, y - currentY); // Get angle between points
    angle = angle * 180 / Math.PI; // Convert to degree

    if (currentTheta != angle) {
      double turnTheta = angle - currentTheta;

      turnTo(turnTheta); // Turn to new angle
    }

    double distance = Math.sqrt(Math.pow(y - currentY, 2) + Math.pow(x - currentX, 2)); // Distance
                                                                                        // to travel

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    // Travel to point
    leftMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance), true);
    rightMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance), true);
  }

  // Turns to theta relative to current angle
  void turnTo(double theta) {
    if (theta <= -180 && theta >= -359) { // Ensures that robot takes minimum angle
      theta += 360;
    } else if (theta >= 180 && theta <= 359) {
      theta -= 360;
    } else if (theta >= 360 || theta <= -360) {
      theta = theta % 360;
    }

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    // Rotate to new angle
    leftMotor.rotate(convertAngle(NavigationLab.WHEEL_RADIUS, NavigationLab.TRACK, theta), true);
    rightMotor.rotate(-convertAngle(NavigationLab.WHEEL_RADIUS, NavigationLab.TRACK, theta), false);
  }

  // First step of avoidance
  // Travel distance 90 degrees from current position to avoid object
  void avoidObject_1(int direction, double distance) {
    turnTo(90 * direction);

    leftMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance), true);
    rightMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance), true);
  }

  // Second step of avoidance
  // Travel back in direction of point relative to original position of robot
  void avoidObject_2(int direction, double distance) {
    turnTo(-90 * direction);

    leftMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance * 1.55), true);
    rightMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance * 1.55), true);
  }

  // Third and last step of avoidance
  // Return to exact opposite side of the obstacle relative to original position of robot
  void avoidObject_3(int direction, double distance) {
    turnTo(-90 * direction);

    leftMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance * 0.5), true);
    rightMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance * 0.5), true);
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
