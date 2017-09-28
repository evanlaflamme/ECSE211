package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
  private static final int FORWARD_SPEED = 190;
  private static final int ROTATE_SPEED = 150;
  
  private Odometer odometer;
  
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  
  private boolean navigating;
  
  public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.navigating = false;
  }
  
  void travelTo(double x, double y) {
    navigating = true;
    
    x = x * NavigationLab.SQUARE_LENGTH;
    y = y * NavigationLab.SQUARE_LENGTH;
    
    double currentX = odometer.getX();
    double currentY = odometer.getY();
    double currentTheta = odometer.getTheta();
    
    double angle = (Math.atan2(x - currentX, y - currentY) * Math.PI) / 180;
    angle = ((angle) % 360 + 360) % 360; //Must be within 0 and 360
    
    if (currentTheta != angle) {
      turnTo(angle);
    }
    
    double distance = Math.sqrt(Math.pow(y - currentY, 2) + Math.pow(x - currentX, 2));
    
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    leftMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance), true);
    rightMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance), false);
  }
  
  void turnTo(double theta) { 
    navigating = true;
    
    double currentTheta = odometer.getTheta();
    
    double turnTheta = ((currentTheta - theta) % 360 + 360) % 360; //Turn theta must be within 0 and 360
    
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    leftMotor.rotate(convertAngle(NavigationLab.WHEEL_RADIUS, NavigationLab.TRACK, turnTheta), true);
    rightMotor.rotate(-convertAngle(NavigationLab.WHEEL_RADIUS, NavigationLab.TRACK, turnTheta), false);
  }
  
  boolean isNavigating() {
    return navigating;
  }
  
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}
