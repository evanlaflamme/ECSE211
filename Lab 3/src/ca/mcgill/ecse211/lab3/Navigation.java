package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
  private static final int FORWARD_SPEED = 190;
  private static final int ROTATE_SPEED = 150;
  
  //Bounds of the board
  private static final double MAX_X = 3 * NavigationLab.SQUARE_LENGTH;
  private static final double MIN_X = -1 * NavigationLab.SQUARE_LENGTH;
  
  private static final double MAX_Y = 3 * NavigationLab.SQUARE_LENGTH;
  private static final double MIN_Y = -1 * NavigationLab.SQUARE_LENGTH;
  
  private Odometer odometer;
  
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  
  public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }
  
  void travelTo(double x, double y) {    
    x = x * NavigationLab.SQUARE_LENGTH; //Convert to odometer coordinates
    y = y * NavigationLab.SQUARE_LENGTH;
    
    double currentX = odometer.getX();
    double currentY = odometer.getY();
    double currentTheta = odometer.getTheta();
    
    double angle = Math.atan2(x - currentX, y - currentY); //Get angle between points
    angle = angle * 180 / Math.PI; //Convert to degree
    angle = positiveModulus(angle, 360); //Must be within 0 and 360
    
    if (currentTheta != angle) { //Ensure that the angle is the minimum
      double turnTheta = angle - currentTheta;
      
      if (turnTheta <= -180 && turnTheta >= -359) {
        turnTheta += 360;
      } else if (turnTheta >= 180 && turnTheta <= 359) {
        turnTheta -= 360;
      }
      
      turnTo(turnTheta);
    }
    
    double distance = Math.sqrt(Math.pow(y - currentY, 2) + Math.pow(x - currentX, 2));
    
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    leftMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance), true);
    rightMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance), true);
  }
  
  void turnTo(double theta) {    
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    leftMotor.rotate(convertAngle(NavigationLab.WHEEL_RADIUS, NavigationLab.TRACK, theta), true);
    rightMotor.rotate(-convertAngle(NavigationLab.WHEEL_RADIUS, NavigationLab.TRACK, theta), false);
  }
  
  void avoidObject() {
    double distance = (NavigationLab.SQUARE_LENGTH * 1.5);
    
    double currentX = odometer.getX();
    double currentY = odometer.getY();
    double currentTheta = odometer.getTheta();
    
    double rightTheta = 90 - currentTheta;
    double rightXComponent = Math.sin(rightTheta) * distance;
    double rightYComponent = Math.cos(rightTheta) * distance;
    
    double leftTheta = 90 + currentTheta;
    double leftXComponent = Math.cos(leftTheta) * distance;
    double leftYComponent = Math.sin(leftTheta) * distance;
    
    if (currentX + rightXComponent < MAX_X && currentX + rightXComponent > MIN_X &&
        currentY + rightYComponent < MAX_Y && currentY + rightYComponent > MIN_Y) {
      turnTo(positiveModulus(currentTheta + 90, 360));
    } else if (currentX + leftXComponent < MAX_X && currentX + leftXComponent > MIN_X &&
        currentY + leftYComponent < MAX_Y && currentY + leftYComponent > MIN_Y) {
      turnTo(positiveModulus(currentTheta - 90, 360));
    }
    
    leftMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance), true);
    rightMotor.rotate(convertDistance(NavigationLab.WHEEL_RADIUS, distance), false);
  }
  
  boolean isNavigating() {
    return leftMotor.isMoving() && rightMotor.isMoving();
  }
  
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  
  private double positiveModulus(double num, int val) {
    return ((num) % val + val) % val;
  }
}
