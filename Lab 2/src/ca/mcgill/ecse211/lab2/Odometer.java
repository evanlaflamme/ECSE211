package ca.mcgill.ecse211.lab2;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {  
  // robot position
  private double x;
  private double y;
  private double theta;
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private static final long ODOMETER_PERIOD = 25; /*odometer update period, in ms*/

  private Object lock; /*lock object for mutual exclusion*/

  // default constructor
  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.x = 0.0;
    this.y = 0.0;
    this.theta = 0.0;
    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
    lock = new Object();
  }

  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();
      
      //Get current tacho counts
      int nowLeftMotorTachoCount = leftMotor.getTachoCount();
      int nowRightMotorTachoCount = rightMotor.getTachoCount();
      
      //Get wheel displacements
      double distanceLeft = Math.PI * OdometryLab.WHEEL_RADIUS * (nowLeftMotorTachoCount - getLeftMotorTachoCount()) / 180;
      double distanceRight = Math.PI * OdometryLab.WHEEL_RADIUS * (nowRightMotorTachoCount - getRightMotorTachoCount()) / 180;
      
      //Save new tacho counts
      setLeftMotorTachoCount(nowLeftMotorTachoCount);
      setRightMotorTachoCount(nowRightMotorTachoCount);
      
      double deltaD = 0.5 * (distanceLeft + distanceRight); //Vehicle displacement
      double deltaT = (distanceLeft - distanceRight) / OdometryLab.TRACK; //Change in heading
      
      double deltaX = deltaD * Math.sin((getTheta() * Math.PI) / 180); //X component (convert theta to radians)
      double deltaY = deltaD * Math.cos((getTheta() * Math.PI) / 180); //Y component (convert theta to radians)
      
      deltaT = (deltaT * 180)/Math.PI; //Convert from radians to degrees

      synchronized (lock) {
        /**
         * Don't use the variables x, y, or theta anywhere but here! Only update the values of x, y,
         * and theta in this block. Do not perform complex math
         * 
         */
        
        setTheta(((getTheta() + deltaT) % 360 + 360) % 360); //Update heading (mod 360 to ensure it is always between 0 and 360)
        setX(getX() + deltaX); //Update estimates
        setY(getY() + deltaY);
      }

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometer will be interrupted by
          // another thread
        }
      }
    }
  }

  public void getPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        position[0] = x;
      if (update[1])
        position[1] = y;
      if (update[2])
        position[2] = theta;
    }
  }

  public double getX() {
    double result;

    synchronized (lock) {
      result = x;
    }

    return result;
  }

  public double getY() {
    double result;

    synchronized (lock) {
      result = y;
    }

    return result;
  }

  public double getTheta() {
    double result;

    synchronized (lock) {
      result = theta;
    }

    return result;
  }

  // mutators
  public void setPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        x = position[0];
      if (update[1])
        y = position[1];
      if (update[2])
        theta = position[2];
    }
  }

  public void setX(double x) {
    synchronized (lock) {
      this.x = x;
    }
  }

  public void setY(double y) {
    synchronized (lock) {
      this.y = y;
    }
  }

  public void setTheta(double theta) {
    synchronized (lock) {
      this.theta = theta;
    }
  }

  /**
   * @return the leftMotorTachoCount
   */
  public int getLeftMotorTachoCount() {
    return leftMotorTachoCount;
  }

  /**
   * @param leftMotorTachoCount the leftMotorTachoCount to set
   */
  public void setLeftMotorTachoCount(int leftMotorTachoCount) {
    synchronized (lock) {
      this.leftMotorTachoCount = leftMotorTachoCount;
    }
  }

  /**
   * @return the rightMotorTachoCount
   */
  public int getRightMotorTachoCount() {
    return rightMotorTachoCount;
  }

  /**
   * @param rightMotorTachoCount the rightMotorTachoCount to set
   */
  public void setRightMotorTachoCount(int rightMotorTachoCount) {
    synchronized (lock) {
      this.rightMotorTachoCount = rightMotorTachoCount;
    }
  }
}
