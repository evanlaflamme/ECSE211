package ca.mcgill.ecse211.lab1;

public class BangBangController implements UltrasonicController {

  private static final int FILTER_OUT = 300;

  private final int bandCenter;
  private final int bandWidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  private int filterControl;

  public BangBangController(int bandCenter, int bandWidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandWidth = bandWidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
    if (distance >= 150 && filterControl < FILTER_OUT) { // Large value, increment filter control
      filterControl++;
    } else if (distance >= 150) { // Repeated large values, leave distance alone
      this.distance = distance;
    } else {
      // distance went below 150: reset filter and leave distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    int leftMotorSpeed = 0;
    int rightMotorSpeed = 0;

    if (distance < bandCenter - bandWidth) { // Robot is too close to wall
      if (distance < 30) { // Robot is really close to wall
        WallFollowingLab.leftMotor.setSpeed(motorHigh);
        WallFollowingLab.rightMotor.setSpeed(motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.backward(); // Reverse right wheel to turn faster
        return;
      }

      leftMotorSpeed = motorHigh;
      rightMotorSpeed = motorLow;
    } else if (distance > bandCenter + bandWidth) { // If too far from wall
      leftMotorSpeed = motorLow;
      rightMotorSpeed = motorHigh;
    } else { // Within range, so move forward
      leftMotorSpeed = motorHigh;
      rightMotorSpeed = motorHigh;
    }

    // Move robot forward
    WallFollowingLab.leftMotor.setSpeed(leftMotorSpeed);
    WallFollowingLab.rightMotor.setSpeed(rightMotorSpeed);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
