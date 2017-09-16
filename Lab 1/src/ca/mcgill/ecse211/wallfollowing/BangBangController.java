package ca.mcgill.ecse211.wallfollowing;

public class BangBangController implements UltrasonicController {

  private static final int FILTER_OUT = 200;

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
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing there: leave the distance
      // alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    int leftMotorSpeed = 0;
    int rightMotorSpeed = 0;

    if (distance < bandCenter - bandWidth) { // Robot is too close to wall
      if (distance < 30) { // Robot is really close to wall
        WallFollowingLab.leftMotor.setSpeed((motorHigh * 5) / 2); // Increase turning speed
        WallFollowingLab.rightMotor.setSpeed(motorLow);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.backward(); // Reverse right wheel to turn faster
        return;
      }

      leftMotorSpeed = motorHigh;
      rightMotorSpeed = motorLow;
    } else if (distance > bandCenter + bandWidth) { // If too far from wall
      if (distance > 150) { // Really far from wall
        leftMotorSpeed = (motorLow * 4) / 3; // Turn faster
        rightMotorSpeed = motorHigh;
      } else {
        leftMotorSpeed = motorLow;
        rightMotorSpeed = motorHigh;
      }
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
