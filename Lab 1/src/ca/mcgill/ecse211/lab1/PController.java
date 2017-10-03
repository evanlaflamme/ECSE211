package ca.mcgill.ecse211.lab1;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 175;
  private static final int FILTER_OUT = 400;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
    // rudimentary filter - toss out invalid samples corresponding to null signal.
    // (n.b. this was not included in the Bang-bang controller, but easily could have).
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

    int diff = distance - bandCenter - 5;
    int deltaSpeed = 0;

    if (diff > bandWidth) { // Robot is too far from wall
      if (diff > 28) { // Robot is very far from wall
        deltaSpeed = 65 + (500 / diff);

        // Make wider turn
        WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - (deltaSpeed / 2));
        WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + deltaSpeed);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
        return;
      }

      deltaSpeed = (diff * 3);
    } else if (diff < bandWidth) { // Robot is too close to wall
      if (diff < -5) { // Robot is very close to wall
        deltaSpeed = -140 + diff;

        // Change direction in place
        WallFollowingLab.leftMotor.setSpeed(deltaSpeed);
        WallFollowingLab.rightMotor.setSpeed(Math.abs(deltaSpeed));
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.backward();
        return;
      }

      deltaSpeed = (diff * 8);
    } else { // Within band
      deltaSpeed = 0;
    }

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - deltaSpeed);
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + deltaSpeed);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }



  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
