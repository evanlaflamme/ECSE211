package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 170;
  private static final int FILTER_OUT = 200;

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

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
    
    int delta;
    int diff = distance - bandCenter;

    // TODO: process a movement based on the us distance passed in (P style)
       

    if (Math.abs(diff) < 10) { //Not too far off
    	delta = diff / 2;
    }
    else if (diff > bandWidth) { //Too far
    	delta = (diff * 3);
    }
    else if (diff < bandWidth){ //Too close
    	delta = (diff * 13);
    } else { //Within band
    	delta = 0;
    }
    
    //Add difference (will be pos if too far, neg if too close)
    // set a threshold to avoid maximum motor output
    if (delta >= 70){ //Too far away
    	delta = 70;
    	
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - (delta / 2));
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + delta);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    else if(delta <= -210){ //Too close
    	delta = -210;
    	
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - delta);
    	WallFollowingLab.rightMotor.setSpeed(Math.abs(MOTOR_SPEED + delta));
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.backward();
    } else {
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - delta);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + delta);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }    
  }
  


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
