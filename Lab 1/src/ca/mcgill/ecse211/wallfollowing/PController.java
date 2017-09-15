package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;


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
    
    int delta = 0;
    int diff = distance - bandCenter;      

    if (diff > bandWidth) { //Too far
    	if (diff > 28) { //Really far
    		delta = 65 + (500 / diff);
    		
    		//Wider turn
    		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - (delta / 2));
        	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + delta);
        	WallFollowingLab.leftMotor.forward();
        	WallFollowingLab.rightMotor.forward();
        	return;
    	}
    	
    	delta = (diff * 3);
    }
    else if (diff < bandWidth){ //Too close
    	if (diff < -5) { //Really close
    		delta = -140 + diff;
    		
    		//Change direction in place
    		WallFollowingLab.leftMotor.setSpeed(delta);
        	WallFollowingLab.rightMotor.setSpeed(Math.abs(delta));
        	WallFollowingLab.leftMotor.forward();
        	WallFollowingLab.rightMotor.backward();
        	return;
    	}
    	
    	delta = (diff * 8);
    } 
    else { //Within band
    	delta = 0;
    }
    
	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - delta);
	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + delta);
	WallFollowingLab.leftMotor.forward();
	WallFollowingLab.rightMotor.forward();  
  }
  


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
