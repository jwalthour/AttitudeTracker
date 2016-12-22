/* 
 * A Kalman filter designed for estimating the attitude (roll, pitch, yaw) of a FIRST sized robot.
 * 
 * Much of this code is credit Peter Abeles as example code for the EJM library at:
 * https://github.com/lessthanoptimal/ejm
 * 
 * Technically it would be sufficient to incorporate Peter's example as a member, but I feel this
 * more plagiaristic method is more effective.
 * 
 * This is meant to incorporate physical realities with the theoretical concepts of a Kalman filter.
 * 
 * */

package AttitudeTracker;

public class AttitudeTracker {
	private SingleAxisRotatingObjectTracker rollTracker  = new SingleAxisRotatingObjectTracker();
	private SingleAxisRotatingObjectTracker pitchTracker = new SingleAxisRotatingObjectTracker();
	private SingleAxisRotatingObjectTracker yawTracker   = new SingleAxisRotatingObjectTracker();
	private boolean haveFirstReading = false;
	private double timeNow;
	
	public AttitudeTracker() {
		// Currently all handled in member initializers
	}
	
	/**
	 * Provide the filter with sensor and control data with which to update the state estimate.
	 * After calling this function an updated attitude estimate will be available.
	 * 
	 * @param time_now in ms since epoch - used to measure the time between readings
	 * @param magX  measured angular position in roll  axis, in radians
	 * @param magY  measured angular position in pitch axis, in radians
	 * @param magZ  measured angular position in yaw   axis, in radians
	 * @param gyroX measured angular velocity in roll  axis, in radians per second
	 * @param gyroY measured angular velocity in pitch axis, in radians per second
	 * @param gyroZ measured angular velocity in yaw   axis, in radians per second
	 * 
	 */
	public void updateSensorData(double timeNow, 
			double magX,  double magY,  double magZ,
			double gyroX, double gyroY, double gyroZ) {
		// TODO
		if(haveFirstReading) {
			double dt = timeNow - this.timeNow;
			rollTracker .updateSensorData(dt, gyroX, magX);
			pitchTracker.updateSensorData(dt, gyroY, magY);
			yawTracker  .updateSensorData(dt, gyroZ, magZ);
			
		} else {
			rollTracker .initTracker(gyroX, magX);
			pitchTracker.initTracker(gyroY, magY);
			yawTracker  .initTracker(gyroZ, magZ);
			
			haveFirstReading = true;
		}
	}
}
