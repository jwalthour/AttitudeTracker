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

import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;

public class AttitudeTracker {
	private SimpleMatrix F = new SimpleMatrix(new DenseMatrix64F(0, 0)); // Model that predicts what the next x value will be, "a" or Φ in some documents
	private SimpleMatrix Q = new SimpleMatrix(new DenseMatrix64F(0, 0)); // Process noise covariance
	private SimpleMatrix H = new SimpleMatrix(new DenseMatrix64F(0, 0)); // "the noiseless connection between the state vector and the measurement vector"
	private SimpleMatrix x = new SimpleMatrix(new DenseMatrix64F(0, 0)); // The current state estimate, as [roll, pitch, yaw]
	private SimpleMatrix P = new SimpleMatrix(new DenseMatrix64F(0, 0)); // Error covariance matrix

	public AttitudeTracker() {
		
	}
	/**
	 * Provide the filter with sensor and control data with which to update the state estimate.
	 * After calling this function an updated attitude estimate will be available.
	 * 
	 * @param time_now What time it is now - used to measure the time between readings
	 * @param mag_x  - measured angular position in roll  axis
	 * @param mag_y  - measured angular position in pitch axis
	 * @param mag_z  - measured angular position in yaw   axis
	 * @param gyro_x - measured angular velocity in roll  axis
	 * @param gyro_y - measured angular velocity in pitch axis
	 * @param gyro_z - measured angular velocity in yaw   axis
	 * 
	 */
	public void updateDataTankDrive(double time_now, 
			double mag_x,  double mag_y,  double mag_z,
			double gyro_x, double gyro_y, double gyro_z,
			double left_wheel_speed, double right_wheel_speed) {
		
	}
	
	/**
	 * Provide the filter with sensor and control data with which to update the state estimate.
	 * After calling this function an updated attitude estimate will be available.
	 * 
	 * @param time_now What time it is now - used to measure the time between readings
	 * @param mag_x  - measured angular position in roll  axis
	 * @param mag_y  - measured angular position in pitch axis
	 * @param mag_z  - measured angular position in yaw   axis
	 * @param gyro_x - measured angular velocity in roll  axis
	 * @param gyro_y - measured angular velocity in pitch axis
	 * @param gyro_z - measured angular velocity in yaw   axis
	 * 
	 */
	public void updateSensorData(double time_now, 
			double mag_x,  double mag_y,  double mag_z,
			double gyro_x, double gyro_y, double gyro_z) {
		
	}
	
	/**
	 * Indicate that time has elapsed, without providing new sensor readings.
	 * This new time is used by the prediction logic.
	 * 
	 * @param time_now What time it is now
	 * 
	 */
	public void timeElapsed(double time_now) {
		
	}
}
