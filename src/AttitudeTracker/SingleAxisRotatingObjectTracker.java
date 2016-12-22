/**
 * 
 */
package AttitudeTracker;

import org.ejml.data.DenseMatrix64F;

/**
 * @author John Walthour
 *
 */
public class SingleAxisRotatingObjectTracker {
	private KalmanFilterSimple filter = new KalmanFilterSimple();

	/**
	 * 
	 * @param momentOfInertia
	 */
	public SingleAxisRotatingObjectTracker() {
		// TODO: When you start incorporating the control model,
		// you may wish to take into account the vehicle's moment
		// of inertia along this axis. 
		// TODO: configure filter object
	}
	
	public void initTracker(double gyro, double compass) {
		// TODO
	}
	
	/**
	 * Register a new sensor reading.
	 * 
	 * @param gyro The angular rate reading, in radians per second
	 * @param compass The angular position reading, in radians
	 */
	public void updateSensorData(double dt, double gyro, double compass) {
		// TODO
	}

	public double getAngleEstimate() {
		// TODO
		return 0;
	}
}
