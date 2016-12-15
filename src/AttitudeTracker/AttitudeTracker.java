/* 
 * A Kalman filter designed for estimating the attitude (roll, pitch, yaw) of a FIRST sized robot.
 * 
 * Much of this code is credit Peter Abeles as example code for the EJM library at:
 * https://github.com/lessthanoptimal/ejm
 * 
 * Technically it would be sufficient to incorporate Peter's example as a member, but I feel this
 * more plagiaristic method 
 * 
 * */

package AttitudeTracker;

import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;

public class AttitudeTracker {
	private SimpleMatrix F; // Model that predicts what the next x value will be, "a" or Φ in some documents
	private SimpleMatrix Q; // Process noise covariance
	private SimpleMatrix H; // "the noiseless connection between the state vector and the measurement vector"
	private SimpleMatrix x; // The current state estimate, as [roll, pitch, yaw]
	private SimpleMatrix P; // Error covariance matrix

}
