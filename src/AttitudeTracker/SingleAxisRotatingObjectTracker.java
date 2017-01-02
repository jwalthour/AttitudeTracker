/**
 * MIT License
 * 
 * Copyright (c) 2016 John Walthour
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
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
