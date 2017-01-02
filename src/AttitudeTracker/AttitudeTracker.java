/* 
 * A Kalman filter class designed for estimating the attitude (roll, pitch, yaw) of a FIRST-class robot.
 * This is meant to be "turnkey", in that its inputs are sensor readings and its output is a
 * vehicle attitude estimate.  The package is designed with the FIRST robotics competition in mind,
 * but is meant to be generalized to ground vehicles.
 * 
 * The specific sensors expected are a combination of 3-axis gyro and digital magnetometer/compass.
 * 
 * This particular implementation suffers from gimbal lock.  That is, accuracy degrades dramatically
 * under extreme roll/pitch.  This isn't a huge issue for ground-based vehicles but is prohibitive 
 * for aerial and submarine vehicles.
 * 
 * The core KF was developed by Peter Abeles under the Apache License as example code for his EJML
 * matrix library.
 * 
 * The concepts of a Kalman Filter are discussed at length in the following resources.
 * I recommend reading them all at length prior to trying to understand the math herein.
 * 
 * Note that the difference between a Kalman Filter (KF) and an Extended Kalman Filter (EKF)
 * is that the EKF supports nonlinear state transition and observation models.  That is,
 * with the KF, F and H are matrices that multiply the state and control vectors.  Whereas
 * with the EKF, f() and h() are general functions of the state and control vectors.
 * 
 * https://home.wlu.edu/~levys/kalman_tutorial/ - a great introduction by Simon Levy.  He also
 * makes available implementations in JavaScript, C++, and Python.  If the equations aren't 
 * rendering correctly, download his PDF here: https://home.wlu.edu/~levys/kalman_tutorial/kalman.pdf
 * 
 * http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies - another, less complete take on the
 * same process.
 * 
 * http://ejml.org/wiki/index.php?title=Example_Kalman_Filter - Peter Abeles's page on the EKF.
 * 
 * https://en.wikipedia.org/wiki/Extended_Kalman_filter - the Wikipedia page provides an in-depth
 * mathematical background.
 * 
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
