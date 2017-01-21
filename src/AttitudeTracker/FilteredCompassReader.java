package AttitudeTracker;

import org.ejml.data.DenseMatrix64F;

public class FilteredCompassReader {
	KalmanFilterSimple kf = new KalmanFilterSimple();
	double lastUpdateTimestamp;
	boolean isFirstUpdate = true;
	double lastMeasuredHeading = 0.0;
	double lastEstimatedHeading = 0.0;
	double lastDTheta = 0.0;
	double lastEstimatedAngVel = 0.0;
	DenseMatrix64F R;
	double headingBoost = 0.0;
	
	FilteredCompassReader() {

		
		DenseMatrix64F F =  makeF(0.1); // TODO 
		DenseMatrix64F Q = new DenseMatrix64F(new double[][]{
			{0.05, 0.0},
			{0.0,  1.0},
		}); 
		DenseMatrix64F H = new DenseMatrix64F(new double[][]{
			{1.0, 0.0},
			{0.0, 1.0},
			{0.0, 1.0},
		}); 
		R = new DenseMatrix64F(new double[][]{
			{10.0, 0.0, 0.0},
			{ 0.0, 1.0, 0.0},
			{ 0.0, 0.0, 0.0},
		}); 
		kf.configure(F, Q, H);

		isFirstUpdate = true;
	}
	
	private DenseMatrix64F makeF(double dt) {
		return new DenseMatrix64F(new double[][]{
			{1.0, dt / 45},
			{0.0, 1.0},
		});
	}
	
	// Temporary function for testing outside the robot
	public double getHeading() {
		
		return 0.0;
	}
	
	public double getW() {
		return 0;
	}
	
//	public double get
	
	public void updateEstimate() {
		double w = 0;//gyro.getZAng();
		double theta = getHeading();
		double dt = 0;
		
		double now = 0;//Timer.getFPGATimestamp();
		if(!isFirstUpdate) 
		{ dt = now - lastUpdateTimestamp; }
		updateEstimate(theta, w, dt);
		
		lastUpdateTimestamp = now;
		isFirstUpdate = false;
	}
	
	private void updateEstimate(double theta_measured, double w_measured, double dt) {
		if(dt == 0.0) {
			// Initial measurement
			
		} else {
			// Normal update
			double theta_demod;
			if((theta_measured >  Math.PI / 2.0 && lastMeasuredHeading < -Math.PI / 2.0) ||
			   (theta_measured < -Math.PI / 2.0 && lastMeasuredHeading >  Math.PI / 2.0)) {
				// Defunct the modulo
				headingBoost += 2 * Math.PI;
			}
			theta_demod = theta_measured + headingBoost;
			DenseMatrix64F z = new DenseMatrix64F(new double [][]{
				{theta_demod},
				{lastDTheta},
				{w_measured},
			});
			kf.setF(makeF(dt));
			kf.predict();
			kf.update(null, R);
			double[] est = kf.getState().getData();
			lastDTheta = est[0] - lastEstimatedHeading;
			lastMeasuredHeading = theta_measured;
			lastEstimatedHeading = est[0];
			lastEstimatedAngVel  = est[1];
		}
	}
	
	public double getFilteredHeading() {
		return 0;
	}
	public double getFilteredAngularVelocity() {
		return 0;
	}

	void main() {
		System.out.println("Hello cruel world.");
	}
}
