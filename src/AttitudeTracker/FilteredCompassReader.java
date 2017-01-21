package AttitudeTracker;

import org.ejml.data.DenseMatrix64F;

public class FilteredCompassReader {
	KalmanFilterSimple kf = new KalmanFilterSimple();
	HeadingProvider headingProvider = null;
	HeadingRateProvider headingRateProvider = null;
	TimeProvider timeProvider = null;
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
		
		DenseMatrix64F x_init = new DenseMatrix64F(new double [][]{
			{0.0},
			{0.0},
		}); // Start stationary
		DenseMatrix64F p_init = new DenseMatrix64F(new double [][]{
			{1, 0},
			{0, 1},
		}); // Arbitrary at the moment
		kf.setState(x_init, p_init);

		isFirstUpdate = true;
	}
	
	public void setDataSources(HeadingProvider hp, HeadingRateProvider hrp, TimeProvider tp) {
		headingProvider = hp;
		headingRateProvider = hrp;
		timeProvider = tp;
	}
	
	private DenseMatrix64F makeF(double dt) {
		return new DenseMatrix64F(new double[][]{
			{1.0, dt / 45},
			{0.0, 1.0},
		});
	}
	
	
	public void updateEstimate() {
		double w = headingRateProvider.getW();
		double theta = headingProvider.getHeading();
		double now = timeProvider.getTime();
		
		double dt = 0;
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
			kf.update(z, R);
			double[] est = kf.getState().getData();
			lastDTheta = est[0] - lastEstimatedHeading;
			lastMeasuredHeading = theta_measured;
			lastEstimatedHeading = est[0];
			lastEstimatedAngVel  = est[1];
		}
	}
	
	public double getFilteredHeading() {
		return lastEstimatedHeading;
	}
	public double getFilteredAngularVelocity() {
		return lastEstimatedAngVel;
	}
}
