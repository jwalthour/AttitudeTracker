package AttitudeTracker;

public class ScalarKalmanFilter {
	private double f = 1, q = 0, h = 1, b = 0;
	private double x, p = 1, k; // k is only here for debugging
	
	public void configure(double f, double q, double h, double b) {
		this.f = f;
		this.q = q;
		this.h = h;		
		this.b = b;
	}
	public void configure(double f, double q, double h) {
		configure(f, q, h, 0);
	}
	
	public void predict() {
		predict(0);
	}
	public void predict(double u) {
		x = f * x + b * u;
		p = f * p * f + q;
	}
	public void update(double z) {
		update(z, 0);
	}
	public void update(double z, double r) {
		double y = z - h * x;
		double s = h * p * h + r;
		
		k = p * h / s;
		x = x + k * y;
		p = (1 - h * k) * p;
	}
	
	public double getState() { return x; }
	public void   setState(double x) { this.x = x; }
	
	public double getP() { return p; }
	public double getK() { return k; }
}
