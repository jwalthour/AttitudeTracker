/*
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
 * 
 */
package AttitudeTracker;

import java.awt.Dimension;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import javax.swing.JDialog;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;
import org.ejml.data.DenseMatrix64F;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class AttitudeTrackerTests {
	public static final int NUM_DATAPOINTS = 10000;
	public static final double DT = 0.1; // seconds
	public static final double PROCESS_NOISE_MAGNITUDE = 10;
	public static final double X_MEASUREMENT_NOISE_MAGNITUDE = 2;
	public static final double V_MEASUREMENT_NOISE_MAGNITUDE = 0.000001;
	public static final double A_MEASUREMENT_NOISE_MAGNITUDE = 0.000001;
	public static final double PROCESS_RATE = 1-10.0/NUM_DATAPOINTS; // the only parameter of this fictional process

	public static void main(String[] args) {
//		testSimpleKfWithSyntheticData(false);
//		testSpeedKfWithSyntheticData(false);
//		testAccelKfWithSyntheticData(false);
//		testUnmeasuredAccelKfWithSyntheticData(true);
		testTankSteerWithoutControl(true);
	}
	
	/**
	 * Run a test on the KF class, using synthetic data,
	 * modeling a trivially simple movement process, and only
	 * tracking a single variable ("Position").
	 */
	public static void testSimpleKfWithSyntheticData(boolean terminateAfter) {
		// Generate synthetic data for a simple fictional movement process
		double[] time         = new double[NUM_DATAPOINTS];
		double[] x_ideal      = new double[NUM_DATAPOINTS];
		double[] x_actual     = new double[NUM_DATAPOINTS];
		double[] x_measured   = new double[NUM_DATAPOINTS];
		double[] x_estimated  = new double[NUM_DATAPOINTS];
		double x_cum_error = 0; // cumulative positional error = estimated - actual
		double x_cum_pred_error = 0; // cumulative positional error = estimated - actual
		time       [0] =   0.0;
		x_ideal    [0] = 100.0;
		x_actual   [0] = 100.0;
		x_measured [0] = 100.0;
		x_estimated[0] =   0.0;
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			time[i] = i * DT;
			x_ideal[i]    = x_ideal [i-1] * PROCESS_RATE;
			x_actual[i]   = x_ideal [i]   + PROCESS_NOISE_MAGNITUDE     * (Math.random() * 2 - 1);
			x_measured[i] = x_actual[i]   + X_MEASUREMENT_NOISE_MAGNITUDE * (Math.random() * 2 - 1);
		}
		
		// Configure filter
		KalmanFilterSimple kf = new KalmanFilterSimple();
		DenseMatrix64F F = new DenseMatrix64F(new double[][]{
			{0.9},
		}); // S. Levy's tutorial calls this A
		DenseMatrix64F Q = new DenseMatrix64F(new double[][]{
			{0.0},
		}); // S. Levy's tutorial lacks this term
		DenseMatrix64F H = new DenseMatrix64F(new double[][]{
			{1.0},
		}); // S. Levy's tutorial calls this C
		DenseMatrix64F R = new DenseMatrix64F(new double[][]{
			{0.75},
		}); // Expected magnitude of sensor noise
		kf.configure(F, Q, H);
		
		// Run filter
		DenseMatrix64F x_init = new DenseMatrix64F(new double [][]{
			{x_measured[0]}
		}); // Cheating a little here with an accurate initial estimate
		DenseMatrix64F p_init = new DenseMatrix64F(new double [][]{
			{1}
		}); // Arbitrary, cannot be 0
		kf.setState(x_init, p_init);
		
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			DenseMatrix64F z = new DenseMatrix64F(new double [][]{
				{x_measured[i]}
			});
			kf.predict();
			double x_prediction = kf.getState().data[0];
			kf.update(z, R);
			x_estimated[i] = kf.getState().data[0];
			double x_err = x_actual[i] - x_estimated[i]; 
			x_cum_error += x_err * x_err;
			double x_pred_err = x_actual[i] - x_prediction;
			x_cum_pred_error += x_pred_err * x_pred_err;
		}
		double x_stddev = Math.sqrt(x_cum_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double x_pred_stddev = Math.sqrt(x_cum_pred_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		System.out.println("Positional KF: Typical error: " + x_stddev + " typical deviation from prediction: " + x_pred_stddev);
		
		// Graph results
		XYSeries x_i_s = new XYSeries("Movement model");
		XYSeries x_a_s = new XYSeries("Actual position");
		XYSeries x_m_s = new XYSeries("Measured position");
		XYSeries x_e_s = new XYSeries("Estimated position");
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			x_i_s.add(time[i], x_ideal[i]);
			x_a_s.add(time[i], x_actual[i]);
			x_m_s.add(time[i], x_measured[i]);
			x_e_s.add(time[i], x_estimated[i]);
		}
		XYSeriesCollection sc = new XYSeriesCollection();
		sc.addSeries(x_e_s);
		sc.addSeries(x_m_s);
		sc.addSeries(x_a_s);
		sc.addSeries(x_i_s);
		
		JFreeChart chart = ChartFactory.createXYLineChart("Simple KF test", "Time (s)", "Value", sc);
		
		showChart(chart, terminateAfter);
	}
	
	/**
	 * Run a test on the KF class, using synthetic data,
	 * modeling a trivially simple movement process, tracking
	 * both position and velocity.
	 */
	public static void testSpeedKfWithSyntheticData(boolean terminateAfter) {
		// Generate synthetic data for a simple fictional movement process
		double[] time         = new double[NUM_DATAPOINTS];
		// X represents the position
		double[] x_ideal      = new double[NUM_DATAPOINTS];
		double[] x_actual     = new double[NUM_DATAPOINTS];
		double[] x_measured   = new double[NUM_DATAPOINTS];
		double[] x_estimated  = new double[NUM_DATAPOINTS];
		double   x_cum_error = 0; // cumulative error = estimated - actual
		double   x_cum_pred_error = 0; // cumulative predicted error = predicted - actual
		// V represents the velocity (derivative of position)
		double[] v_ideal      = new double[NUM_DATAPOINTS];
		double[] v_actual     = new double[NUM_DATAPOINTS];
		double[] v_measured   = new double[NUM_DATAPOINTS];
		double[] v_estimated  = new double[NUM_DATAPOINTS];
		double   v_cum_error = 0; // cumulative error = estimated - actual
		double   v_cum_pred_error = 0; // cumulative prediction error = predicted - actual
		time       [0] =   0.0;
		x_ideal    [0] = 100.0;
		x_actual   [0] = 100.0;
		x_measured [0] = 100.0;
		x_estimated[0] =   0.0;
		v_ideal    [0] =   0.0;
		v_actual   [0] = -50.0;
		v_measured [0] = -50.0;
		v_estimated[0] =   0.0;
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			time[i] = i * DT;
			x_ideal[i]    = x_ideal [i-1] * PROCESS_RATE;
			x_actual[i]   = x_ideal [i]   + PROCESS_NOISE_MAGNITUDE       * (Math.random() * 2 - 1);
			x_measured[i] = x_actual[i]   + X_MEASUREMENT_NOISE_MAGNITUDE * (Math.random() * 2 - 1);
			v_ideal[i]    = (x_ideal [i] - x_ideal [i-1]) / DT;
			v_actual[i]   = (x_actual[i] - x_actual[i-1]) / DT;
			v_measured[i] = v_actual[i]   + V_MEASUREMENT_NOISE_MAGNITUDE * (Math.random() * 2 - 1);
		}
		
		// Configure filter
		KalmanFilterSimple kf = new KalmanFilterSimple();
		DenseMatrix64F F = new DenseMatrix64F(new double[][]{
			{1.0, DT},
			{0.0, PROCESS_RATE},
		}); // The system dynamics model, S. Levy's tutorial calls this A
		DenseMatrix64F Q = new DenseMatrix64F(new double[][]{
			{PROCESS_NOISE_MAGNITUDE * 0.25, 0.0},
			{0.0, 0.0},
		}); // Noise covariance, must be estimated or ignored.  S. Levy's tutorial lacks this term, but it's added to the system dynamics model each predict step
		DenseMatrix64F H = new DenseMatrix64F(new double[][]{
			{1.0, 0.0},
			{0.0, 1.0},
		}); // Maps observations to state variables - S. Levy's tutorial calls this C
		DenseMatrix64F R = new DenseMatrix64F(new double[][]{
			{X_MEASUREMENT_NOISE_MAGNITUDE * 0.5, 0.0},
			{0.0, V_MEASUREMENT_NOISE_MAGNITUDE * 0.5},
		}); // Sensor value variance/covariance.  This is a fake estimate for now.
		kf.configure(F, Q, H);
		
		// Run filter
		DenseMatrix64F x_init = new DenseMatrix64F(new double [][]{
			{x_measured[0]},
			{v_measured[0]},
		}); // Cheating a little here with an accurate initial estimate
		DenseMatrix64F p_init = new DenseMatrix64F(new double [][]{
			{1, 0},
			{0, 1},
		}); // Arbitrary at the moment
		kf.setState(x_init, p_init);
		
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			DenseMatrix64F z = new DenseMatrix64F(new double [][]{
				{x_measured[i]},
				{v_measured[i]},
			});
			kf.predict();
			double x_prediction = kf.getState().data[0];
			double v_prediction = kf.getState().data[1];
			kf.update(z, R);
			x_estimated[i] = kf.getState().data[0];
			v_estimated[i] = kf.getState().data[1];
			double x_err = x_actual[i] - x_estimated[i]; 
			x_cum_error += x_err * x_err;
			double x_pred_err = x_actual[i] - x_prediction;
			x_cum_pred_error += x_pred_err * x_pred_err;
			double v_err = v_actual[i] - v_estimated[i]; 
			v_cum_error += v_err * v_err;
			double v_pred_err = v_actual[i] - v_prediction;
			v_cum_pred_error += v_pred_err * v_pred_err;
		}
		double x_stddev = Math.sqrt(x_cum_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double x_pred_stddev = Math.sqrt(x_cum_pred_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double v_stddev = Math.sqrt(v_cum_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double v_pred_stddev = Math.sqrt(v_cum_pred_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		System.out.println("Speed KF: Typical X error: " + x_stddev + " typical X deviation from prediction: " + x_pred_stddev);
		System.out.println("Speed KF: Typical V error: " + v_stddev + " typical V deviation from prediction: " + v_pred_stddev);
		
		// Graph results
		XYSeries x_i_s = new XYSeries("Model position");
		XYSeries x_a_s = new XYSeries("Actual position");
		XYSeries x_m_s = new XYSeries("Measured position");
		XYSeries x_e_s = new XYSeries("Estimated position");
		XYSeries v_i_s = new XYSeries("Model speed");
		XYSeries v_a_s = new XYSeries("Actual speed");
		XYSeries v_m_s = new XYSeries("Measured speed");
		XYSeries v_e_s = new XYSeries("Estimated speed");
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			x_i_s.add(time[i], x_ideal[i]);
			x_a_s.add(time[i], x_actual[i]);
			x_m_s.add(time[i], x_measured[i]);
			x_e_s.add(time[i], x_estimated[i]);
			v_i_s.add(time[i], v_ideal[i]);
			v_a_s.add(time[i], v_actual[i]);
			v_m_s.add(time[i], v_measured[i]);
			v_e_s.add(time[i], v_estimated[i]);
		}
		XYSeriesCollection sc = new XYSeriesCollection();
		sc.addSeries(x_e_s);
		sc.addSeries(x_m_s);
		sc.addSeries(x_a_s);
		sc.addSeries(x_i_s);
		sc.addSeries(v_e_s);
		sc.addSeries(v_m_s);
		sc.addSeries(v_a_s);
		sc.addSeries(v_i_s);
		
		JFreeChart chart = ChartFactory.createXYLineChart("Speed KF test", "Time (s)", "Value", sc);
		
		showChart(chart, terminateAfter);
	}


	/**
	 * Run a test on the KF class, using synthetic data,
	 * modeling a trivially simple movement process, tracking
	 * position, velocity, and acceleration.
	 */
	public static void testAccelKfWithSyntheticData(boolean terminateAfter) {
		// Generate synthetic data for a simple fictional movement process
		double[] time         = new double[NUM_DATAPOINTS];
		// X represents the position
		double[] x_ideal      = new double[NUM_DATAPOINTS];
		double[] x_actual     = new double[NUM_DATAPOINTS];
		double[] x_measured   = new double[NUM_DATAPOINTS];
		double[] x_estimated  = new double[NUM_DATAPOINTS];
		double   x_cum_error = 0; // cumulative error = estimated - actual
		double   x_cum_pred_error = 0; // cumulative predicted error = predicted - actual
		// V represents the velocity (derivative of position)
		double[] v_ideal      = new double[NUM_DATAPOINTS];
		double[] v_actual     = new double[NUM_DATAPOINTS];
		double[] v_measured   = new double[NUM_DATAPOINTS];
		double[] v_estimated  = new double[NUM_DATAPOINTS];
		double   v_cum_error = 0; // cumulative error = estimated - actual
		double   v_cum_pred_error = 0; // cumulative prediction error = predicted - actual
		// A represents the acceleration (derivative of velocity)
		double[] a_ideal      = new double[NUM_DATAPOINTS];
		double[] a_actual     = new double[NUM_DATAPOINTS];
		double[] a_measured   = new double[NUM_DATAPOINTS];
		double[] a_estimated  = new double[NUM_DATAPOINTS];
		double   a_cum_error = 0; // cumulative error = estimated - actual
		double   a_cum_pred_error = 0; // cumulative predicted error = predicted - actual
		time       [0] =   0.0;
		x_ideal    [0] = 100.0;
		x_actual   [0] = 100.0;
		x_measured [0] = 100.0;
		x_estimated[0] =   0.0;
		v_ideal    [0] =   0.0;
		v_actual   [0] = -50.0;
		v_measured [0] = -50.0;
		v_estimated[0] =   0.0;
		a_ideal    [0] =   0.0;
		a_actual   [0] =   0.0;
		a_measured [0] =   0.0;
		a_estimated[0] =   0.0;
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			time[i] = i * DT;
			x_ideal[i]    = x_ideal [i-1] * PROCESS_RATE;
			x_actual[i]   = x_ideal [i]   + PROCESS_NOISE_MAGNITUDE       * (Math.random() * 2 - 1);
			x_measured[i] = x_actual[i]   + X_MEASUREMENT_NOISE_MAGNITUDE * (Math.random() * 2 - 1);
			v_ideal[i]    = (x_ideal [i] - x_ideal [i-1]) / DT;
			v_actual[i]   = (x_actual[i] - x_actual[i-1]) / DT;
			v_measured[i] = v_actual[i]   + V_MEASUREMENT_NOISE_MAGNITUDE * (Math.random() * 2 - 1);
			a_ideal[i]    = (v_ideal [i] - v_ideal [i-1]) / DT;
			a_actual[i]   = (v_actual[i] - v_actual[i-1]) / DT;
			a_measured[i] = a_actual[i]   + A_MEASUREMENT_NOISE_MAGNITUDE * (Math.random() * 2 - 1);
		}
		
		// Configure filter
		KalmanFilterSimple kf = new KalmanFilterSimple();
		DenseMatrix64F F = new DenseMatrix64F(new double[][]{
			{1.0, DT, 0.0},
			{0.0, 1.0, DT},
			{0.0, 0.0, 1.0},
		}); // The system dynamics model, S. Levy's tutorial calls this A
		DenseMatrix64F Q = new DenseMatrix64F(new double[][]{
			{PROCESS_NOISE_MAGNITUDE * 0.25, 0.0, 0.0},
			{0.0, 0.0, 0.0},
			{0.0, 0.0, 0.0},
		}); // Noise covariance, must be estimated or ignored.  S. Levy's tutorial lacks this term, but it's added to the system dynamics model each predict step
		DenseMatrix64F H = new DenseMatrix64F(new double[][]{
			{1.0, 0.0, 0.0},
			{0.0, 1.0, 0.0},
			{0.0, 0.0, 1.0},
		}); // Maps observations to state variables - S. Levy's tutorial calls this C
		DenseMatrix64F R = new DenseMatrix64F(new double[][]{
			{X_MEASUREMENT_NOISE_MAGNITUDE * 0.5, 0.0, 0.0},
			{0.0, V_MEASUREMENT_NOISE_MAGNITUDE * 0.5, 0.0},
			{0.0, 0.0, A_MEASUREMENT_NOISE_MAGNITUDE * 0.5},
		}); // Sensor value variance/covariance.  This is a fake estimate for now.
		kf.configure(F, Q, H);
		
		// Run filter
		DenseMatrix64F x_init = new DenseMatrix64F(new double [][]{
			{x_measured[0]},
			{v_measured[0]},
			{a_measured[0]},
		}); // Cheating a little here with an accurate initial estimate
		DenseMatrix64F p_init = new DenseMatrix64F(new double [][]{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		}); // Arbitrary at the moment
		kf.setState(x_init, p_init);
		
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			DenseMatrix64F z = new DenseMatrix64F(new double [][]{
				{x_measured[i]},
				{v_measured[i]},
				{a_measured[i]},
			});
			kf.predict();
			double x_prediction = kf.getState().data[0];
			double v_prediction = kf.getState().data[1];
			double a_prediction = kf.getState().data[2];
			kf.update(z, R);
			x_estimated[i] = kf.getState().data[0];
			v_estimated[i] = kf.getState().data[1];
			a_estimated[i] = kf.getState().data[2];
			double x_err = x_actual[i] - x_estimated[i]; 
			x_cum_error += x_err * x_err;
			double x_pred_err = x_actual[i] - x_prediction;
			x_cum_pred_error += x_pred_err * x_pred_err;
			double v_err = v_actual[i] - v_estimated[i]; 
			v_cum_error += v_err * v_err;
			double v_pred_err = v_actual[i] - v_prediction;
			v_cum_pred_error += v_pred_err * v_pred_err;
			double a_err = a_actual[i] - a_estimated[i]; 
			a_cum_error += a_err * a_err;
			double a_pred_err = a_actual[i] - a_prediction;
			a_cum_pred_error += a_pred_err * a_pred_err;
		}
		double x_stddev = Math.sqrt(x_cum_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double x_pred_stddev = Math.sqrt(x_cum_pred_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double v_stddev = Math.sqrt(v_cum_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double v_pred_stddev = Math.sqrt(v_cum_pred_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double a_stddev = Math.sqrt(a_cum_error / NUM_DATAPOINTS); // not actually the standard deaiation but a similar computation
		double a_pred_stddev = Math.sqrt(a_cum_pred_error / NUM_DATAPOINTS); // not actually the standard deaiation but a similar computation
		System.out.println("Accel KF: Typical X error: " + x_stddev + " typical X deviation from prediction: " + x_pred_stddev);
		System.out.println("Accel KF: Typical V error: " + v_stddev + " typical V deviation from prediction: " + v_pred_stddev);
		System.out.println("Accel KF: Typical A error: " + a_stddev + " typical A deviation from prediction: " + a_pred_stddev);
		
		// Graph results
		XYSeries x_i_s = new XYSeries("Model position");
		XYSeries x_a_s = new XYSeries("Actual position");
		XYSeries x_m_s = new XYSeries("Measured position");
		XYSeries x_e_s = new XYSeries("Estimated position");
		XYSeries v_i_s = new XYSeries("Model speed");
		XYSeries v_a_s = new XYSeries("Actual speed");
		XYSeries v_m_s = new XYSeries("Measured speed");
		XYSeries v_e_s = new XYSeries("Estimated speed");
		XYSeries a_i_s = new XYSeries("Model accel");
		XYSeries a_a_s = new XYSeries("Actual accel");
		XYSeries a_m_s = new XYSeries("Measured accel");
		XYSeries a_e_s = new XYSeries("Estimated accel");
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			x_i_s.add(time[i], x_ideal[i]);
			x_a_s.add(time[i], x_actual[i]);
			x_m_s.add(time[i], x_measured[i]);
			x_e_s.add(time[i], x_estimated[i]);
			v_i_s.add(time[i], v_ideal[i]);
			v_a_s.add(time[i], v_actual[i]);
			v_m_s.add(time[i], v_measured[i]);
			v_e_s.add(time[i], v_estimated[i]);
			a_i_s.add(time[i], a_ideal[i]);
			a_a_s.add(time[i], a_actual[i]);
			a_m_s.add(time[i], a_measured[i]);
			a_e_s.add(time[i], a_estimated[i]);
		}
		XYSeriesCollection sc = new XYSeriesCollection();
		sc.addSeries(x_e_s);
		sc.addSeries(x_m_s);
		sc.addSeries(x_a_s);
		sc.addSeries(x_i_s);
		sc.addSeries(v_e_s);
		sc.addSeries(v_m_s);
		sc.addSeries(v_a_s);
		sc.addSeries(v_i_s);
		sc.addSeries(a_e_s);
		sc.addSeries(a_m_s);
		sc.addSeries(a_a_s);
		sc.addSeries(a_i_s);
		
		JFreeChart chart = ChartFactory.createXYLineChart("Accel KF test", "Time (s)", "Value", sc);
		
		showChart(chart, terminateAfter);
	}



	/**
	 * Run a test on the KF class, using synthetic data,
	 * modeling a trivially simple movement process, tracking
	 * position, velocity, and acceleration.
	 * 
	 * However, this one tracks acceleration without any sensor input representing acceleration.
	 */
	public static void testUnmeasuredAccelKfWithSyntheticData(boolean terminateAfter) {
		// Generate synthetic data for a simple fictional movement process
		double[] time         = new double[NUM_DATAPOINTS];
		// X represents the position
		double[] x_ideal      = new double[NUM_DATAPOINTS];
		double[] x_actual     = new double[NUM_DATAPOINTS];
		double[] x_measured   = new double[NUM_DATAPOINTS];
		double[] x_estimated  = new double[NUM_DATAPOINTS];
		double   x_cum_error = 0; // cumulative error = estimated - actual
		double   x_cum_pred_error = 0; // cumulative predicted error = predicted - actual
		// V represents the velocity (derivative of position)
		double[] v_ideal      = new double[NUM_DATAPOINTS];
		double[] v_actual     = new double[NUM_DATAPOINTS];
		double[] v_measured   = new double[NUM_DATAPOINTS];
		double[] v_estimated  = new double[NUM_DATAPOINTS];
		double   v_cum_error = 0; // cumulative error = estimated - actual
		double   v_cum_pred_error = 0; // cumulative prediction error = predicted - actual
		// A represents the acceleration (derivative of velocity)
		double[] a_ideal      = new double[NUM_DATAPOINTS];
		double[] a_actual     = new double[NUM_DATAPOINTS];
		double[] a_estimated  = new double[NUM_DATAPOINTS];
		double   a_cum_error = 0; // cumulative error = estimated - actual
		double   a_cum_pred_error = 0; // cumulative predicted error = predicted - actual
		time       [0] =   0.0;
		x_ideal    [0] = 100.0;
		x_actual   [0] = 100.0;
		x_measured [0] = 100.0;
		x_estimated[0] =   0.0;
		v_ideal    [0] =   0.0;
		v_actual   [0] = -50.0;
		v_measured [0] = -50.0;
		v_estimated[0] =   0.0;
		a_ideal    [0] =   0.0;
		a_actual   [0] =   0.0;
		a_estimated[0] =   0.0;
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			time[i] = i * DT;
			x_ideal[i]    = x_ideal [i-1] * PROCESS_RATE;
			x_actual[i]   = x_ideal [i]   + PROCESS_NOISE_MAGNITUDE       * (Math.random() * 2 - 1);
			x_measured[i] = x_actual[i]   + X_MEASUREMENT_NOISE_MAGNITUDE * (Math.random() * 2 - 1);
			v_ideal[i]    = (x_ideal [i] - x_ideal [i-1]) / DT;
			v_actual[i]   = (x_actual[i] - x_actual[i-1]) / DT;
			v_measured[i] = v_actual[i]   + V_MEASUREMENT_NOISE_MAGNITUDE * (Math.random() * 2 - 1);
			a_ideal[i]    = (v_ideal [i] - v_ideal [i-1]) / DT;
			a_actual[i]   = (v_actual[i] - v_actual[i-1]) / DT;
		}
		
		// Configure filter
		KalmanFilterSimple kf = new KalmanFilterSimple();
		DenseMatrix64F F = new DenseMatrix64F(new double[][]{
			{1.0, DT, 0.0},
			{0.0, 1.0, DT},
			{0.0, 0.0, 1.0},
		}); // The system dynamics model, S. Levy's tutorial calls this A
		DenseMatrix64F Q = new DenseMatrix64F(new double[][]{
			{PROCESS_NOISE_MAGNITUDE * 0.25, 0.0, 0.0},
			{0.0, 0.0, 0.0},
			{0.0, 0.0, 0.0},
		}); // Noise covariance, must be estimated or ignored.  S. Levy's tutorial lacks this term, but it's added to the system dynamics model each predict step
		DenseMatrix64F H = new DenseMatrix64F(new double[][]{
			{1.0, 0.0, 0.0},
			{0.0, 1.0, 0.0},
		}); // Maps observations to state variables - S. Levy's tutorial calls this C
		DenseMatrix64F R = new DenseMatrix64F(new double[][]{
			{X_MEASUREMENT_NOISE_MAGNITUDE * 0.5, 0.0},
			{0.0, V_MEASUREMENT_NOISE_MAGNITUDE * 0.5},
		}); // Sensor value variance/covariance.  This is a fake estimate for now.
		kf.configure(F, Q, H);
		
		// Run filter
		DenseMatrix64F x_init = new DenseMatrix64F(new double [][]{
			{x_measured[0]},
			{v_measured[0]},
			{0.0},
		}); // Cheating a little here with an accurate initial estimate
		DenseMatrix64F p_init = new DenseMatrix64F(new double [][]{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		}); // Arbitrary at the moment
		kf.setState(x_init, p_init);
		
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			DenseMatrix64F z = new DenseMatrix64F(new double [][]{
				{x_measured[i]},
				{v_measured[i]},
			});
			kf.predict();
			double x_prediction = kf.getState().data[0];
			double v_prediction = kf.getState().data[1];
			double a_prediction = kf.getState().data[2];
			kf.update(z, R);
			x_estimated[i] = kf.getState().data[0];
			v_estimated[i] = kf.getState().data[1];
			a_estimated[i] = kf.getState().data[2];
			double x_err = x_actual[i] - x_estimated[i]; 
			x_cum_error += x_err * x_err;
			double x_pred_err = x_actual[i] - x_prediction;
			x_cum_pred_error += x_pred_err * x_pred_err;
			double v_err = v_actual[i] - v_estimated[i]; 
			v_cum_error += v_err * v_err;
			double v_pred_err = v_actual[i] - v_prediction;
			v_cum_pred_error += v_pred_err * v_pred_err;
			double a_err = a_actual[i] - a_estimated[i]; 
			a_cum_error += a_err * a_err;
			double a_pred_err = a_actual[i] - a_prediction;
			a_cum_pred_error += a_pred_err * a_pred_err;
		}
		double x_stddev = Math.sqrt(x_cum_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double x_pred_stddev = Math.sqrt(x_cum_pred_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double v_stddev = Math.sqrt(v_cum_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double v_pred_stddev = Math.sqrt(v_cum_pred_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double a_stddev = Math.sqrt(a_cum_error / NUM_DATAPOINTS); // not actually the standard deaiation but a similar computation
		double a_pred_stddev = Math.sqrt(a_cum_pred_error / NUM_DATAPOINTS); // not actually the standard deaiation but a similar computation
		System.out.println("Unmeasured Accel KF: Typical X error: " + x_stddev + " typical X deviation from prediction: " + x_pred_stddev);
		System.out.println("Unmeasured Accel KF: Typical V error: " + v_stddev + " typical V deviation from prediction: " + v_pred_stddev);
		System.out.println("Unmeasured Accel KF: Typical A error: " + a_stddev + " typical A deviation from prediction: " + a_pred_stddev);
		
		// Graph results
		XYSeries x_i_s = new XYSeries("Model position");
		XYSeries x_a_s = new XYSeries("Actual position");
		XYSeries x_m_s = new XYSeries("Measured position");
		XYSeries x_e_s = new XYSeries("Estimated position");
		XYSeries v_i_s = new XYSeries("Model speed");
		XYSeries v_a_s = new XYSeries("Actual speed");
		XYSeries v_m_s = new XYSeries("Measured speed");
		XYSeries v_e_s = new XYSeries("Estimated speed");
		XYSeries a_i_s = new XYSeries("Model accel");
		XYSeries a_a_s = new XYSeries("Actual accel");
		XYSeries a_e_s = new XYSeries("Estimated accel");
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			x_i_s.add(time[i], x_ideal[i]);
			x_a_s.add(time[i], x_actual[i]);
			x_m_s.add(time[i], x_measured[i]);
			x_e_s.add(time[i], x_estimated[i]);
			v_i_s.add(time[i], v_ideal[i]);
			v_a_s.add(time[i], v_actual[i]);
			v_m_s.add(time[i], v_measured[i]);
			v_e_s.add(time[i], v_estimated[i]);
			a_i_s.add(time[i], a_ideal[i]);
			a_a_s.add(time[i], a_actual[i]);
			a_e_s.add(time[i], a_estimated[i]);
		}
		XYSeriesCollection sc = new XYSeriesCollection();
		sc.addSeries(x_e_s);
		sc.addSeries(x_m_s);
		sc.addSeries(x_a_s);
		sc.addSeries(x_i_s);
		sc.addSeries(v_e_s);
		sc.addSeries(v_m_s);
		sc.addSeries(v_a_s);
		sc.addSeries(v_i_s);
		sc.addSeries(a_e_s);
		sc.addSeries(a_a_s);
		sc.addSeries(a_i_s);
		
		JFreeChart chart = ChartFactory.createXYLineChart("Accel KF test without measurement input", "Time (s)", "Value", sc);
		
		showChart(chart, terminateAfter);
	}

	/**
	 * Simulate a tank-drive robot
	 * 
	 */
	public static void testTankSteerWithoutControl(boolean terminateAfter) {
		// Generate simulated sensor data for a robot that:
		/* - Sits still for 1s
		 * - Pivots right for 2s
		 * - Drives straight for 1s
		 * - Curves left for 3s
		 */
		// Shadow some of the default values
		final int NUM_DATAPOINTS = (int)((1+2+1+3) / DT);
		final double MAX_VEHICLE_PIVOT_RATE_RAD_PER_S = Math.PI;
		final double YAW_PROCESS_NOISE_MAGNITUDE_RAD = 5 * Math.PI / 180;
		final double YAW_RATE_PROCESS_NOISE_MAGNITUDE_RAD_S = 5 * Math.PI / 180;
		final double YAW_MEASUREMENT_NOISE_MAGNITUDE_RAD = 15 * Math.PI / 180;
		final double YAW_RATE_MEASUREMENT_NOISE_MAGNITUDE_RAD_PER_S = 1 * Math.PI / 180;

		double[] time          = new double[NUM_DATAPOINTS];
		// Control input
		double[] left_wheel_speed  = new double[NUM_DATAPOINTS];
		double[] right_wheel_speed = new double[NUM_DATAPOINTS];
		// Position
		double[] yaw_ideal     = new double[NUM_DATAPOINTS];
		double[] yaw_actual    = new double[NUM_DATAPOINTS];
		double[] yaw_measured  = new double[NUM_DATAPOINTS];
		double[] yaw_estimated = new double[NUM_DATAPOINTS];
		double   yaw_cum_error = 0; // cumulative error = estimated - actual
		double   yaw_cum_pred_error = 0; // cumulative predicted error = predicted - actual
		// Rate - d(yaw)/dt
		double[] yaw_rate_ideal     = new double[NUM_DATAPOINTS];
		double[] yaw_rate_actual    = new double[NUM_DATAPOINTS];
		double[] yaw_rate_measured  = new double[NUM_DATAPOINTS];
		double[] yaw_rate_estimated = new double[NUM_DATAPOINTS];
		double   yaw_rate_cum_error = 0; // cumulative error = estimated - actual
		double   yaw_rate_cum_pred_error = 0; // cumulative predicted error = predicted - actual

		for(int i = 0; i < NUM_DATAPOINTS; ++i) {
			time[i] = i * DT;
			boolean moving = true;
			if(time[i] < 1.0) {
				// Sit still to start out
				moving = false;
				left_wheel_speed [i] = 0.0;
				right_wheel_speed[i] = 0.0;
			} else if(time[i] < 3.0) {
				// Pivot right
				left_wheel_speed [i] = +1.0;
				right_wheel_speed[i] = -1.0;
				
			} else if(time[i] < 5.0) {
				// Go straight
				left_wheel_speed [i] = +1.0;
				right_wheel_speed[i] = +1.0;
			} else  {
				// Curve left
				left_wheel_speed [i] =  0.0;
				right_wheel_speed[i] = +1.0;
			}
			
			// Compute turning equation
			yaw_rate_ideal[i] = MAX_VEHICLE_PIVOT_RATE_RAD_PER_S * (left_wheel_speed[i] - right_wheel_speed[i]) / 2;
			double yaw = (i == 0)? 0 : yaw_ideal[i - 1];
			yaw_ideal[i] = yaw + yaw_rate_ideal[i] * DT;
			
			// There's no wobbling when the vehicle's stationary
			yaw_actual[i]      = yaw_ideal[i];
			yaw_rate_actual[i] = yaw_rate_ideal[i];
			if(moving) {
				yaw_actual[i]      += (2 * Math.random() - 1) * YAW_PROCESS_NOISE_MAGNITUDE_RAD;
				yaw_rate_actual[i] += (2 * Math.random() - 1) * YAW_RATE_PROCESS_NOISE_MAGNITUDE_RAD_S;
			}
			
			// There's always sensor noise
			yaw_measured[i]      = yaw_actual[i]      + (2 * Math.random() - 1) * YAW_MEASUREMENT_NOISE_MAGNITUDE_RAD;
			yaw_rate_measured[i] = yaw_rate_actual[i] + (2 * Math.random() - 1) * YAW_RATE_MEASUREMENT_NOISE_MAGNITUDE_RAD_PER_S;
		}
		
		// Configure filter
		KalmanFilterSimple kf = new KalmanFilterSimple();
		DenseMatrix64F F = new DenseMatrix64F(new double[][]{
			{1.0, DT },
			{0.0, 1.0},
		}); // The system dynamics model, S. Levy's tutorial calls this A
		DenseMatrix64F Q = new DenseMatrix64F(new double[][]{
			{YAW_PROCESS_NOISE_MAGNITUDE_RAD * 0.25, 0.0},
			{0.0, YAW_RATE_PROCESS_NOISE_MAGNITUDE_RAD_S * 0.25},
		}); // Noise covariance, must be estimated or ignored.  S. Levy's tutorial lacks this term, but it's added to the system dynamics model each predict step
		DenseMatrix64F H = new DenseMatrix64F(new double[][]{
			{1.0, 0.0},
			{0.0, 1.0},
		}); // Maps observations to state variables - S. Levy's tutorial calls this C
		DenseMatrix64F R = new DenseMatrix64F(new double[][]{
			{YAW_MEASUREMENT_NOISE_MAGNITUDE_RAD * 0.5, 0.0},
			{0.0, YAW_RATE_MEASUREMENT_NOISE_MAGNITUDE_RAD_PER_S * 0.5},
		}); // Sensor value variance/covariance.  This is a fake estimate for now.
		kf.configure(F, Q, H);
		
		// Run filter
		DenseMatrix64F x_init = new DenseMatrix64F(new double [][]{
			{0.0},
			{0.0},
		}); // Start stationary
		DenseMatrix64F p_init = new DenseMatrix64F(new double [][]{
			{1, 0},
			{0, 1},
		}); // Arbitrary at the moment
		kf.setState(x_init, p_init);
		
		for(int i = 0; i < NUM_DATAPOINTS; ++i) {
			DenseMatrix64F z = new DenseMatrix64F(new double [][]{
				{yaw_measured[i]},
				{yaw_rate_measured[i]},
			});
			kf.predict();
			double yaw_prediction = kf.getState().data[0];
			double yaw_rate_prediction = kf.getState().data[1];
			kf.update(z, R);
			yaw_estimated[i] = kf.getState().data[0];
			yaw_rate_estimated[i] = kf.getState().data[1];
			double yaw_err = yaw_actual[i] - yaw_estimated[i]; 
			yaw_cum_error += yaw_err * yaw_err;
			double yaw_pred_err = yaw_actual[i] - yaw_prediction;
			yaw_cum_pred_error += yaw_pred_err * yaw_pred_err;
			double yaw_rate_err = yaw_rate_actual[i] - yaw_rate_estimated[i]; 
			yaw_rate_cum_error += yaw_rate_err * yaw_rate_err;
			double yaw_rate_pred_err = yaw_rate_actual[i] - yaw_rate_prediction;
			yaw_rate_cum_pred_error += yaw_rate_pred_err * yaw_rate_pred_err;
			
		}
		double yaw_stddev = Math.sqrt(yaw_cum_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double yaw_pred_stddev = Math.sqrt(yaw_cum_pred_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double yaw_rate_stddev = Math.sqrt(yaw_rate_cum_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		double yaw_rate_pred_stddev = Math.sqrt(yaw_rate_cum_pred_error / NUM_DATAPOINTS); // not actually the standard deviation but a similar computation
		System.out.println("Tank steer KF: Typical yaw error: " + yaw_stddev * 180 / Math.PI + "deg, typical yaw deviation from prediction: " + yaw_pred_stddev * 180 / Math.PI);
		System.out.println("Tank steer KF: Typical yaw rate error: " + yaw_rate_stddev * 180 / Math.PI + "deg, typical yaw rate deviation from prediction: " + yaw_rate_pred_stddev * 180 / Math.PI);
		
		// Graph results
		XYSeries x_i_s = new XYSeries("Model yaw");
		XYSeries x_a_s = new XYSeries("Actual yaw");
		XYSeries x_m_s = new XYSeries("Measured yaw");
		XYSeries x_e_s = new XYSeries("Estimated yaw");
		XYSeries v_i_s = new XYSeries("Model yaw rate");
		XYSeries v_a_s = new XYSeries("Actual yaw rate");
		XYSeries v_m_s = new XYSeries("Measured yaw rate");
		XYSeries v_e_s = new XYSeries("Estimated yaw rate");
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			x_i_s.add(time[i], yaw_ideal[i]);
			x_a_s.add(time[i], yaw_actual[i]);
			x_m_s.add(time[i], yaw_measured[i]);
			x_e_s.add(time[i], yaw_estimated[i]);
			v_i_s.add(time[i], yaw_rate_ideal[i]);
			v_a_s.add(time[i], yaw_rate_actual[i]);
			v_m_s.add(time[i], yaw_rate_measured[i]);
			v_e_s.add(time[i], yaw_rate_estimated[i]);
		}
		XYSeriesCollection sc = new XYSeriesCollection();
		sc.addSeries(x_e_s);
		sc.addSeries(x_m_s);
		sc.addSeries(x_a_s);
		sc.addSeries(x_i_s);
		sc.addSeries(v_e_s);
		sc.addSeries(v_m_s);
		sc.addSeries(v_a_s);
		sc.addSeries(v_i_s);
		
		JFreeChart chart = ChartFactory.createXYLineChart("Tank steer KF", "Time (s)", "Yaw (radians) or Yaw Rate (rad/s)", sc);
		
		showChart(chart, terminateAfter);
	}

	
	/**
	 * Display a chart in a dialog window
	 * @param chart
	 * @param terminateAfter
	 */
	private static void showChart(JFreeChart chart, boolean terminateAfter) {
		ChartPanel panel = new ChartPanel(chart);
        panel.setFillZoomRectangle(true);
        panel.setMouseWheelEnabled(true);
        JDialog dialog = new JDialog();
        if(terminateAfter) {
	        dialog.addWindowListener(new WindowListener() {
				@Override
				public void windowOpened(WindowEvent e) {}
				@Override
				public void windowIconified(WindowEvent e) {}
				@Override
				public void windowDeiconified(WindowEvent e) {}
				@Override
				public void windowDeactivated(WindowEvent e) {}
				@Override
				public void windowClosing(WindowEvent e) { System.exit(0);}
				@Override
				public void windowClosed(WindowEvent e) {}
				@Override
				public void windowActivated(WindowEvent e) {}
			});
        }
        dialog.setDefaultCloseOperation(JDialog.DISPOSE_ON_CLOSE);
        dialog.add(panel);
        dialog.pack();
        dialog.setVisible(true);        
	}

	public static void testAttitudeTrackerWithSyntheticData() {
		
	
	}
	public static void testAttitudeTrackerWithRecordedData() {

		CSVParser parser;
		try {
			parser = new CSVParser(new FileReader("test_data_spinning.csv"), CSVFormat.DEFAULT);
			for(CSVRecord record : parser.getRecords()) {
				System.out.println("Got a record");
			}
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
