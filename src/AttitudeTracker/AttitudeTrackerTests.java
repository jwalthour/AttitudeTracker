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
//		testTankSteerWithControl(true);
		testKfWithRecordedData(true);
	}
	
	/**
	 * Simulate a tank-drive robot
	 * 
	 */
	public static void testTankSteerWithControl(boolean terminateAfter) {
		// Generate simulated sensor data for a robot that:
		/* - Sits still for 1s
		 * - Pivots right for 2s
		 * - Drives straight for 1s
		 * - Curves left for 3s
		 * - Drives straight back for 1s
		 * - Tries to pivot for 2s but the robot is held fixed in place
		 * - Drives straight forward for 5s
		 */
		// Shadow some of the default values
		final int NUM_DATAPOINTS = (int)((1+2+1+3+1+2+5) / DT);
		final double MAX_VEHICLE_PIVOT_RATE_RAD_PER_S = Math.PI;
		final double YAW_PROCESS_NOISE_MAGNITUDE_RAD = 5 * Math.PI / 180;
		final double YAW_RATE_PROCESS_NOISE_MAGNITUDE_RAD_S = 5 * Math.PI / 180;
		final double YAW_MEASUREMENT_NOISE_MAGNITUDE_RAD = 15 * Math.PI / 180;
		final double YAW_RATE_MEASUREMENT_NOISE_MAGNITUDE_RAD_PER_S = 10 * Math.PI / 180;

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
			boolean wheels_slipping = false;
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
			} else if(time[i] < 8.0) {
				// Curve left
				left_wheel_speed [i] =  0.0;
				right_wheel_speed[i] = +1.0;
			} else if(time[i] < 9.0) {
				// Straight back
				left_wheel_speed [i] = -1.0;
				right_wheel_speed[i] = -1.0;
			} else if(time[i] < 11.0) {
				// Get stuck trying to turn
				wheels_slipping = true;
				yaw_rate_ideal[i] = 0;
				left_wheel_speed [i] = -1.0;
				right_wheel_speed[i] = +1.0;
			} else if(time[i] < 16.0) {
				// Straight forward
				left_wheel_speed [i] = +1.0;
				right_wheel_speed[i] = +1.0;
			}
			
			// Compute turning equation
			if(!wheels_slipping) {
				yaw_rate_ideal[i] = MAX_VEHICLE_PIVOT_RATE_RAD_PER_S * (left_wheel_speed[i] - right_wheel_speed[i]) / 2;
			}
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
		KalmanFilterWithControl kf = new KalmanFilterWithControl();
		DenseMatrix64F F = new DenseMatrix64F(new double[][]{
			{1.0, DT },
			{0.0,0.9}, // base the yaw rate estimate entirely on the controls
		}); // The system dynamics model, S. Levy's tutorial calls this A
		DenseMatrix64F Q = new DenseMatrix64F(new double[][]{
			{YAW_PROCESS_NOISE_MAGNITUDE_RAD * 0.25, 0.0},
			{0.0, YAW_RATE_PROCESS_NOISE_MAGNITUDE_RAD_S * 0.25},
		}); // Noise covariance, must be estimated or ignored.
		DenseMatrix64F H = new DenseMatrix64F(new double[][]{
			{1.0, 0.0},
			{0.0, 1.0},
		}); // Maps observations to state variables - S. Levy's tutorial calls this C
		DenseMatrix64F B = new DenseMatrix64F(new double[][]{
			{0.0, 0.0},
			{0.1, -0.1}, // Effect on each state variable for each control input
		}); // Maps controls to state variables
		DenseMatrix64F R = new DenseMatrix64F(new double[][]{
			{YAW_MEASUREMENT_NOISE_MAGNITUDE_RAD * 0.5, 0.0},
			{0.0, YAW_RATE_MEASUREMENT_NOISE_MAGNITUDE_RAD_PER_S * 0.5},
		}); // Sensor value variance/covariance.  This is a fake estimate for now.
		kf.configure(F, Q, H, B);
		
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
			DenseMatrix64F u = new DenseMatrix64F(new double [][]{
				{left_wheel_speed [i]},
				{right_wheel_speed[i]},
			});
			kf.predict(u);
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

	public static void testKfWithRecordedData(boolean terminateAfter) {
		CsvDataPlayer player = new CsvDataPlayer("2017-1-13 IMU sensor data with corrections - Kovaka.csv", 6, 7, 5);
		FilteredCompassReader frc = new FilteredCompassReader();
		frc.setDataSources(player, player, player);
		
		
		final int NUM_DATAPOINTS = 7162;
		// t for Theta, the heading to magnetic north
		// w for lowercase omega, angular velocity
		double[] time        = new double[NUM_DATAPOINTS];
		double[] t_measured  = new double[NUM_DATAPOINTS];
		double[] t_estimated = new double[NUM_DATAPOINTS];
		double[] t_demod     = new double[NUM_DATAPOINTS];
		double   t_cum_err   = 0;
		double[] d_theta     = new double[NUM_DATAPOINTS];
		double[] w_measured  = new double[NUM_DATAPOINTS];
		double[] w_estimated = new double[NUM_DATAPOINTS];
		
		
		int i = 0;
		double heading_boost = 0;
		while(player.hasMoreData()) {
			t_measured[i] = player.getHeading();
			w_measured[i] = player.getW();
			if((i > 0) && (
					(t_measured[i] >  Math.PI / 2.0 && t_measured[i - 1] < -Math.PI / 2.0) ||
					(t_measured[i] < -Math.PI / 2.0 && t_measured[i - 1] >  Math.PI / 2.0)
					 )) {
				// Defunct the modulo
				heading_boost += 2 * Math.PI;
			}
			t_demod[i] = t_measured[i] + heading_boost;
			time[i] = player.getTime();
			// In this case we've been sampling the magnetometer faster than it updates.
			// The estimate has an update every timepoint, so there aren't zeroes most of
			// the samples.  The downfall is a 1-sample lag.
			d_theta[i] = i>1? t_estimated[i - 1] - t_estimated[i - 2] : 0;
//				d_theta[i] = i>0? t_demod[i] - t_demod[i - 1] : 0;
			
			// The above computes the change over one dt, while the units of w are in rad/s
			// We didn't measure the dt in the data capture.  So here we approximate dividing by that dt.
			d_theta[i] *= 250;
			
			DenseMatrix64F z = new DenseMatrix64F(new double [][]{
				{t_demod[i]},
				{d_theta[i]},
				{w_measured[i]},
			});
			frc.updateEstimate();
			t_estimated[i] = frc.getFilteredHeading();
			w_estimated[i] = frc.getFilteredAngularVelocity();

			// This should eventually add up to zero
			t_cum_err += (t_estimated[i] - t_demod[i]);

			player.advancePlayback();
			++i;
		}
		
		System.out.println("Cumulative error: " + t_cum_err);
	
		
		// Graph results
		XYSeries x_m_s = new XYSeries("Measured position");
		XYSeries x_e_s = new XYSeries("Estimated position");
		XYSeries d_m_s = new XYSeries("Change in position");
		XYSeries v_m_s = new XYSeries("Measured speed");
		XYSeries v_e_s = new XYSeries("Estimated speed");
		for(i = 0; i < NUM_DATAPOINTS; ++i) {
			x_m_s.add(time[i], t_demod[i]);
			x_e_s.add(time[i], t_estimated[i]);
			d_m_s.add(time[i], d_theta[i]);
			v_m_s.add(time[i], w_measured[i]);
			v_e_s.add(time[i], w_estimated[i]);
		}
		XYSeriesCollection sc = new XYSeriesCollection();
		sc.addSeries(x_e_s);
		sc.addSeries(x_m_s);
		sc.addSeries(d_m_s);
		sc.addSeries(v_e_s);
		sc.addSeries(v_m_s);
		
		JFreeChart chart = ChartFactory.createXYLineChart("KF test w/rec data", "Time (s)", "Value", sc);
		
		showChart(chart, terminateAfter);

	}

	public static void testScalarKfWithRecordedData(boolean terminateAfter) {
		CSVParser parser = null;
		try {
			parser = new CSVParser(new FileReader("2017-1-13 IMU sensor data with corrections - Kovaka.csv"), CSVFormat.DEFAULT);
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		final int NUM_DATAPOINTS = 7162;
		// t for Theta, the heading to magnetic north
		// w for lowercase omega, angular velocity
		double[] time        = new double[NUM_DATAPOINTS];
		double[] t_measured  = new double[NUM_DATAPOINTS];
		double[] t_estimated = new double[NUM_DATAPOINTS];
		double[] t_demod     = new double[NUM_DATAPOINTS];
		double[] w_measured  = new double[NUM_DATAPOINTS];
		double[] w_estimated = new double[NUM_DATAPOINTS];
		double[] k = new double[NUM_DATAPOINTS];
		double[] p = new double[NUM_DATAPOINTS];
		
		// Configure filters
		ScalarKalmanFilter pos_kf = new ScalarKalmanFilter();
//		ScalarKalmanFilter vel_kf = new ScalarKalmanFilter();
		pos_kf.configure(1, 0.05, 1);
		pos_kf.setState(2);		
//		vel_kf.setState(0);		
		
		int i = 0;
		try {
			// I deep-sixed the column headers, they were:
			// X,Y,Z,gX,gY,gZ,Hard Fe Y a=-36,Hard Fe Z ß=-221.5
			final int COL_CORR_MAG_Y = 6;
			final int COL_CORR_MAG_Z = 7;
			final int COL_GYRO_Z = 5;
			double heading_boost = 0.0;
			for(CSVRecord record : parser.getRecords()) {
				t_measured[i] = Math.atan2(Double.parseDouble(record.get(COL_CORR_MAG_Y)), Double.parseDouble(record.get(COL_CORR_MAG_Z)));
				w_measured[i] = Double.parseDouble(record.get(COL_GYRO_Z)) * Math.PI / 180.0;
				if((i > 0) && (
						(t_measured[i] >  Math.PI / 2.0 && t_measured[i - 1] < -Math.PI / 2.0) ||
						(t_measured[i] < -Math.PI / 2.0 && t_measured[i - 1] >  Math.PI / 2.0)
						 )) {
					// Defunct the modulo
					heading_boost += 2 * Math.PI;
				}
				t_demod[i] = t_measured[i] + heading_boost;
				time[i] = i * DT;
				
				pos_kf.predict();
//				vel_kf.predict();
				pos_kf.update(t_demod[i], 10);
//				vel_kf.update(w_measured[i], 0.01);
				
				t_estimated[i] = pos_kf.getState();
//				w_estimated[i] = vel_kf.getState();
				k[i] = pos_kf.getK();
				p[i] = pos_kf.getP();
				++i;
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	
		
		// Graph results
		XYSeries x_m_s = new XYSeries("Measured position");
		XYSeries x_e_s = new XYSeries("Estimated position");
		XYSeries k_s = new XYSeries("k");
		XYSeries p_s = new XYSeries("p");
//		XYSeries v_m_s = new XYSeries("Measured speed");
//		XYSeries v_e_s = new XYSeries("Estimated speed");
		for(i = 0; i < NUM_DATAPOINTS; ++i) {
			x_m_s.add(time[i], t_demod[i]);
			x_e_s.add(time[i], t_estimated[i]);
			k_s.add(time[i], k[i] * 10);
			p_s.add(time[i], p[i] * 10);
//			v_m_s.add(time[i], w_measured[i]);
//			v_e_s.add(time[i], w_estimated[i]);
		}
		XYSeriesCollection sc = new XYSeriesCollection();
		sc.addSeries(x_e_s);
		sc.addSeries(x_m_s);
		sc.addSeries(k_s);
		sc.addSeries(p_s);
//		sc.addSeries(v_e_s);
//		sc.addSeries(v_m_s);
		
		JFreeChart chart = ChartFactory.createXYLineChart("Scalar KF test", "Time (s)", "Value", sc);
		
		showChart(chart, terminateAfter);

	}
	
	public static void testAttitudeTrackerWithRecordedData(boolean terminateAfter) {

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
