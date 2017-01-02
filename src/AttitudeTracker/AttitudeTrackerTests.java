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

	public static void main(String[] args) {
//		testSimpleKfWithSyntheticData(false);
		testSpeedKfWithSyntheticData(true);
	}
	
	/**
	 * Run a test on the KF class, using synthetic data,
	 * modeling a trivially simple movement process, and only
	 * tracking a single variable ("Position").
	 */
	public static void testSimpleKfWithSyntheticData(boolean terminateAfter) {
		// Generate synthetic data for a simple fictional movement process
		final int NUM_DATAPOINTS = 100;
		final double DT = 0.2; // seconds
		final double PROCESS_NOISE_MAGNITUDE = 5;
		final double MEASUREMENT_NOISE_MAGNITUDE = 10;
		final double PROCESS_RATE = 0.9; // the only parameter of this fictional process
		double[] time         = new double[NUM_DATAPOINTS];
		double[] x_ideal      = new double[NUM_DATAPOINTS];
		double[] x_actual     = new double[NUM_DATAPOINTS];
		double[] x_measured   = new double[NUM_DATAPOINTS];
		double[] x_estimated  = new double[NUM_DATAPOINTS];
		time       [0] =   0.0;
		x_ideal    [0] = 100.0;
		x_actual   [0] = 100.0;
		x_measured [0] = 100.0;
		x_estimated[0] =   0.0;
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			time[i] = i * DT;
			x_ideal[i]    = x_ideal [i-1] * PROCESS_RATE;
			x_actual[i]   = x_ideal [i]   + PROCESS_NOISE_MAGNITUDE     * (Math.random() * 2 - 1);
			x_measured[i] = x_actual[i]   + MEASUREMENT_NOISE_MAGNITUDE * (Math.random() * 2 - 1);
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
			kf.update(z, R);
			x_estimated[i] = kf.getState().data[0];
		}
		
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
		
		JFreeChart chart = ChartFactory.createXYLineChart("Simple kf test", "Time (s)", "Value", sc);
		
		showChart(chart, terminateAfter);
	}
	/**
	 * Run a test on the KF class, using synthetic data,
	 * modeling a trivially simple movement process, tracking
	 * both position and velocity.
	 */
	public static void testSpeedKfWithSyntheticData(boolean terminateAfter) {
		// Generate synthetic data for a simple fictional movement process
		final int NUM_DATAPOINTS = 100;
		final double DT = 0.2; // seconds
		final double PROCESS_NOISE_MAGNITUDE = 0.001;
		final double X_MEASUREMENT_NOISE_MAGNITUDE = 10;
		final double V_MEASUREMENT_NOISE_MAGNITUDE = 10;
		final double PROCESS_RATE = 0.9; // the only parameter of this fictional process
		double[] time         = new double[NUM_DATAPOINTS];
		// X represents the position
		double[] x_ideal      = new double[NUM_DATAPOINTS];
		double[] x_actual     = new double[NUM_DATAPOINTS];
		double[] x_measured   = new double[NUM_DATAPOINTS];
		double[] x_estimated  = new double[NUM_DATAPOINTS];
		// V represents the velocity (derivative of position)
		double[] v_ideal      = new double[NUM_DATAPOINTS];
		double[] v_actual     = new double[NUM_DATAPOINTS];
		double[] v_measured   = new double[NUM_DATAPOINTS];
		double[] v_estimated  = new double[NUM_DATAPOINTS];
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
			{0.0, 0.0},
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
			{1, 1},
			{1, 1},
		}); // Arbitrary at the moment
		kf.setState(x_init, p_init);
		
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			DenseMatrix64F z = new DenseMatrix64F(new double [][]{
				{x_measured[i]},
				{v_measured[i]},
			});
			kf.predict();
			kf.update(z, R);
			x_estimated[i] = kf.getState().data[0];
			v_estimated[i] = kf.getState().data[1];
		}
		
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
		
		JFreeChart chart = ChartFactory.createXYLineChart("Speed kf test", "Time (s)", "Value", sc);
		
		showChart(chart, terminateAfter);
	}

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
