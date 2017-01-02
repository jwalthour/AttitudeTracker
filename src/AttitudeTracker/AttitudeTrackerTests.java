package AttitudeTracker;

import java.awt.Dimension;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import javax.swing.JDialog;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class AttitudeTrackerTests {


	public static void main(String[] args) {
		testSimpleEkfWithSyntheticData();
	}
	
	public static void testSimpleEkfWithSyntheticData() {
		// Generate synthetic data for a simple fictional movement process
		final int NUM_DATAPOINTS = 100;
		final double DT = 0.2; // seconds
		final double PROCESS_NOISE_MAGNITUDE = 0.5;
		final double MEASUREMENT_NOISE_MAGNITUDE = 0.25;
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
			x_actual[i]   = x_ideal [i]   + PROCESS_NOISE_MAGNITUDE * Math.random();
			x_measured[i] = x_actual[i]   + MEASUREMENT_NOISE_MAGNITUDE * Math.random();
		}
		
		// Graph results
		XYSeries x_i_s = new XYSeries("Movement model");
		XYSeries x_a_s = new XYSeries("Actual position");
		XYSeries x_m_s = new XYSeries("Measured position");
		
		for(int i = 1; i < NUM_DATAPOINTS; ++i) {
			x_i_s.add(time[i], x_ideal[i]);
			x_a_s.add(time[i], x_actual[i]);
			x_m_s.add(time[i], x_measured[i]);
		}
		XYSeriesCollection sc = new XYSeriesCollection();
		sc.addSeries(x_i_s);
		sc.addSeries(x_a_s);
		sc.addSeries(x_m_s);
		
		JFreeChart chart = ChartFactory.createScatterPlot("Simple EKF test", "Time (s)", "Value", sc);
		showChart(chart);
	}

	private static void showChart(JFreeChart chart) {
        ChartPanel panel = new ChartPanel(chart);
        panel.setFillZoomRectangle(true);
        panel.setMouseWheelEnabled(true);
        JDialog dialog = new JDialog();
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
