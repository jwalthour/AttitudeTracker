package AttitudeTracker;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;

import org.apache.commons.csv.*;

public class CsvDataPlayer implements HeadingProvider, HeadingRateProvider, TimeProvider {
	int csvColHeading = -1;
	int csvColW       = -1;
	int csvColMagAxA  = -1;
	int csvColMagAxB  = -1;
	double magCorrA = 0.0; 
	double magCorrB = 0.0;
	int csvColTime    = -1;
	List<CSVRecord> csvRecords = null;
	// Advance to the next record after all the information has been added
	int numReadingsTakenThisRecord = 0;
	final int READINGS_PER_RECORD = 3;
	int curRecord = 0;
	double thisRecordTime = 0;
	double thisRecordHeading = 0;
	double thisRecordRotationRate = 0;
	
	CsvDataPlayer(String filename, int col_mag_ax_a, int col_mag_ax_b, double mag_corr_a, double mag_corr_b, int col_w, int col_time) {
		try {
			CSVParser csv_parser = new CSVParser(new FileReader(filename), CSVFormat.DEFAULT);
			csvRecords = csv_parser.getRecords();
			csv_parser.close();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		csvColMagAxA  = col_mag_ax_a;
		csvColMagAxB  = col_mag_ax_b;
		magCorrA = mag_corr_a;
		magCorrB = mag_corr_b;
		csvColW = col_w;
		csvColTime    = col_time;
	}
	
	public void advancePlayback() {
		if(++curRecord >= csvRecords.size()) {
			 curRecord  = csvRecords.size() - 1;
		}
	}
	
	public boolean hasMoreData() {
		return (curRecord < (csvRecords.size() - 1));
	}

	@Override
	public double getTime() {
		if(csvColTime >= 0) {
			return Double.parseDouble(csvRecords.get(curRecord).get(csvColTime));
		} else {
			return curRecord * 0.1;
		}
	}

	@Override
	public double getW() {
		return Double.parseDouble(csvRecords.get(curRecord).get(csvColW));
	}

	@Override
	public double getHeading() {
		double mag_ax_a = Double.parseDouble(csvRecords.get(curRecord).get(csvColMagAxA)) - magCorrA;
		double mag_ax_b = Double.parseDouble(csvRecords.get(curRecord).get(csvColMagAxB)) - magCorrB;
		return Math.atan2(mag_ax_a, mag_ax_b);
	}

}
