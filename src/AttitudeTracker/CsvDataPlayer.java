package AttitudeTracker;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;

import org.apache.commons.csv.*;

public class CsvDataPlayer implements HeadingProvider, HeadingRateProvider, TimeProvider {
	int csvColMagneticAxisA = 6;
	int csvColMagneticAxisB = 7;
	int csvColRotationRate  = 5;
	List<CSVRecord> csvRecords = null;
	// Advance to the next record after all the information has been added
	int numReadingsTakenThisRecord = 0;
	final int READINGS_PER_RECORD = 3;
	int curRecord = 0;
	double thisRecordTime = 0;
	double thisRecordHeading = 0;
	double thisRecordRotationRate = 0;
	
	CsvDataPlayer(String filename, int col_mag_ax_a, int col_mag_ax_b, int col_rotation_rates) {
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
		
		csvColMagneticAxisA = col_mag_ax_a;
		csvColMagneticAxisB = col_mag_ax_b;
		csvColRotationRate  = col_rotation_rates;
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
		return curRecord * 0.1; // TODO: actual time
	}

	@Override
	public double getW() {
		return Double.parseDouble(csvRecords.get(curRecord).get(csvColRotationRate)) * Math.PI / 180.0;
	}

	@Override
	public double getHeading() {
		double mag_ax_a = Double.parseDouble(csvRecords.get(curRecord).get(csvColMagneticAxisA));
		double mag_ax_b = Double.parseDouble(csvRecords.get(curRecord).get(csvColMagneticAxisB));
		return Math.atan2(mag_ax_a, mag_ax_b);
	}

}
