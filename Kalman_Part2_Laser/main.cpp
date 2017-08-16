#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>

#include "Eigen/Dense"

#include "measurement_package.h"
#include "tracking.h"


int main() {
	 /*
	 We have a file with lines of data. The line could be either from RADAR or LIDAR.
	 Consider each line a measurement package.
	 Let's create a list (vector) of the first 4 LASER measurments and process the last 3 of it.
	 */



    /* OPEN MEASUREMENTS FILE */
	// hardcoded input file with laser and radar measurements
	std::string in_file_name_ = "EKF_DATA/obj_pose-laser-radar-synthetic-input.txt";
	std::ifstream in_file(in_file_name_.c_str()); // c_str() converts string to char*, ifstream::in is a flag to allow access to input from the stream
    // Check for error
	if (!in_file.is_open()) {
		std::cout << "Cannot open input file: " << in_file_name_ << std::endl;
	}



	/* READ FILE - line by line, only LASER, 4 lines */
	std::vector<MeasurementPackage> measurement_pack_list; // Here is the list of measurment packages (all 3 LASER lines)
	MeasurementPackage meas_package; // Each line data
    long timestamp; // timestamp data from measurment package (line)
    float x; // x position data from measurment package (line)
    float y; // y position data from measurment package (line)
	std::string line; // it will hold temporary stream line information
	// set i to get only first 4 measurements
	int i = 0; // starting with zero the while will loop 0,1,2,3 (4 lines)
	while(std::getline(in_file, line) && (i<=3)){

        /* READ LASER ONLY */
		std::istringstream iss(line);  // istringstream class is used to split line information into variables
		std::string sensor_type;
		iss >> sensor_type;	//reads first element from the current line
		if(sensor_type.compare("L") == 0){	//laser measurement, string.compare method will return 0 if true
			//read measurements
			meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = Eigen::VectorXd(2);
			iss >> x;
			iss >> y;
			meas_package.raw_measurements_ << x,y;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);
		}
		else if(sensor_type.compare("R") == 0){
			//Skip Radar measurements
			continue;
		}
		i++;
	}



    /* PROCESS THE STORED LASER MEASUREMENTS */
	//Create a Tracking instance
	Tracking tracking; // tracking class has the method ProcessMeasurement, used to process the data
	//call the ProcessingMeasurement() function for each measurement
	size_t N = measurement_pack_list.size(); // how many packages (lines) to loop through? size_t id used because we define a variable that takes a size
	for (size_t k = 0; k < N; ++k) {
		tracking.ProcessMeasurement(measurement_pack_list[k]); // the method will use first line (package) only to initialize
                                                               // so the filtering effectivelly starts from the second frame
                                                               // (the speed is also unknown in the first frame, so that is appropriate)
	}


	/* CLOSE MEASUREMENTS FILE */
	if(in_file.is_open()){
		in_file.close();
	}


	return 0;
}
