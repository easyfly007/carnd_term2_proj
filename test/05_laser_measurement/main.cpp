#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "Dense"
#include "measurement_package.h"
#include "tracking.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

int main()
{
	/*
	set measurements
	*/
	std::vector<MeasurementPackage> measurement_pack_list;

	// hard coded input file with the laser and radar measurements
	string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt"
	ifstream in_file(in_file_name_.c_str(), std::ifstream::in);

	if (!in_file.is_open())
	{
		cout << "Cannot open input file: " << in_file_name_ << endl;
	}

	string line;
	// set i to get only first 3 measurements
	int i = 0;
	while (getline(in_file, line) && (i <= 3))
	{
		MeasurementPackage meas_package;
		istringstream iss(line);
		string sensor_type;
		iss >> sensor_type; // read first element from the current line
		int64_t timestamp;
		if (sensor_type.compare("L") == 0)
		{
			// laser measurement
			meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = VectorXd(2);
			float x;
			float y;
			iss >> x;
			iss >> y;
			meas_package.raw_measurements_ << x, y;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);
		}
		else if (sensor_type.compare("R") == 0)
		{
			// skip radar measurements
			continue;
		}
		i ++ ;
	}

	// create a tracking instance;
	Tracking tracking;

	size_t N = measurement_pack_list.size();
	for (size_t k = 0; k < N; k ++)
	{
		tracking.ProcessMeasurement(measurement_pack_list[k]);
	}

	if (in_file.is_open())
	{
		in_file.close();
	}
	return 0;
}