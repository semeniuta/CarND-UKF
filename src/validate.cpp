#include <iostream>
#include <fstream>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "ukf.h"
#include "tools.h"

using namespace std;
using Eigen::VectorXd;


VectorXd readVector(istringstream& iss, int size) {

  std::vector<double> data;

  double x;
  for (int i = 0; i < size; i++) {
    iss >> x;
    data.push_back(x);
  }

  Eigen::VectorXd result = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(data.data(), data.size());

  return result;


}


void readData(
    const string& fname,
    vector<MeasurementPackage>& measurement_pack_list,
    vector<VectorXd>& ground_truth

) {

  ifstream in_file(fname.c_str(), std::ifstream::in);

  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << fname << endl;
  }

  string line;
  while(getline(in_file, line)){

    MeasurementPackage meas_package;

    istringstream iss(line);
    string sensor_type;
    iss >> sensor_type;
    int64_t timestamp;

    if (sensor_type.compare("L") == 0) {

      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = readVector(iss, 2); // x_measured, y_measured

      iss >> timestamp;
      meas_package.timestamp_ = timestamp;

      measurement_pack_list.push_back(meas_package);

    } else if (sensor_type.compare("R") == 0) {

      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = readVector(iss, 3); // rho_measured, phi_measured, rhodot_measured

      iss >> timestamp;
      meas_package.timestamp_ = timestamp;

      measurement_pack_list.push_back(meas_package);

    }

    ground_truth.push_back( readVector(iss, 4) ); // x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth

    // All ground truth values available:
    // x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

  }

  if(in_file.is_open()){
    in_file.close();
  }

}


int main() {

  vector<MeasurementPackage> measurement_pack_list;
  vector<VectorXd> ground_truth;
  vector<VectorXd> estimations;
  Tools tools;
  UKF tracking{};

  string fname = "../data/obj_pose-laser-radar-synthetic-input.txt";
  readData(fname, measurement_pack_list, ground_truth);

  auto n = measurement_pack_list.size();
  for (int i = 0; i < n; i++) {

    cout << "=== (" << i << ") =========================\n";
    cout << "x_GT = \n" << ground_truth[i] << "\n";

    tracking.ProcessMeasurement(measurement_pack_list[i]);

    //cout << "x_CTRV = \n" << tracking.x_ << "\n";

    //VectorXd x_hat = tools.CTRVTransform(tracking.x_);
    //cout << "x_hat = \n" << tracking.x_ << "\n";

    estimations.push_back(tracking.x_);

  }

  VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
  cout << "RMSE:\n" << rmse << "\n";

  return 0;

}