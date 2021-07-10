#include <iostream>

#include "imu_tk/io_utils.h"
#include "imu_tk/calibration.h"
#include "imu_tk/filters.h"
#include "imu_tk/integration.h"
#include "imu_tk/visualization.h"

using namespace std;
using namespace imu_tk;
using namespace Eigen;

struct ImuData
{
    int tv_sec;
    int tv_nsec;
    double acc_x, acc_y, acc_z;
    double gyro_x, gyro_y, gyro_z;
    double vel_x, vel_y, vel_z;
    double pos_x, pos_y, pos_z;
    double q_w, q_x, q_y, q_z;
};

void load_imu_file(const char *filename, vector<TriadData> &acc_data, vector<TriadData> &gyro_data, vector<ImuData> &all_data)
{
    std::ifstream ifs(filename);
    ImuData d;
    while(ifs) {
        ifs >> d.tv_sec >> d.tv_nsec >> d.acc_x >> d.acc_y >> d.acc_z
            >> d.gyro_x >> d.gyro_y >> d.gyro_z
            >> d.vel_x >>  d.vel_y >>  d.vel_z
            >> d.pos_x >>  d.pos_y >>  d.pos_z
            >> d.q_w >> d.q_x >> d.q_y >> d.q_z;
        all_data.push_back(d);
        double t = d.tv_sec + d.tv_nsec / 1e9;
        acc_data.push_back(TriadData(t, d.acc_x, d.acc_y, d.acc_z));
        gyro_data.push_back(TriadData(t, d.gyro_x, d.gyro_y, d.gyro_z));
    }
}

int main(int argc, char** argv)
{
    if( argc < 3 )
        return -1;

    vector< ImuData > all_data_calib, all_data;
    vector< TriadData > acc_data, gyro_data;
    vector< TriadData > acc_data_target, gyro_data_target;

    cout<<"Importing calibration data from .imu file : "<< argv[1]<<endl;
    load_imu_file(argv[1], acc_data, gyro_data, all_data_calib);
    cout<<"Importing IMU data for correction from .imu file : "<< argv[2]<<endl;
    load_imu_file(argv[2], acc_data_target, gyro_data_target, all_data);
  
  
  CalibratedTriad init_acc_calib, init_gyro_calib;
  init_acc_calib.setBias( Vector3d(0, 0, 0) );
  init_gyro_calib.setScale( Vector3d(1.0, 1.0, 1.0) );
  //init_acc_calib.setBias( Vector3d(32768, 32768, 32768) );
  //init_gyro_calib.setScale( Vector3d(1.0/6258.0, 1.0/6258.0, 1.0/6258.0) );
  
  MultiPosCalibration mp_calib;
    
  mp_calib.setInitStaticIntervalDuration(30.0);
  mp_calib.setInitAccCalibration( init_acc_calib );
  mp_calib.setInitGyroCalibration( init_gyro_calib );  
  mp_calib.setGravityMagnitude(9.7979);
  mp_calib.enableVerboseOutput(true);
  mp_calib.enableAccUseMeans(false);
  //mp_calib.setGyroDataPeriod(0.01);
  mp_calib.calibrateAccGyro(acc_data, gyro_data );
  mp_calib.getAccCalib().save("test_imu_acc.calib");
  mp_calib.getGyroCalib().save("test_imu_gyro.calib");
  
//   for( int i = 0; i < acc_data.size(); i++)
//   {
//     cout<<acc_data[i].timestamp()<<" "
  //         <<acc_data[i].x()<<" "<<acc_data[i].y()<<" "<<acc_data[i].z()<<" "
  //         <<gyro_data[i].x()<<" "<<gyro_data[i].y()<<" "<<gyro_data[i].z()<<endl;
//   }
//   cout<<"Read "<<acc_data.size()<<" tuples"<<endl;


  auto acc_calib_ = mp_calib.getAccCalib();
  auto gyro_calib_ = mp_calib.getGyroCalib();

  int n_samps = acc_data_target.size();
  std::vector<TriadData> ca, cg;
  ca.reserve(n_samps);
  
  // Calibrate the input accelerometer data with the obtained calibration
  for( int i = 0; i < n_samps; i++) {
    ca.push_back( acc_calib_.unbiasNormalize( acc_data_target[i]) );
    cg.push_back( gyro_calib_.unbiasNormalize( gyro_data_target[i]) );
  }
  std::ofstream ofs("calibrated_out.imu");
  for( int i = 0; i < n_samps; i++) {
    ofs << all_data[i].tv_sec << "\t" << all_data[i].tv_nsec << "\t"
        << ca[i].x() << "\t" << ca[i].y() << "\t" << ca[i].z() << "\t"
        << cg[i].x() << "\t" << cg[i].y() << "\t" << cg[i].z() << "\t"
        << all_data[i].vel_x << "\t" << all_data[i].vel_y << "\t" << all_data[i].vel_z << "\t"
        << all_data[i].pos_x << "\t" << all_data[i].pos_y << "\t" << all_data[i].pos_z << "\t"
        << all_data[i].q_w << "\t" << all_data[i].q_x << "\t" << all_data[i].q_y << "\t" << all_data[i].q_z
        << std::endl;
  }
  
  return 0;
}
