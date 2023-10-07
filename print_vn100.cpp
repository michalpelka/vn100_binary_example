#include "iostream"
#include "vn100/vn100.h"
#include <fstream>

int main(int argc, char **argv) {


  std::string port = "/dev/ttyUSB0";
  std::string csv_file = "imu_data.csv";
  double ts = 10.0;
  if (argc > 1) {
    port = argv[1];
  }

  if(argc > 2){
    csv_file = argv[2];
  }

  if(argc > 3){
    ts = std::stod(argv[3]);
  }

  std::ofstream ofs(csv_file, std::ofstream::out);
  ofs << "timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z" << std::endl;
  const auto imu_data_handler = [&](double ts, std::array<float, 4> q,
                                    std::array<float, 3> acc,
                                    std::array<float, 3> gyro) {
    std::cout << ts << " " << acc[0] << " " << acc[1] << " " << acc[2] << " "
              << gyro[0] << " " << acc[1] << " " << acc[2] << std::endl;
    ofs << ts << "," << acc[0] << "," << acc[1] << "," << acc[2] << ","
        << gyro[0] << "," << gyro[1] << "," << gyro[2] << std::endl;
  };


  vn100_client imu(port);
  imu.setHandler_data(imu_data_handler);

  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(ts * 1000)));

}
