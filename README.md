# このレポジトリについて
このレポジトリは[IMU-TK](https://bitbucket.org/alberto_pretto/imu_tk/)をフォークして改変しているものです。
主な修正は下記の通りです

- クラッシュバグの修正
- Qt5への移行
- キャリブレーションと補正を同時に行う例を追加

私が使っている独自形式のデータファイル(.imu)を用いてキャリブレーションを行い，その結果で別のデータファイルを補正するには
```
./calib_and_correct_imufile -c <calibration_data_file.imu> -i <datafile_to_correct.imu> -o <output_corrected_file.imu> -s <initial_still_time(sec)>
```
のようにしてください。
静止検出がうまく行っていないようでしたら，-w <ウインドウ幅> のオプションを追加して静止検出に使うウインドウの幅を調整してください。
デフォルトのウインドウ幅は100Hzデータでは101，1000Hzデータでは1001になります。

.imuデータファイルの形式は各行に
```
sec nanosec acc_x acc_y acc_z gyro_x gyro_y gyro_z pos_x pos_y pos_z vel_x vel_y vel_z quat_w quat_x quat_y quat_z
```
とデータがタブ区切りで入っています。
pos, vel, quatはこのキャリブレーションツールでは使わないので適当な値を入れておけばOKです。
toolsディレクトリにスポーツセンシング社のワイヤレスモーションセンサのcsvファイルから.imuファイルに変換するツールが入っています。

## キャリブレーションデータの取り方
- IMUを机などの上で静止させ，データ記録を開始
- そのまま静止させておく(30秒~1分くらい)
- IMUの向きを適当に変えて静止させる(3~5秒間くらい)
- いろいろな向きに変えて数秒間静止，を20~30回程度繰り返す(偏らないように)

----
以下はオリジナルのREADMEです。

# IMU-TK: Inertial Measurement Unit ToolKit #

The C++ IMU-TK Library (Inertial Measurement Unit ToolKit) provides simple functions and data structures to calibrate MEMS-based inertial navigation units, and to process and display IMU data. 
IMU-TK implements a multi-position calibration method that does not require any parameter tuning and simply requires the sensor to be moved by hand and placed in a set of different, automatically detected, static positions. IMU-TK also provides a collection of functions for data integration.

## References ##

Papers Describing the Approach:

D. Tedaldi, A. Pretto and E. Menegatti, "A Robust and Easy to Implement Method for IMU Calibration without External Equipments". In: Proceedings of the IEEE International Conference on Robotics and Automation (ICRA 2014), May 31 - June 7, 2014 Hong Kong, China, Page(s): 3042 - 3049 ([PDF](http://www.dis.uniroma1.it/~pretto/papers/tpm_icra2014.pdf))


```
#!bibtex

@inproceedings{tpm_icra2014,
  title={A Robust and Easy to Implement Method for IMU Calibration 
            without External Equipments},
  author={Tedaldi, A. and Pretto, A. and Menegatti, E.},
  booktitle={Proc. of: IEEE International Conference on Robotics and
                   Automation (ICRA)},
  year={2014},
  pages={3042--3049}
}
```



A. Pretto and G. Grisetti, "Calibration and performance evaluation of low-cost IMUs". In Proceedings of the 20th IMEKO TC4 International Symposium, Sep. 15 - 17, 2014 Benevento, Italy, pages: 429 - 434 ([PDF](http://www.dis.uniroma1.it/~pretto/papers/pg_imeko2014.pdf))


```
#!bibtex

@inproceedings{pg_imeko2014,
  title={Calibration and performance evaluation of low-cost IMUs},
  author={Pretto, A. and Grisetti, G.},
  booktitle={Proc. of: 20th IMEKO TC4 International Symposium},
  year={2014},
  pages={429--434}
}
```

## License ##

IMU-TK is licensed under the BSD License.
IMU-TK is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the licenses for more details.

## Requirements ##

The code is tested on Ubuntu 14.04. IMU-TK requires the following tools and libraries: CMake, Eigen3, Ceres Solver, OpenGL, QT and Gnuplot. To install these required packages on Ubuntu, use the terminal command:


```
#!bash

sudo apt-get install build-essential cmake libeigen3-dev libqt4-dev libqt4-opengl-dev freeglut3-dev gnuplot
```
and follow this [guide](http://ceres-solver.org/building.html) to install Ceres Solver.

## Building ##

To build IMU-TK on Ubuntu, type in a terminal the following command sequence.

```
#!bash

cd imu_tk
mkdir build
cd build
cmake  ..
make

```
Test the library with the **test_imu_calib** app (binary in /bin, source code in src/test_imu_calib.cpp): **test_imu_calib** performs an IMU calibration given the data included in bin/test_data/:

```
#!bash

./test_imu_calib test_data/xsens_acc.mat test_data/xsens_gyro.mat

```

## Contact information ##

Alberto Pretto [pretto@dis.uniroma1.it](mailto:pretto@dis.uniroma1.it)
