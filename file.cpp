#define _CRT_SECURE_NO_WARNINGS             /* Eigenのインクルードでopencvを呼んでいる？ */
/* Eigenのコア機能とジオメトリ機能を使う */
//#include <Eigen/Core>
#include <Eigen/Geometry>
#include "file.h"
#include "GlobalVariable.h"
#include "Inline.h"
#include "Imudata.h"
#include "Csm.h"
#include "TrackingObject.h"
#include "MovingObjectDetection.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <string>
#include <sstream>
#include "Eigen/Dense"

int SMAP_COUNT;
const int NUM_OUTPUT = 10;               /* 出力する数 */
const int NUMSET = 12;                                   /* 1パケットに対して入っている32個の観測点のセット数 */
const int LINEAR_INTERPOLATION_FLAG = 1;                 /* 線形補間を行うかどうかのフラグ，1：あり，other：なし */
const std::string  FOLDER_NAME = "Output/World";						//pcdファイルのディレクトリ

FILE *result;             /* GNSS/INS複合装置から得られるデータのファイルポインタ */
FILE *pointcloud;
FILE *out;
FILE *outputcurb;
FILE *output[NUM_OUTPUT];
FILE *robot;
FILE *miki;
FILE *csm;             /* IMUから得られるデータのファイルポインタ */
errno_t error;
std::ifstream ifs;

//pcl::visualization::CloudViewer viewer("Viewer");
Laserpoint g_laserpoint[LAYER][UPPERLIMIT_PSUM];        /* レーザ観測点のクラス，ローカルで宣言するとメモリがオーバーするため */
Laserpoint g_laserpoint2[LAYER][UPPERLIMIT_PSUM];        /* レーザ観測点のクラス，ローカルで宣言するとメモリがオーバーするため */
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI> map_cloud;
pcl::PointCloud<pcl::PointXYZI> starget_cloud;
std::vector<pcl::PointCloud<pcl::PointXYZI>> reference_smap_vector;
int output_number[NUM_OUTPUT] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };  /* 追跡結果を出力する追跡物体番号 */
int file_position = 0;                  /* ローカルにすると描画速度が遅くなる */
long diff_mlstime = 0;                  /* IMUの最初のデータの時刻とMLSの最初のタイムスタンプの差 */
long mls_starttime = 0;                 /* MLSの最初のパケットが得られた時刻のtimeGetTime()の値 */
long prev_localization_time = 0;        /* 前時刻における自己姿勢推定データのタイムスタンプ */
char folder_path[NAME_ARREY_SIZE];      /* ファイルのあるフォルダのパス */
double before_position[NUM_USE_LINEAR_INTERPOLATION_VARIABLES] = { 0 };    /* 前ループでの自己姿勢情報 */

extern int g_numdata;            /* 描画するデータ番号 */

								 
void outputrobot(double now_position[], double v) {
	fprintf(robot, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
		now_position[0], now_position[1], now_position[2], now_position[3], now_position[4], now_position[5],v);
}
void Outputmiki(double x,double y) {
	fprintf(miki, "%lf,%lf\n",x,y);
}
//マップ生成
void savemap(pcl::PointCloud<pcl::PointXYZI>::Ptr target_scan) {
	int i;
	
	map_cloud += *target_scan;
	//viewer.showCloud(target_scan);
	//make output folder
	_mkdir("Output");

	//Save map_cloud
	pcl::io::savePCDFileBinary("Output/map_cloud.pcd", map_cloud);
	
	/*
	map_cloud += *target_scan;//地図+=参照地図
	viewer.showCloud(target_scan);
	for (i = 1; i < 10; i++) {
		if (g_loopcount == 1000 * i) {//1000×10=10000まで対応
			std::cout << "AAAA" << std::endl;//通ってるか確認 OK
			//Save map_cloud
			//pcl::PointCloud<pcl::PointXYZI> map_cloud;
			pcl::io::savePCDFileBinary("Output/map_cloud.pcd", map_cloud);//PCLライブラリでmap_cloud.pcdを出力
		}
	}
	*/


}
/*//////////////////////////////////////////////////////
ファイルオープンを行う関数
引数
なし
返り値
なし
//////////////////////////////////////////////////////*/
void MicroevFileopen(void) {
	int i;
	FILE *velostarttime;

	/* ファイル名保存配列 */
	char mls_starttime_name[NAME_ARREY_SIZE];        /* velo_start_time.datはGNSS/INS複合装置の最初のパケットが送られてきたときのパソコンの時間 */
	char result_name[NAME_ARREY_SIZE];              /* result.txtはGNSS/INS複合装置のデータ */
	char output_name[NAME_ARREY_SIZE];
	char csm_file_name[NAME_ARREY_SIZE];

	/* ファイルのあるフォルダのパスを設定 */
	sprintf_s(folder_path, NAME_ARREY_SIZE, "InputData");

	errno_t error;
	if (IMU_DATA == 1) {
		/* ファイル名を設定 */
		sprintf_s(mls_starttime_name, NAME_ARREY_SIZE, "%s\\%d_velo_start_time.dat", folder_path, g_numdata);
		sprintf_s(result_name, NAME_ARREY_SIZE, "%s\\%d_result.txt", folder_path, g_numdata);
		sprintf_s(csm_file_name, NAME_ARREY_SIZE, "%s\\%d_MTi300.csv", folder_path, g_numdata);

		if ((error = fopen_s(&velostarttime, mls_starttime_name, "r")) != 0) {
			std::cout << g_numdata << "_velo_start_time.dat file open error!" << std::endl;
			FileOpenError(error);
		}
		if ((error = fopen_s(&result, result_name, "r")) != 0) {
			std::cout << g_numdata << "_result.txt file open error!" << std::endl;
			FileOpenError(error);
		}
		/* 最初のパケットが送られてきた時点でのtimeGT()の値を取得 */
		fscanf_s(velostarttime, "%ld", &mls_starttime);
		fclose(velostarttime);

		ifs.open(csm_file_name);
		if (ifs.fail()) {
			//std::cerr << "csm file open error!" << std::endl;
			std::cerr << "xsens file open error!" << std::endl;
		}

		std::string str;
		getline(ifs,str);
	}

	/* ファイルオープン */
	if ((error = fopen_s(&pointcloud, "Output/pointcloud.csv", "w")) != 0) {
		std::cout << "pointcloud.csv file open error!" << std::endl;
		FileOpenError(error);
	}
	if ((error = fopen_s(&robot, "Output/outputrobot.csv", "w")) != 0) {
		std::cout << "outputrobot.csv file open error!" << std::endl;
		FileOpenError(error);
	}
	if ((error = fopen_s(&out, "Output/out.csv", "w")) != 0) {
		std::cout << "out.csv file open error!" << std::endl;
		FileOpenError(error);
	}
	if ((error = fopen_s(&outputcurb, "Output/outputcurb.csv", "w")) != 0) {
		std::cout << "outputcurb.csv file open error!" << std::endl;
		FileOpenError(error);
	}
	if ((error = fopen_s(&miki, "Output/miki.csv", "w")) != 0) {
		std::cout << "pointcloud.csv file open error!" << std::endl;
		FileOpenError(error);
	}
	for (i = 0; i < NUM_OUTPUT; i++) {
		sprintf_s(output_name, NAME_ARREY_SIZE, "Output/output%d.csv", i);
		if ((error = fopen_s(&output[i], output_name, "w")) != 0) {
			std::cout << output_name << " file open error!" << std::endl;
			FileOpenError(error);
		}
	}
}

/*//////////////////////////////////////////////////////
GNSS/INS複合航法装置のデータスキップ関数
引数
なし
返り値
なし
//////////////////////////////////////////////////////*/
void DataSkip(void) {
	/* IMUのデータ読み込み変数 */
	int intdum = 0;
	double doubledum[13] = { 0 };

	/* GPSタイム　緯度　経度　高さ　roll速度　ピッチ速度　ヨー速度　roll pitch yaw x座標 y座標 z座標　タイムスタンプ 計14個 */
	fscanf_s(result, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %d\t\n",
		&doubledum[0], &doubledum[1], &doubledum[2], &doubledum[3], &doubledum[4], &doubledum[5], &doubledum[6],
		&doubledum[7], &doubledum[8], &doubledum[9], &doubledum[10], &doubledum[11], &doubledum[12], &intdum);
}


/*//////////////////////////////////////////////////////
microEV用ファイルクローズ
引数
なし
返り値
なし
//////////////////////////////////////////////////////*/
void MicroevFileclose(void) {
	int i;
	fclose(robot);
	fclose(result);
	fclose(pointcloud);
	fclose(out);
	for (i = 0; i < NUM_OUTPUT; i++) {
		fclose(output[i]);
	}
}


/*//////////////////////////////////////////////////////
MLSのデータ読み込み関数
引数
const int：GNSS/INS複合航法装置のタイムスタンプ
返り値
int：
//////////////////////////////////////////////////////*/
int MlsDataRead(const int i_timestamp) {
	const int NUM_PACKET = 1206;                /* MLSのパケットのバイト数 */
	const int USE_OBSERVATION_POINT = 100;      /* 自己姿勢情報に対して用いる観測点の時間（ms） */

	int i = 0, j = 0;
	int mlspointsum = 0;                        /* 読み込んだ観測点数 */
	int packetcount = 0;                        /* 読み込んだパケット数 */
	unsigned int ts_mls = 0;                    /* MLSのタイムスタンプ */
	long localization_time = i_timestamp;       /* 現時刻における自己姿勢データのタイムスタンプ　このまわり-100ms～0msの観測点を使用 */

	char g_mlsDataName[NAME_ARREY_SIZE] = { 0 };        /* MLSのデータ名配列 */
	sprintf_s(g_mlsDataName, NAME_ARREY_SIZE, "%s\\%d_hdl_data.bin", folder_path, g_numdata);

	FILE *mls;
	errno_t error;

	if ((error = fopen_s(&mls, g_mlsDataName, "rb")) != 0) {
		std::cout << "_mls_data.bin file open error!" << std::endl;
		FileOpenError(error);
	}
	fseek(mls, file_position, SEEK_SET);        /* ファイルの位置指示子を移動 */

	char *velo_buf = new char[NUM_PACKET];        /* mls読み込み用バッファのメモリ領域確保 */

												  /* 初期スキャンにおける処理 あたまの1パケットを取得してタイムスタンプを参照 */
	if (g_loopcount == 0) {
		fread(velo_buf, sizeof(char), NUM_PACKET, mls);        /* 1パケット分をバッファに読み出し */
															   /* ビットシフトでタイムスタンプを取得 */
		ts_mls = ((unsigned int)((((unsigned int)((unsigned char)velo_buf[NUM_PACKET - 3])) << 24) + (((unsigned int)((unsigned char)velo_buf[NUM_PACKET - 4])) << 16)
			+ (((unsigned int)((unsigned char)velo_buf[NUM_PACKET - 5])) << 8) + (unsigned char)velo_buf[NUM_PACKET - 6]));
		/* パケットのタイムスタンプ(マイクロ秒)とtGT(ミリ秒)との対応を取る */
		diff_mlstime = mls_starttime - MicroToMilli(ts_mls);
	}

	while (1) {
		/* １パケット分をバッファに読み出し */
		fread(velo_buf, sizeof(char), NUM_PACKET, mls);

		/* 読み出したパケットのタイムスタンプを参照 */
		/* ビットシフトでタイムスタンプを取得 */
		ts_mls = ((unsigned int)((((unsigned int)((unsigned char)velo_buf[NUM_PACKET - 3])) << 24) + (((unsigned int)((unsigned char)velo_buf[NUM_PACKET - 4])) << 16)
			+ (((unsigned int)((unsigned char)velo_buf[NUM_PACKET - 5])) << 8) + (unsigned char)velo_buf[NUM_PACKET - 6]));
		//std::cout << "mlstimestamp" << localization_time << std::endl;
		/* 自己姿勢情報に対応する観測点のみを取得 */
		/* 自己姿勢情報のタイムスタンプの100 [ms]より過去に得られた観測点　→　何もしない（現在の自己姿勢情報に対応するところまで読み飛ばす） */
		if (MicroToMilli(ts_mls) < (unsigned int)(localization_time - diff_mlstime) - USE_OBSERVATION_POINT) {
		}
		/* 自己姿勢情報のタイムスタンプより未来に得られた観測点　→　これ以上読み込んでも現在の自己姿勢情報に対応する観測点は得られないので，ここで読み込みを終了 */
		else if (MicroToMilli(ts_mls) > (unsigned int)(localization_time - diff_mlstime)) {
			break;
		}
		/* 自己姿勢情報のタイムスタンプの前後50msの観測点のみ採用 */
		else {
			/* MLSのデータを配列に代入 */
			packetcount = AssignmentToLaserpoint(ts_mls, velo_buf, packetcount);
		}

		////CSM同期（ごり押したやつ）
		//if (ts_csm * 1000 - ts_mls > 300000 && packetcount > 180) {
		//	packetcount = 0;
		//}
		//if (ts_csm * 1000 < ts_mls) {
		//	break;
		//}
	
	}
/*
	if (packetcount == 0) {
		packetcount = 180;
		}*/

	mlspointsum = packetcount * NUMSET;        /* 1パケットには32個の観測点が12セット入っている */

	file_position = ftell(mls);
	fclose(mls);
	delete[] velo_buf;                /* 動的配列のメモリ解放 */

	return mlspointsum;
}


/*//////////////////////////////////////////////////////
MLSのデータをクラスに代入
引数
なし
返り値
int：
//////////////////////////////////////////////////////*/
int AssignmentToLaserpoint(unsigned int i_ts_mls, char *i_velo_buf, int o_packetcount) {

	int i = 0, j = 0;
	double rot_angle = 0;                     /* 観測点の角度 */

	
	/* 垂直角度におけるsinの値 */
	const double sin_siita[32] = { -0.5101, -0.1621, -0.4898, -0.1392, -0.4695,
		-0.1160, -0.4487, -0.0929, -0.4278, -0.0698,
		-0.4067, -0.0466, -0.3854, -0.0232, -0.3637,
		0.0, -0.3420, 0.0232, -0.3201, 0.0466,
		-0.2979, 0.0698, -0.2756, 0.0929, -0.2533,
		0.1162, -0.2306, 0.1392, -0.2079, 0.1621,
		-0.1852, 0.1852 };
	/* 垂直角度におけるcosの値 */
	const double cos_siita[32] = { 0.8601, 0.9868, 0.8718, 0.9903, 0.8829,
		0.9933, 0.8937, 0.9957, 0.9039, 0.9976,
		0.9135, 0.9989, 0.9227, 0.9997, 0.93151,
		1.0, 0.9397, 0.9997, 0.9474, 0.9989,
		0.9546, 0.9976, 0.9613, 0.9957, 0.9674,
		0.9932, 0.9731, 0.9903, 0.9781, 0.9868,
		0.9827, 0.9827 };
	/* MLSとSPANの位置関係の設定がミスしているため補正する値 */


	double SPAN_TO_MLS_X;
	double SPAN_TO_MLS_Y;
	double SPAN_TO_MLS_Z;
	double YAW_CORRECTION;
	double PITCH_CORRECTION;
	double TRANSFORMATION_TO_ROBOT_COORDINATE;

	
	if (Vehicle_Type == 0) {
		SPAN_TO_MLS_X = 0.0;
		SPAN_TO_MLS_Y = 0.0;
		SPAN_TO_MLS_Z = -0.2;
		YAW_CORRECTION = 0.0;
		PITCH_CORRECTION = 0.0;
		//TRANSFORMATION_TO_ROBOT_COORDINATE = 90.0;
	}
	else if (Vehicle_Type == 1) {
		SPAN_TO_MLS_X = 0.0;      
		SPAN_TO_MLS_Y = -0.32;     
		SPAN_TO_MLS_Z = -0.2;      
		YAW_CORRECTION = 0.0;   
		PITCH_CORRECTION = 1.5;     /*MLSのpitch角は1.5°ずれている未計測*/
		//TRANSFORMATION_TO_ROBOT_COORDINATE = 90.0;  
	}
	else if (Vehicle_Type == 2) {
		SPAN_TO_MLS_X = -0.245;      /* 初期値0.4475   測定値0.6     補正値 -0.245     計算値 -0.31 */
		SPAN_TO_MLS_Y = -0.38;       /* 初期値0.2      測定値0.1     補正値 -0.38      計算値 -0.46 */
		SPAN_TO_MLS_Z = -1.95;       /* 初期値1.9      測定値1.8     補正値 0.6        計算値 0.93 , 変更前-1.5 */
		YAW_CORRECTION = 0.0;        /* MLSのyaw角は2°ずれている（らしい．未計測） */
		PITCH_CORRECTION = 0.0;
		//TRANSFORMATION_TO_ROBOT_COORDINATE = 90.0;    /* センサ座標系からロボット座標系に変換する値．現時点では90°プラス方向に回転している */
	}

	for (i = 0; i < NUMSET; i++) {
		/* 観測点の角度をパケットからビットシフトで取得 */
		rot_angle = ((double)((((unsigned int)((unsigned char)i_velo_buf[3 + i * 100])) << 8) + (unsigned char)i_velo_buf[2 + i * 100])) / 100.0;
		for (j = 0; j < LAYER; j++) {
			g_laserpoint[j][o_packetcount * NUMSET + i].hori_angle = rot_angle+ YAW_CORRECTION;
			//g_laserpoint[j][o_packetcount * NUMSET + i].hori_angle = rot_angle + YAW_CORRECTION;
			g_laserpoint[j][o_packetcount * NUMSET + i].vertical_angle = rot_angle + PITCH_CORRECTION;
			g_laserpoint[j][o_packetcount * NUMSET + i].distance = (double)((((unsigned int)((unsigned char)i_velo_buf[j * 3 + 5 + i * 100])) << 8) + (unsigned char)i_velo_buf[j * 3 + 4 + i * 100]) * 0.002;
			g_laserpoint[j][o_packetcount * NUMSET + i].ref_intensity = (double)((unsigned char)i_velo_buf[j * 3 + 6 + i * 100]);
			g_laserpoint[j][o_packetcount * NUMSET + i].x_l = -SPAN_TO_MLS_X + g_laserpoint[j][o_packetcount * NUMSET + i].distance * cos_siita[j] * cos(DigreeToRadian(g_laserpoint[j][o_packetcount * NUMSET + i].hori_angle));
			g_laserpoint[j][o_packetcount * NUMSET + i].y_l = -SPAN_TO_MLS_Y + g_laserpoint[j][o_packetcount * NUMSET + i].distance * cos_siita[j] * -sin(DigreeToRadian(g_laserpoint[j][o_packetcount * NUMSET + i].hori_angle));
			g_laserpoint[j][o_packetcount * NUMSET + i].z_l = -SPAN_TO_MLS_Z + g_laserpoint[j][o_packetcount * NUMSET + i].distance * sin_siita[j];
			g_laserpoint[j][o_packetcount * NUMSET + i].timestump = i_ts_mls;
		}
	}
	o_packetcount++;
	return o_packetcount;
}


/*//////////////////////////////////////////////////////
二次元の線形補間つき座標変換関数
引数
なし
返り値
なし
//////////////////////////////////////////////////////*/
void LinearInterpolationTwoDimention(const int i_timestamp, double i_robotX, double i_robotY, double i_robotYaw, int i_mlspointsum) {
	const double NOT_ZERO_CONSTANT = 0.000000001;           /* time_diffが0となったときの0除算とならないようにする定数 */
	const double DISTANCE_BETWEEN_CENTER_AND_MLS = 0.0;     /* 車両中心からMLS設置点までの距離 */

	int i = 0, j = 0;
	long time_diff = 0;                          /* 自己姿勢推定情報取得時刻間隔 */
	long localization_time = i_timestamp;        /* 現時刻における自己姿勢データのタイムスタンプ　このまわり-100ms～0msの観測点を使用 */
	double linear_interpolation_gradient[3] = { 0 };        /* 線形補間用の傾き，1：x，2：y，3：yaw */
	double temp_time = 0;                        /* 観測点が得られたときの時間と前時刻の自己姿勢情報が得られた時間の差 */
	double temp_position[3] = { 0 };             /* 補間したロボット位置 */
	double now_position[3] = { i_robotX, i_robotY, i_robotYaw };    /* 現時刻におけるロボット位置，1：x，2：y，3：yaw */
	double temp_x, temp_y;

	/* 前時刻と現時刻の間に移動した傾きを線形で算出 */
	for (i = 0; i < 2; i++) {
		linear_interpolation_gradient[i] = (now_position[i] - before_position[i]) / ((double)(localization_time - prev_localization_time) + NOT_ZERO_CONSTANT);
	}

	/* 角度が0から2πまでの範囲なのでその転換点の処理 */
	if (now_position[2] - before_position[2] > PI) {
		linear_interpolation_gradient[2] = (now_position[2] - before_position[2] - 2 * PI) / ((double)(localization_time - prev_localization_time) + NOT_ZERO_CONSTANT);
	}
	else if (now_position[2] - before_position[2] < -PI) {
		linear_interpolation_gradient[2] = (now_position[2] - before_position[2] + 2 * PI) / ((double)(localization_time - prev_localization_time) + NOT_ZERO_CONSTANT);
	}
	else {
		linear_interpolation_gradient[i] = (now_position[i] - before_position[i]) / ((double)(localization_time - prev_localization_time) + NOT_ZERO_CONSTANT);
	}

	/* 観測点のタイムスタンプと算出した傾きからそのタイムスタンプのときの姿勢を補間する */
	for (i = 0; i < i_mlspointsum; i++) {
		if (LINEAR_INTERPOLATION_FLAG == 1) {
			temp_time = (double)(((long)(int)g_laserpoint[0][i].timestump) + (MilliToMicro(diff_mlstime - prev_localization_time)));
			for (j = 0; j < 3; j++) {
				temp_position[j] = (linear_interpolation_gradient[j] * MicroToMilliDouble(temp_time)) + before_position[j];
			}
		}
		else {
			for (j = 0; j < 3; j++) {
				temp_position[j] = now_position[j];
			}
		}
		for (j = 0; j < LAYER; j++) {
			/* ポイントクラウドをロボット座標系に変換 */
			//temp_x = DISTANCE_BETWEEN_CENTER_AND_MLS + g_laserpoint[j][i].x_l * cos(0.5 * PI) - g_laserpoint[j][i].y_l * sin(0.5 * PI);
			//temp_y = g_laserpoint[j][i].x_l * sin(0.5 * PI) + g_laserpoint[j][i].y_l * cos(0.5 * PI);
			temp_x = g_laserpoint[j][i].x_l;
			temp_y = g_laserpoint[j][i].y_l;
			/* ポイントクラウドを地上固定座標系に変換 */
			g_laserpoint[j][i].x = temp_position[0] + temp_x * cos(temp_position[2]) - temp_y * sin(temp_position[2]);
			g_laserpoint[j][i].y = temp_position[1] + temp_x * sin(temp_position[2]) + temp_y * cos(temp_position[2]);
			g_laserpoint[j][i].z = g_laserpoint[j][i].z_l;
		}
	}

	/* 前時刻の情報を更新 */
	for (i = 0; i < 3; i++) {
		before_position[i] = now_position[i];
	}
	prev_localization_time = localization_time;
}


/*//////////////////////////////////////////////////////
三次元の線形補間つき座標変換関数
引数
const int：GNSS/INS複合航法装置のタイムスタンプ，
double[]：ロボットの姿勢（0：x，1：y，2：z，3：roll，4：pitch，5：yaw），int：1層の観測点数
返り値
なし
//////////////////////////////////////////////////////*/
void LinearInterpolationThreeDimention(const int i_timestamp, double now_position[], int i_mlspointsum) {
	const double NOT_ZERO_CONSTANT = 0.000000001;           /* time_diffが0となったときの0除算とならないようにする定数 */
	const double DISTANCE_BETWEEN_CENTER_AND_MLS = 0.0;     /* 車両中心からMLS設置点までの距離 */
	const int START_DEGREE = 3;                             /* position行列の角度のデータが始まる位置 x,y,z,roll,pitch,yawだから3 */
	const int END_DEGREE = 6;                               /* position行列の角度のデータが終わる位置 x,y,z,roll,pitch,yawだから6 */
	int count = 0;
	int i = 0, j = 0;
	long localization_time = i_timestamp;                /* 現時刻における自己姿勢データのタイムスタンプ　このまわり-100ms～0msの観測点を使用 */
	double linear_interpolation_gradient[NUM_USE_LINEAR_INTERPOLATION_VARIABLES] = { 0 };        /* 線形補間用の傾き，1：x，2：y，3：yaw */
	double temp_time = 0;                                /* 補間する観測点の時間とSPAN-CPTの時間の対応 */
	double temp_position[NUM_USE_LINEAR_INTERPOLATION_VARIABLES] = { 0 };                        /* 補間したロボット位置 */

	Eigen::MatrixXd point_b = Eigen::MatrixXd::Zero(3, 1);
	Eigen::MatrixXd point_w = Eigen::MatrixXd::Zero(3, 1);
	Eigen::MatrixXd transform = Eigen::MatrixXd::Zero(3, 3);

	///* 前時刻と現時刻の間に移動した傾きを線形で算出 */
	for (i = 0; i < START_DEGREE; i++) {
		linear_interpolation_gradient[i] = (now_position[i] - before_position[i]) / ((double)(localization_time - prev_localization_time) + NOT_ZERO_CONSTANT);
	}

	/* 角度が0から2πまでの範囲なのでその転換点の処理 */
	for (i = START_DEGREE; i < END_DEGREE; i++) {
		if (now_position[i] - before_position[i] > PI) {
			linear_interpolation_gradient[i] = (now_position[i] - before_position[i] - 2 * PI) / ((double)(localization_time - prev_localization_time) + NOT_ZERO_CONSTANT);
		}
		else if (now_position[i] - before_position[i] < -PI) {
			linear_interpolation_gradient[i] = (now_position[i] - before_position[i] + 2 * PI) / ((double)(localization_time - prev_localization_time) + NOT_ZERO_CONSTANT);
		}
		else {
			linear_interpolation_gradient[i] = (now_position[i] - before_position[i]) / ((double)(localization_time - prev_localization_time) + NOT_ZERO_CONSTANT);
		}
	}

	for (i = 0; i < i_mlspointsum; i++) {
		if (LINEAR_INTERPOLATION_FLAG == 1) {
			/* 前時刻の時間から観測点が得られた時間までを算出 */
			temp_time = (double)(((long)(int)g_laserpoint[0][i].timestump) + (MilliToMicro(diff_mlstime - prev_localization_time)));
			/* 前時刻と現時刻の間に移動した傾きを線形で算出 */
			for (j = 0; j < NUM_USE_LINEAR_INTERPOLATION_VARIABLES; j++) {
				temp_position[j] = (linear_interpolation_gradient[j] * MicroToMilliDouble(temp_time)) + before_position[j];
			}
		}
		else {
			for (j = 0; j < NUM_USE_LINEAR_INTERPOLATION_VARIABLES; j++) {
				temp_position[j] = now_position[j];
			}
		}

	
		

		/* 三次元の座標変換式 */
		transform(0, 0) = cos(temp_position[4]) * cos(temp_position[5]);
		transform(1, 0) = cos(temp_position[4]) * sin(temp_position[5]);
		transform(2, 0) = -sin(temp_position[4]);
		transform(0, 1) = sin(temp_position[3]) * sin(temp_position[4]) * cos(temp_position[5]) - cos(temp_position[3]) * sin(temp_position[5]);
		transform(1, 1) = sin(temp_position[3]) * sin(temp_position[4]) * sin(temp_position[5]) + cos(temp_position[3]) * cos(temp_position[5]);
		transform(2, 1) = sin(temp_position[3]) * cos(temp_position[4]);
		transform(0, 2) = cos(temp_position[3]) * sin(temp_position[4]) * cos(temp_position[5]) + sin(temp_position[3]) * sin(temp_position[5]);
		transform(1, 2) = cos(temp_position[3]) * sin(temp_position[4]) * sin(temp_position[5]) - sin(temp_position[3]) * cos(temp_position[5]);
		transform(2, 2) = cos(temp_position[3]) * cos(temp_position[4]);

		for (j = 0; j < LAYER; j++) {
			if (g_loopcount > 0) {
				point_b(0, 0) = g_laserpoint[j][i].x_l;
				point_b(1, 0) = g_laserpoint[j][i].y_l;
				point_b(2, 0) = g_laserpoint[j][i].z_l;
				/* 座標変換の行列計算 */
				point_w = transform * point_b;
			}

			/* ポイントクラウドを地上固定座標系に変換 */
			g_laserpoint[j][i].x = point_w(0, 0) + temp_position[0];
			g_laserpoint[j][i].y = point_w(1, 0) + temp_position[1];
			g_laserpoint[j][i].z = point_w(2, 0) + temp_position[2];
		}
	}

	/* 前時刻の情報を更新 */
	for (i = 0; i < NUM_USE_LINEAR_INTERPOLATION_VARIABLES; i++) {
		before_position[i] = now_position[i];
	}
	prev_localization_time = localization_time;
}

/*//////////////////////////////////////////////////////
ロボット座標系原点算出
引数
IMUデータ
返り値
なし
//////////////////////////////////////////////////////*/
void RobotPosition(double x, double y, double z, double roll, double pitch, double yaw) {
	Eigen::MatrixXd robot_b = Eigen::MatrixXd::Zero(3, 1);
	Eigen::MatrixXd robot_w = Eigen::MatrixXd::Zero(3, 1);
	Eigen::MatrixXd trans = Eigen::MatrixXd::Zero(3, 3);
	double IMU_HEIGHT;
	double IMU_WIDTH;
	double IMU_LENGTH;
	double temp_position[NUM_USE_LINEAR_INTERPOLATION_VARIABLES] = { 0 };

	
	 if (Vehicle_Type == 0) {
		IMU_HEIGHT = 2.0;
		IMU_LENGTH = 0.1;
	 }
	 else if (Vehicle_Type == 1) {
		 IMU_HEIGHT =1.75;
		 IMU_WIDTH = -0.32;
		 IMU_LENGTH = 0.3;
	 }
	 else if (Vehicle_Type == 2) {
		 //IMU_HEIGHT = 0.0;
		 //IMU_WIDTH = -0.07;
		 //IMU_LENGTH = -0.14;
		 IMU_HEIGHT = 0.0;
		 IMU_LENGTH = 0.0;
	 }

	temp_position[0] = x;
	temp_position[1] = y;
	temp_position[2] = z;
	temp_position[3] = roll;
	temp_position[4] = pitch;
	temp_position[5] = yaw;

	/* 三次元の座標変換式 */
	trans(0, 0) = cos(temp_position[4]) * cos(temp_position[5]);
	trans(1, 0) = cos(temp_position[4]) * sin(temp_position[5]);
	trans(2, 0) = -sin(temp_position[4]);
	trans(0, 1) = sin(temp_position[3]) * sin(temp_position[4]) * cos(temp_position[5]) - cos(temp_position[3]) * sin(temp_position[5]);
	trans(1, 1) = sin(temp_position[3]) * sin(temp_position[4]) * sin(temp_position[5]) + cos(temp_position[3]) * cos(temp_position[5]);
	trans(2, 1) = sin(temp_position[3]) * cos(temp_position[4]);
	trans(0, 2) = cos(temp_position[3]) * sin(temp_position[4]) * cos(temp_position[5]) + sin(temp_position[3]) * sin(temp_position[5]);
	trans(1, 2) = cos(temp_position[3]) * sin(temp_position[4]) * sin(temp_position[5]) - sin(temp_position[3]) * cos(temp_position[5]);
	trans(2, 2) = cos(temp_position[3]) * cos(temp_position[4]);
	robot_b(0, 0) = -IMU_LENGTH;
	robot_b(1, 0) = -IMU_WIDTH;
	robot_b(2, 0) = -IMU_HEIGHT;

	/* 座標変換の行列計算 */
	robot_w = trans * robot_b;

	/* 地上固定座標系に変換 */
	g_robot.SetRobotX(robot_w(0, 0)+ temp_position[0]);
	g_robot.SetRobotY(robot_w(1, 0)+ temp_position[1]);
	g_robot.SetRobotZ(robot_w(2, 0) + temp_position[2]);
	g_robot.SetRobotRoll(roll);
	g_robot.SetRobotPitch(pitch);
	g_robot.SetRobotYaw(yaw);
}

/*//////////////////////////////////////////////////////
観測点をMLSの垂直角が大きい順に並べ替える引数
観測点数
返り値
なし
/////////////////////////////////////////////////////*/
void Point_Sort(int mlspointsum) {
	int i, j, d, e;

	/*観測点を並べ替えて他の配列に保存*/
	for (i = 0; i < mlspointsum; i++) {
		d = 0;
		e = 0;
		for (j = 0; j < LAYER; j++) {
			if (j % 2 == 0) {
				g_laserpoint2[j - d][i].distance = g_laserpoint[j][i].distance;
				g_laserpoint2[j - d][i].x_l = g_laserpoint[j][i].x_l;
				g_laserpoint2[j - d][i].y_l = g_laserpoint[j][i].y_l;
				g_laserpoint2[j - d][i].z_l = g_laserpoint[j][i].z_l;
				d++;
			}
			else {
				g_laserpoint2[j + 15 - e][i].distance = g_laserpoint[j][i].distance;
				g_laserpoint2[j + 15 - e][i].x_l = g_laserpoint[j][i].x_l;
				g_laserpoint2[j + 15 - e][i].y_l = g_laserpoint[j][i].y_l;
				g_laserpoint2[j + 15 - e][i].z_l = g_laserpoint[j][i].z_l;
				e++;
			}

		}
	}
	/*保存した観測点をもとの配列に戻す*/
	for (i = 0; i < mlspointsum; i++) {
		for (j = 0; j < LAYER; j++) {
			g_laserpoint[j][i].distance = g_laserpoint2[j][i].distance;
			g_laserpoint[j][i].x_l = g_laserpoint2[j][i].x_l;
			g_laserpoint[j][i].y_l = g_laserpoint2[j][i].y_l;
			g_laserpoint[j][i].z_l = g_laserpoint2[j][i].z_l;
		}
	}
}

/*//////////////////////////////////////////////////////////////////////
観測点出力
引数
int：
返り値
なし
//////////////////////////////////////////////////////////////////////*/
void OutputObservationPoint(int mlspointsum) {
	int i, j;

	for (i = 0; i < LAYER; i++) {
		for (j = 0; j < mlspointsum; j++) {
			/* 観測点保存 */
			if ((-135.0 < g_laserpoint[i][j].x && g_laserpoint[i][j].x < -130.0) && (-50.0 < g_laserpoint[i][j].y && g_laserpoint[i][j].y < -45.0)) {
				fprintf_s(pointcloud, "%lf,%lf,%lf\n", g_laserpoint[i][j].x, g_laserpoint[i][j].y, g_laserpoint[i][j].z);
			}
		}
	}
}

/*//////////////////////////////////////////////////////////////////////
追跡結果出力
引数
int：追跡物体数，int：ループ回数
返り値
なし
//////////////////////////////////////////////////////////////////////*/
void OutputTrack(int track_num, int loop) {
	int i, j = 0;
	double v;           /* 速度 */
	double distance;    /* 距離 */

	for (i = 0; i < track_num; i++) {
		if (tracking_object[i].trackflag == 1
			) {
			if (tracking_object[i].trackonlyflag == 0
				|| tracking_object[i].trackonlyflag == 1
				) {
				//if (g_gridmap[VoteCell(tracking_object[i].predict_x[0], robotcell_x)][VoteCell(tracking_object[i].predict_x[2], robotcell_y)].planeflag == 0){
				/* 速度(km/h) */
				v = sqrt((tracking_object[i].predict_x[1] * tracking_object[i].predict_x[1])
					+ (tracking_object[i].predict_x[3] * tracking_object[i].predict_x[3])
					+ (tracking_object[i].predict_x[5] * tracking_object[i].predict_x[5])) * 3.6;
				/* 直線距離算出 */
				distance = sqrt((tracking_object[i].predict_x[0] * tracking_object[i].predict_x[0])
					+ (tracking_object[i].predict_x[2] * tracking_object[i].predict_x[2])
					+ (0 * 0));
				/* 追跡結果出力 */
				for (j = 0; j < NUM_OUTPUT; j++) {
					if (tracking_object[i].number == output_number[j]) {
						/* x座標[m]，x速度[m/s]，y座標[m]，y速度[m/s]，z座標[m]，z速度[m/s]，直線距離[m]，ループ回数，幅，長さ，高さ，速度 */
						fprintf(output[j], "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%lf,%lf,%lf,%lf\n",
							tracking_object[i].predict_x[0], tracking_object[i].predict_x[1], tracking_object[i].predict_x[2],
							tracking_object[i].predict_x[3], tracking_object[i].predict_x[4], tracking_object[i].predict_x[5],
							distance, loop, tracking_object[i].current_size[0], tracking_object[i].current_size[1], tracking_object[i].height, v);
					}
					//}
				}
			}
			else {
				for (j = 0; j < NUM_OUTPUT; j++) {
					if (tracking_object[i].number == output_number[j]) {
						/* x座標[m]，x速度[m/s]，y座標[m]，y速度[m/s]，z座標[m]，z速度[m/s]，直線距離[m]，ループ回数，幅，長さ，高さ，速度 */
						fprintf(output[j], "\n");
					}
				}
			}
		}
	}
}



//道路端（縁石）観測点の位置情報を出力する
void Output_CurbPoint(int mlspointsum) {
	int i, j, k, l;
	int alone;

	for (k = 0; k < mlspointsum; ++k)
	{
		for (l = 0; l < LAYER; ++l)
		{
			Laserpoint &lp = g_laserpoint[l][k];
			lp.moveflag = 0;
		}
	}


	//道路端と判定された観測点の中から移動物体の構成点を除去する
	for (i = GRIDMAP_SIZE_MARGIN; i < GRIDMAP_SIZE + GRIDMAP_SIZE_MARGIN; i++) {
		for (j = GRIDMAP_SIZE_MARGIN; j < GRIDMAP_SIZE + GRIDMAP_SIZE_MARGIN; j++) {
			if (g_gridmap[i][j].molabel != -1) {
				for (k = 0; k < mlspointsum; ++k)
				{
					for (l = 0; l < LAYER; ++l)
					{
						Laserpoint &lp = g_laserpoint[l][k];
						if (lp.x_ongrid == i && lp.y_ongrid == j)
						{
							lp.moveflag = 1;
						}
					}
				}
			}
		}
	}

	for (i = 0; i < mlspointsum; ++i)
	{
		for (j = 0; j < LAYER; ++j)
		{
			Laserpoint &lp = g_laserpoint[j][i];
			if (lp.hor_classification == Laserpoint::H_HIGH && lp.horizontal_distance < 30 && lp.moveflag != 1)
			{
				alone = 0;
				for (k = -10; k < 10; k++)
				{
					if (i + k < 0 || i + k >= mlspointsum)
						continue;
					Laserpoint &far_lp = g_laserpoint[j + 1][i + k];

					if (far_lp.moveflag != 1 && far_lp.hor_classification == Laserpoint::H_HIGH)
						alone++;
				}
				if (alone >= 1) {
					fprintf(outputcurb, "%lf,%lf\n", lp.x, lp.y);
				}
				break;
			}
		}
	}


}


/*//////////////////////////////////////////////////////////////////////
エラー処理関数
引数
errno_t：エラー番号
返り値
なし
//////////////////////////////////////////////////////////////////////*/
void FileOpenError(errno_t error) {
	std::cerr << "error num " << error << std::endl;
	getchar();        /* 一つだと止まらない */
	getchar();
	exit(error);
}