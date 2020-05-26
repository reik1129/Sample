#define _CRT_SECURE_NO_WARNINGS             /* Eigen�̃C���N���[�h��opencv���Ă�ł���H */
/* Eigen�̃R�A�@�\�ƃW�I���g���@�\���g�� */
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
const int NUM_OUTPUT = 10;               /* �o�͂��鐔 */
const int NUMSET = 12;                                   /* 1�p�P�b�g�ɑ΂��ē����Ă���32�̊ϑ��_�̃Z�b�g�� */
const int LINEAR_INTERPOLATION_FLAG = 1;                 /* ���`��Ԃ��s�����ǂ����̃t���O�C1�F����Cother�F�Ȃ� */
const std::string  FOLDER_NAME = "Output/World";						//pcd�t�@�C���̃f�B���N�g��

FILE *result;             /* GNSS/INS�������u���瓾����f�[�^�̃t�@�C���|�C���^ */
FILE *pointcloud;
FILE *out;
FILE *outputcurb;
FILE *output[NUM_OUTPUT];
FILE *robot;
FILE *miki;
FILE *csm;             /* IMU���瓾����f�[�^�̃t�@�C���|�C���^ */
errno_t error;
std::ifstream ifs;

//pcl::visualization::CloudViewer viewer("Viewer");
Laserpoint g_laserpoint[LAYER][UPPERLIMIT_PSUM];        /* ���[�U�ϑ��_�̃N���X�C���[�J���Ő錾����ƃ��������I�[�o�[���邽�� */
Laserpoint g_laserpoint2[LAYER][UPPERLIMIT_PSUM];        /* ���[�U�ϑ��_�̃N���X�C���[�J���Ő錾����ƃ��������I�[�o�[���邽�� */
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI> map_cloud;
pcl::PointCloud<pcl::PointXYZI> starget_cloud;
std::vector<pcl::PointCloud<pcl::PointXYZI>> reference_smap_vector;
int output_number[NUM_OUTPUT] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };  /* �ǐՌ��ʂ��o�͂���ǐՕ��̔ԍ� */
int file_position = 0;                  /* ���[�J���ɂ���ƕ`�摬�x���x���Ȃ� */
long diff_mlstime = 0;                  /* IMU�̍ŏ��̃f�[�^�̎�����MLS�̍ŏ��̃^�C���X�^���v�̍� */
long mls_starttime = 0;                 /* MLS�̍ŏ��̃p�P�b�g������ꂽ������timeGetTime()�̒l */
long prev_localization_time = 0;        /* �O�����ɂ����鎩�Ȏp������f�[�^�̃^�C���X�^���v */
char folder_path[NAME_ARREY_SIZE];      /* �t�@�C���̂���t�H���_�̃p�X */
double before_position[NUM_USE_LINEAR_INTERPOLATION_VARIABLES] = { 0 };    /* �O���[�v�ł̎��Ȏp����� */

extern int g_numdata;            /* �`�悷��f�[�^�ԍ� */

								 
void outputrobot(double now_position[], double v) {
	fprintf(robot, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
		now_position[0], now_position[1], now_position[2], now_position[3], now_position[4], now_position[5],v);
}
void Outputmiki(double x,double y) {
	fprintf(miki, "%lf,%lf\n",x,y);
}
//�}�b�v����
void savemap(pcl::PointCloud<pcl::PointXYZI>::Ptr target_scan) {
	int i;
	
	map_cloud += *target_scan;
	//viewer.showCloud(target_scan);
	//make output folder
	_mkdir("Output");

	//Save map_cloud
	pcl::io::savePCDFileBinary("Output/map_cloud.pcd", map_cloud);
	
	/*
	map_cloud += *target_scan;//�n�}+=�Q�ƒn�}
	viewer.showCloud(target_scan);
	for (i = 1; i < 10; i++) {
		if (g_loopcount == 1000 * i) {//1000�~10=10000�܂őΉ�
			std::cout << "AAAA" << std::endl;//�ʂ��Ă邩�m�F OK
			//Save map_cloud
			//pcl::PointCloud<pcl::PointXYZI> map_cloud;
			pcl::io::savePCDFileBinary("Output/map_cloud.pcd", map_cloud);//PCL���C�u������map_cloud.pcd���o��
		}
	}
	*/


}
/*//////////////////////////////////////////////////////
�t�@�C���I�[�v�����s���֐�
����
�Ȃ�
�Ԃ�l
�Ȃ�
//////////////////////////////////////////////////////*/
void MicroevFileopen(void) {
	int i;
	FILE *velostarttime;

	/* �t�@�C�����ۑ��z�� */
	char mls_starttime_name[NAME_ARREY_SIZE];        /* velo_start_time.dat��GNSS/INS�������u�̍ŏ��̃p�P�b�g�������Ă����Ƃ��̃p�\�R���̎��� */
	char result_name[NAME_ARREY_SIZE];              /* result.txt��GNSS/INS�������u�̃f�[�^ */
	char output_name[NAME_ARREY_SIZE];
	char csm_file_name[NAME_ARREY_SIZE];

	/* �t�@�C���̂���t�H���_�̃p�X��ݒ� */
	sprintf_s(folder_path, NAME_ARREY_SIZE, "InputData");

	errno_t error;
	if (IMU_DATA == 1) {
		/* �t�@�C������ݒ� */
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
		/* �ŏ��̃p�P�b�g�������Ă������_�ł�timeGT()�̒l���擾 */
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

	/* �t�@�C���I�[�v�� */
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
GNSS/INS�����q�@���u�̃f�[�^�X�L�b�v�֐�
����
�Ȃ�
�Ԃ�l
�Ȃ�
//////////////////////////////////////////////////////*/
void DataSkip(void) {
	/* IMU�̃f�[�^�ǂݍ��ݕϐ� */
	int intdum = 0;
	double doubledum[13] = { 0 };

	/* GPS�^�C���@�ܓx�@�o�x�@�����@roll���x�@�s�b�`���x�@���[���x�@roll pitch yaw x���W y���W z���W�@�^�C���X�^���v �v14�� */
	fscanf_s(result, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %d\t\n",
		&doubledum[0], &doubledum[1], &doubledum[2], &doubledum[3], &doubledum[4], &doubledum[5], &doubledum[6],
		&doubledum[7], &doubledum[8], &doubledum[9], &doubledum[10], &doubledum[11], &doubledum[12], &intdum);
}


/*//////////////////////////////////////////////////////
microEV�p�t�@�C���N���[�Y
����
�Ȃ�
�Ԃ�l
�Ȃ�
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
MLS�̃f�[�^�ǂݍ��݊֐�
����
const int�FGNSS/INS�����q�@���u�̃^�C���X�^���v
�Ԃ�l
int�F
//////////////////////////////////////////////////////*/
int MlsDataRead(const int i_timestamp) {
	const int NUM_PACKET = 1206;                /* MLS�̃p�P�b�g�̃o�C�g�� */
	const int USE_OBSERVATION_POINT = 100;      /* ���Ȏp�����ɑ΂��ėp����ϑ��_�̎��ԁims�j */

	int i = 0, j = 0;
	int mlspointsum = 0;                        /* �ǂݍ��񂾊ϑ��_�� */
	int packetcount = 0;                        /* �ǂݍ��񂾃p�P�b�g�� */
	unsigned int ts_mls = 0;                    /* MLS�̃^�C���X�^���v */
	long localization_time = i_timestamp;       /* �������ɂ����鎩�Ȏp���f�[�^�̃^�C���X�^���v�@���̂܂��-100ms�`0ms�̊ϑ��_���g�p */

	char g_mlsDataName[NAME_ARREY_SIZE] = { 0 };        /* MLS�̃f�[�^���z�� */
	sprintf_s(g_mlsDataName, NAME_ARREY_SIZE, "%s\\%d_hdl_data.bin", folder_path, g_numdata);

	FILE *mls;
	errno_t error;

	if ((error = fopen_s(&mls, g_mlsDataName, "rb")) != 0) {
		std::cout << "_mls_data.bin file open error!" << std::endl;
		FileOpenError(error);
	}
	fseek(mls, file_position, SEEK_SET);        /* �t�@�C���̈ʒu�w���q���ړ� */

	char *velo_buf = new char[NUM_PACKET];        /* mls�ǂݍ��ݗp�o�b�t�@�̃������̈�m�� */

												  /* �����X�L�����ɂ����鏈�� �����܂�1�p�P�b�g���擾���ă^�C���X�^���v���Q�� */
	if (g_loopcount == 0) {
		fread(velo_buf, sizeof(char), NUM_PACKET, mls);        /* 1�p�P�b�g�����o�b�t�@�ɓǂݏo�� */
															   /* �r�b�g�V�t�g�Ń^�C���X�^���v���擾 */
		ts_mls = ((unsigned int)((((unsigned int)((unsigned char)velo_buf[NUM_PACKET - 3])) << 24) + (((unsigned int)((unsigned char)velo_buf[NUM_PACKET - 4])) << 16)
			+ (((unsigned int)((unsigned char)velo_buf[NUM_PACKET - 5])) << 8) + (unsigned char)velo_buf[NUM_PACKET - 6]));
		/* �p�P�b�g�̃^�C���X�^���v(�}�C�N���b)��tGT(�~���b)�Ƃ̑Ή������ */
		diff_mlstime = mls_starttime - MicroToMilli(ts_mls);
	}

	while (1) {
		/* �P�p�P�b�g�����o�b�t�@�ɓǂݏo�� */
		fread(velo_buf, sizeof(char), NUM_PACKET, mls);

		/* �ǂݏo�����p�P�b�g�̃^�C���X�^���v���Q�� */
		/* �r�b�g�V�t�g�Ń^�C���X�^���v���擾 */
		ts_mls = ((unsigned int)((((unsigned int)((unsigned char)velo_buf[NUM_PACKET - 3])) << 24) + (((unsigned int)((unsigned char)velo_buf[NUM_PACKET - 4])) << 16)
			+ (((unsigned int)((unsigned char)velo_buf[NUM_PACKET - 5])) << 8) + (unsigned char)velo_buf[NUM_PACKET - 6]));
		//std::cout << "mlstimestamp" << localization_time << std::endl;
		/* ���Ȏp�����ɑΉ�����ϑ��_�݂̂��擾 */
		/* ���Ȏp�����̃^�C���X�^���v��100 [ms]���ߋ��ɓ���ꂽ�ϑ��_�@���@�������Ȃ��i���݂̎��Ȏp�����ɑΉ�����Ƃ���܂œǂݔ�΂��j */
		if (MicroToMilli(ts_mls) < (unsigned int)(localization_time - diff_mlstime) - USE_OBSERVATION_POINT) {
		}
		/* ���Ȏp�����̃^�C���X�^���v��薢���ɓ���ꂽ�ϑ��_�@���@����ȏ�ǂݍ���ł����݂̎��Ȏp�����ɑΉ�����ϑ��_�͓����Ȃ��̂ŁC�����œǂݍ��݂��I�� */
		else if (MicroToMilli(ts_mls) > (unsigned int)(localization_time - diff_mlstime)) {
			break;
		}
		/* ���Ȏp�����̃^�C���X�^���v�̑O��50ms�̊ϑ��_�̂ݍ̗p */
		else {
			/* MLS�̃f�[�^��z��ɑ�� */
			packetcount = AssignmentToLaserpoint(ts_mls, velo_buf, packetcount);
		}

		////CSM�����i���艟������j
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

	mlspointsum = packetcount * NUMSET;        /* 1�p�P�b�g�ɂ�32�̊ϑ��_��12�Z�b�g�����Ă��� */

	file_position = ftell(mls);
	fclose(mls);
	delete[] velo_buf;                /* ���I�z��̃�������� */

	return mlspointsum;
}


/*//////////////////////////////////////////////////////
MLS�̃f�[�^���N���X�ɑ��
����
�Ȃ�
�Ԃ�l
int�F
//////////////////////////////////////////////////////*/
int AssignmentToLaserpoint(unsigned int i_ts_mls, char *i_velo_buf, int o_packetcount) {

	int i = 0, j = 0;
	double rot_angle = 0;                     /* �ϑ��_�̊p�x */

	
	/* �����p�x�ɂ�����sin�̒l */
	const double sin_siita[32] = { -0.5101, -0.1621, -0.4898, -0.1392, -0.4695,
		-0.1160, -0.4487, -0.0929, -0.4278, -0.0698,
		-0.4067, -0.0466, -0.3854, -0.0232, -0.3637,
		0.0, -0.3420, 0.0232, -0.3201, 0.0466,
		-0.2979, 0.0698, -0.2756, 0.0929, -0.2533,
		0.1162, -0.2306, 0.1392, -0.2079, 0.1621,
		-0.1852, 0.1852 };
	/* �����p�x�ɂ�����cos�̒l */
	const double cos_siita[32] = { 0.8601, 0.9868, 0.8718, 0.9903, 0.8829,
		0.9933, 0.8937, 0.9957, 0.9039, 0.9976,
		0.9135, 0.9989, 0.9227, 0.9997, 0.93151,
		1.0, 0.9397, 0.9997, 0.9474, 0.9989,
		0.9546, 0.9976, 0.9613, 0.9957, 0.9674,
		0.9932, 0.9731, 0.9903, 0.9781, 0.9868,
		0.9827, 0.9827 };
	/* MLS��SPAN�̈ʒu�֌W�̐ݒ肪�~�X���Ă��邽�ߕ␳����l */


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
		PITCH_CORRECTION = 1.5;     /*MLS��pitch�p��1.5������Ă��関�v��*/
		//TRANSFORMATION_TO_ROBOT_COORDINATE = 90.0;  
	}
	else if (Vehicle_Type == 2) {
		SPAN_TO_MLS_X = -0.245;      /* �����l0.4475   ����l0.6     �␳�l -0.245     �v�Z�l -0.31 */
		SPAN_TO_MLS_Y = -0.38;       /* �����l0.2      ����l0.1     �␳�l -0.38      �v�Z�l -0.46 */
		SPAN_TO_MLS_Z = -1.95;       /* �����l1.9      ����l1.8     �␳�l 0.6        �v�Z�l 0.93 , �ύX�O-1.5 */
		YAW_CORRECTION = 0.0;        /* MLS��yaw�p��2������Ă���i�炵���D���v���j */
		PITCH_CORRECTION = 0.0;
		//TRANSFORMATION_TO_ROBOT_COORDINATE = 90.0;    /* �Z���T���W�n���烍�{�b�g���W�n�ɕϊ�����l�D�����_�ł�90���v���X�����ɉ�]���Ă��� */
	}

	for (i = 0; i < NUMSET; i++) {
		/* �ϑ��_�̊p�x���p�P�b�g����r�b�g�V�t�g�Ŏ擾 */
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
�񎟌��̐��`��Ԃ����W�ϊ��֐�
����
�Ȃ�
�Ԃ�l
�Ȃ�
//////////////////////////////////////////////////////*/
void LinearInterpolationTwoDimention(const int i_timestamp, double i_robotX, double i_robotY, double i_robotYaw, int i_mlspointsum) {
	const double NOT_ZERO_CONSTANT = 0.000000001;           /* time_diff��0�ƂȂ����Ƃ���0���Z�ƂȂ�Ȃ��悤�ɂ���萔 */
	const double DISTANCE_BETWEEN_CENTER_AND_MLS = 0.0;     /* �ԗ����S����MLS�ݒu�_�܂ł̋��� */

	int i = 0, j = 0;
	long time_diff = 0;                          /* ���Ȏp��������擾�����Ԋu */
	long localization_time = i_timestamp;        /* �������ɂ����鎩�Ȏp���f�[�^�̃^�C���X�^���v�@���̂܂��-100ms�`0ms�̊ϑ��_���g�p */
	double linear_interpolation_gradient[3] = { 0 };        /* ���`��ԗp�̌X���C1�Fx�C2�Fy�C3�Fyaw */
	double temp_time = 0;                        /* �ϑ��_������ꂽ�Ƃ��̎��ԂƑO�����̎��Ȏp����񂪓���ꂽ���Ԃ̍� */
	double temp_position[3] = { 0 };             /* ��Ԃ������{�b�g�ʒu */
	double now_position[3] = { i_robotX, i_robotY, i_robotYaw };    /* �������ɂ����郍�{�b�g�ʒu�C1�Fx�C2�Fy�C3�Fyaw */
	double temp_x, temp_y;

	/* �O�����ƌ������̊ԂɈړ������X������`�ŎZ�o */
	for (i = 0; i < 2; i++) {
		linear_interpolation_gradient[i] = (now_position[i] - before_position[i]) / ((double)(localization_time - prev_localization_time) + NOT_ZERO_CONSTANT);
	}

	/* �p�x��0����2�΂܂ł͈̔͂Ȃ̂ł��̓]���_�̏��� */
	if (now_position[2] - before_position[2] > PI) {
		linear_interpolation_gradient[2] = (now_position[2] - before_position[2] - 2 * PI) / ((double)(localization_time - prev_localization_time) + NOT_ZERO_CONSTANT);
	}
	else if (now_position[2] - before_position[2] < -PI) {
		linear_interpolation_gradient[2] = (now_position[2] - before_position[2] + 2 * PI) / ((double)(localization_time - prev_localization_time) + NOT_ZERO_CONSTANT);
	}
	else {
		linear_interpolation_gradient[i] = (now_position[i] - before_position[i]) / ((double)(localization_time - prev_localization_time) + NOT_ZERO_CONSTANT);
	}

	/* �ϑ��_�̃^�C���X�^���v�ƎZ�o�����X�����炻�̃^�C���X�^���v�̂Ƃ��̎p�����Ԃ��� */
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
			/* �|�C���g�N���E�h�����{�b�g���W�n�ɕϊ� */
			//temp_x = DISTANCE_BETWEEN_CENTER_AND_MLS + g_laserpoint[j][i].x_l * cos(0.5 * PI) - g_laserpoint[j][i].y_l * sin(0.5 * PI);
			//temp_y = g_laserpoint[j][i].x_l * sin(0.5 * PI) + g_laserpoint[j][i].y_l * cos(0.5 * PI);
			temp_x = g_laserpoint[j][i].x_l;
			temp_y = g_laserpoint[j][i].y_l;
			/* �|�C���g�N���E�h��n��Œ���W�n�ɕϊ� */
			g_laserpoint[j][i].x = temp_position[0] + temp_x * cos(temp_position[2]) - temp_y * sin(temp_position[2]);
			g_laserpoint[j][i].y = temp_position[1] + temp_x * sin(temp_position[2]) + temp_y * cos(temp_position[2]);
			g_laserpoint[j][i].z = g_laserpoint[j][i].z_l;
		}
	}

	/* �O�����̏����X�V */
	for (i = 0; i < 3; i++) {
		before_position[i] = now_position[i];
	}
	prev_localization_time = localization_time;
}


/*//////////////////////////////////////////////////////
�O�����̐��`��Ԃ����W�ϊ��֐�
����
const int�FGNSS/INS�����q�@���u�̃^�C���X�^���v�C
double[]�F���{�b�g�̎p���i0�Fx�C1�Fy�C2�Fz�C3�Froll�C4�Fpitch�C5�Fyaw�j�Cint�F1�w�̊ϑ��_��
�Ԃ�l
�Ȃ�
//////////////////////////////////////////////////////*/
void LinearInterpolationThreeDimention(const int i_timestamp, double now_position[], int i_mlspointsum) {
	const double NOT_ZERO_CONSTANT = 0.000000001;           /* time_diff��0�ƂȂ����Ƃ���0���Z�ƂȂ�Ȃ��悤�ɂ���萔 */
	const double DISTANCE_BETWEEN_CENTER_AND_MLS = 0.0;     /* �ԗ����S����MLS�ݒu�_�܂ł̋��� */
	const int START_DEGREE = 3;                             /* position�s��̊p�x�̃f�[�^���n�܂�ʒu x,y,z,roll,pitch,yaw������3 */
	const int END_DEGREE = 6;                               /* position�s��̊p�x�̃f�[�^���I���ʒu x,y,z,roll,pitch,yaw������6 */
	int count = 0;
	int i = 0, j = 0;
	long localization_time = i_timestamp;                /* �������ɂ����鎩�Ȏp���f�[�^�̃^�C���X�^���v�@���̂܂��-100ms�`0ms�̊ϑ��_���g�p */
	double linear_interpolation_gradient[NUM_USE_LINEAR_INTERPOLATION_VARIABLES] = { 0 };        /* ���`��ԗp�̌X���C1�Fx�C2�Fy�C3�Fyaw */
	double temp_time = 0;                                /* ��Ԃ���ϑ��_�̎��Ԃ�SPAN-CPT�̎��Ԃ̑Ή� */
	double temp_position[NUM_USE_LINEAR_INTERPOLATION_VARIABLES] = { 0 };                        /* ��Ԃ������{�b�g�ʒu */

	Eigen::MatrixXd point_b = Eigen::MatrixXd::Zero(3, 1);
	Eigen::MatrixXd point_w = Eigen::MatrixXd::Zero(3, 1);
	Eigen::MatrixXd transform = Eigen::MatrixXd::Zero(3, 3);

	///* �O�����ƌ������̊ԂɈړ������X������`�ŎZ�o */
	for (i = 0; i < START_DEGREE; i++) {
		linear_interpolation_gradient[i] = (now_position[i] - before_position[i]) / ((double)(localization_time - prev_localization_time) + NOT_ZERO_CONSTANT);
	}

	/* �p�x��0����2�΂܂ł͈̔͂Ȃ̂ł��̓]���_�̏��� */
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
			/* �O�����̎��Ԃ���ϑ��_������ꂽ���Ԃ܂ł��Z�o */
			temp_time = (double)(((long)(int)g_laserpoint[0][i].timestump) + (MilliToMicro(diff_mlstime - prev_localization_time)));
			/* �O�����ƌ������̊ԂɈړ������X������`�ŎZ�o */
			for (j = 0; j < NUM_USE_LINEAR_INTERPOLATION_VARIABLES; j++) {
				temp_position[j] = (linear_interpolation_gradient[j] * MicroToMilliDouble(temp_time)) + before_position[j];
			}
		}
		else {
			for (j = 0; j < NUM_USE_LINEAR_INTERPOLATION_VARIABLES; j++) {
				temp_position[j] = now_position[j];
			}
		}

	
		

		/* �O�����̍��W�ϊ��� */
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
				/* ���W�ϊ��̍s��v�Z */
				point_w = transform * point_b;
			}

			/* �|�C���g�N���E�h��n��Œ���W�n�ɕϊ� */
			g_laserpoint[j][i].x = point_w(0, 0) + temp_position[0];
			g_laserpoint[j][i].y = point_w(1, 0) + temp_position[1];
			g_laserpoint[j][i].z = point_w(2, 0) + temp_position[2];
		}
	}

	/* �O�����̏����X�V */
	for (i = 0; i < NUM_USE_LINEAR_INTERPOLATION_VARIABLES; i++) {
		before_position[i] = now_position[i];
	}
	prev_localization_time = localization_time;
}

/*//////////////////////////////////////////////////////
���{�b�g���W�n���_�Z�o
����
IMU�f�[�^
�Ԃ�l
�Ȃ�
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

	/* �O�����̍��W�ϊ��� */
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

	/* ���W�ϊ��̍s��v�Z */
	robot_w = trans * robot_b;

	/* �n��Œ���W�n�ɕϊ� */
	g_robot.SetRobotX(robot_w(0, 0)+ temp_position[0]);
	g_robot.SetRobotY(robot_w(1, 0)+ temp_position[1]);
	g_robot.SetRobotZ(robot_w(2, 0) + temp_position[2]);
	g_robot.SetRobotRoll(roll);
	g_robot.SetRobotPitch(pitch);
	g_robot.SetRobotYaw(yaw);
}

/*//////////////////////////////////////////////////////
�ϑ��_��MLS�̐����p���傫�����ɕ��בւ������
�ϑ��_��
�Ԃ�l
�Ȃ�
/////////////////////////////////////////////////////*/
void Point_Sort(int mlspointsum) {
	int i, j, d, e;

	/*�ϑ��_����בւ��đ��̔z��ɕۑ�*/
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
	/*�ۑ������ϑ��_�����Ƃ̔z��ɖ߂�*/
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
�ϑ��_�o��
����
int�F
�Ԃ�l
�Ȃ�
//////////////////////////////////////////////////////////////////////*/
void OutputObservationPoint(int mlspointsum) {
	int i, j;

	for (i = 0; i < LAYER; i++) {
		for (j = 0; j < mlspointsum; j++) {
			/* �ϑ��_�ۑ� */
			if ((-135.0 < g_laserpoint[i][j].x && g_laserpoint[i][j].x < -130.0) && (-50.0 < g_laserpoint[i][j].y && g_laserpoint[i][j].y < -45.0)) {
				fprintf_s(pointcloud, "%lf,%lf,%lf\n", g_laserpoint[i][j].x, g_laserpoint[i][j].y, g_laserpoint[i][j].z);
			}
		}
	}
}

/*//////////////////////////////////////////////////////////////////////
�ǐՌ��ʏo��
����
int�F�ǐՕ��̐��Cint�F���[�v��
�Ԃ�l
�Ȃ�
//////////////////////////////////////////////////////////////////////*/
void OutputTrack(int track_num, int loop) {
	int i, j = 0;
	double v;           /* ���x */
	double distance;    /* ���� */

	for (i = 0; i < track_num; i++) {
		if (tracking_object[i].trackflag == 1
			) {
			if (tracking_object[i].trackonlyflag == 0
				|| tracking_object[i].trackonlyflag == 1
				) {
				//if (g_gridmap[VoteCell(tracking_object[i].predict_x[0], robotcell_x)][VoteCell(tracking_object[i].predict_x[2], robotcell_y)].planeflag == 0){
				/* ���x(km/h) */
				v = sqrt((tracking_object[i].predict_x[1] * tracking_object[i].predict_x[1])
					+ (tracking_object[i].predict_x[3] * tracking_object[i].predict_x[3])
					+ (tracking_object[i].predict_x[5] * tracking_object[i].predict_x[5])) * 3.6;
				/* ���������Z�o */
				distance = sqrt((tracking_object[i].predict_x[0] * tracking_object[i].predict_x[0])
					+ (tracking_object[i].predict_x[2] * tracking_object[i].predict_x[2])
					+ (0 * 0));
				/* �ǐՌ��ʏo�� */
				for (j = 0; j < NUM_OUTPUT; j++) {
					if (tracking_object[i].number == output_number[j]) {
						/* x���W[m]�Cx���x[m/s]�Cy���W[m]�Cy���x[m/s]�Cz���W[m]�Cz���x[m/s]�C��������[m]�C���[�v�񐔁C���C�����C�����C���x */
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
						/* x���W[m]�Cx���x[m/s]�Cy���W[m]�Cy���x[m/s]�Cz���W[m]�Cz���x[m/s]�C��������[m]�C���[�v�񐔁C���C�����C�����C���x */
						fprintf(output[j], "\n");
					}
				}
			}
		}
	}
}



//���H�[�i���΁j�ϑ��_�̈ʒu�����o�͂���
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


	//���H�[�Ɣ��肳�ꂽ�ϑ��_�̒�����ړ����̂̍\���_����������
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
�G���[�����֐�
����
errno_t�F�G���[�ԍ�
�Ԃ�l
�Ȃ�
//////////////////////////////////////////////////////////////////////*/
void FileOpenError(errno_t error) {
	std::cerr << "error num " << error << std::endl;
	getchar();        /* ����Ǝ~�܂�Ȃ� */
	getchar();
	exit(error);
}