#ifndef __MAIN_H__
#define __MAIN_H__

#include "zf_common_headfile.h"

void car_protect(void)

#define steer PWM4_MODULE3_CHA_C31

#define motor1 PWM1_MODULE3_CHA_D0
#define motor2 PWM2_MODULE3_CHB_D3

#define N 1.1
#define speed 120

    float ADC_Memory[7][3];
float L, LV, M, RV, R, L2, R2;
float offset[2];
float Steer_Mid = 21705; // 舵机中值, 使用前先测试
float STEER_PWM = 0;

float STEER_P = 0.0001;
float STEER_D = 1;

float MOTOR_P = 50;
float MOTOR_I = 100;
float MOTOR_D = 1;

float DIF_ERR[2];
float DIF_delta;

float k = 0.75; // 竖直电感占比系数
float encoder[2][3];
float g_fReal_Speed[2];
float g_fSet_Speed[2] = {speed, speed};
float SPEED_ERR[2][3];

float g_fMOTOR_PWM[2] = {0, 0};
float UD[2][3] = {0, 0};
float Y[2][2] = {0, 0};
float gama = 0.4;

uint8_t image[MT9V03X_H][MT9V03X_W];
uint8_t send_buff[8];
extern uint8_t mt9v03x_csi_finish_flag;

int rjudge = 0;

float huandao = 0;
float start = 0;
int brake = 150;

bool stop = 0;
uint8_t tingcheban = 1;
float tingche = 0;

//**********camera*********
uint8_t ZHONG[120] = {94};
uint8_t ZUO[120] = {0};
uint8_t YOU[120] = {187};

uint8_t QIANZHAN = 30;
uint8_t YUAN, ZHONGJIAN, JIN;
uint8_t err = 0, Yerr = 0;
// float camera_P=1;
// float camera_D=1;
// float camera_KP=1;

#endif
