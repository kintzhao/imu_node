#ifndef __IMU_DATA_DECODE_H__
#define __IMU_DATA_DECODE_H__
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>


int imu_data_decode_init(void);
int get_raw_acc(int16_t* a);
int get_raw_gyo(int16_t* g);
int get_raw_mag(int16_t* m);
int get_id(uint8_t *user_id);
int get_eular(float* e);
int get_quat(float* q);

bool flag_one_frame=false;

#ifdef __cplusplus
}
#endif

#endif


