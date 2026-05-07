#include "dev_dm_imu.h"
#include "drv_can.h"
#include "drv_math.h"
#include "drv_imu.h"
#include <string.h>

imu_t imu = {0};

#define IMU_DATA_REQUEST_STDID 0x6FFu

int float2_to_uint(float x_float, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint2_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void imu_init(uint8_t can_id, uint8_t mst_id, CAN_HandleTypeDef *hcan)
{
	imu.can_id = can_id;
	imu.mst_id = mst_id;
	imu.can_handle = hcan;
}

static void imu_send_cmd(uint8_t reg_id, uint8_t ac, uint32_t data)
{
	uint8_t buf[8] = {0xCC, reg_id, ac, 0xDD, 0, 0, 0, 0};

	memcpy(buf + 4, &data, 4);
	CAN_Send_Data(imu.can_handle, imu.can_id, buf, 8);
}

void imu_write_reg(uint8_t reg_id, uint32_t data)
{
	imu_send_cmd(reg_id, 3, data);
}

void imu_read_reg(uint8_t reg_id)
{
	uint8_t cmd[4];

	cmd[0] = (uint8_t)(imu.can_id & 0xFFu);
	cmd[1] = (uint8_t)((imu.can_id >> 8) & 0xFFu);
	cmd[2] = reg_id;
	cmd[3] = 0xCC;

	CAN_Send_Data(imu.can_handle, IMU_DATA_REQUEST_STDID, cmd, 4);

	if (imu.mst_id != imu.can_id)
	{
		cmd[0] = (uint8_t)(imu.mst_id & 0xFFu);
		cmd[1] = (uint8_t)((imu.mst_id >> 8) & 0xFFu);
		CAN_Send_Data(imu.can_handle, IMU_DATA_REQUEST_STDID, cmd, 4);
	}
}

void imu_reboot()
{
	imu_write_reg(REBOOT_IMU, 0);
}

void imu_accel_calibration()
{
	imu_write_reg(ACCEL_CALI, 0);
}

void imu_gyro_calibration()
{
	imu_write_reg(GYRO_CALI, 0);
}

void imu_change_com_port(imu_com_port_e port)
{
	imu_write_reg(CHANGE_COM, (uint8_t)port);
}

void imu_set_active_mode_delay(uint32_t delay)
{
	imu_write_reg(SET_DELAY, delay);
}

void imu_change_to_active()
{
	imu_write_reg(CHANGE_ACTIVE, 1);
}

void imu_change_to_request()
{
	imu_write_reg(CHANGE_ACTIVE, 0);
}

void imu_set_baud(imu_baudrate_e baud)
{
	imu_write_reg(SET_BAUD, (uint8_t)baud);
}

void imu_set_can_id(uint8_t can_id)
{
	imu_write_reg(SET_CAN_ID, can_id);
}

void imu_set_mst_id(uint8_t mst_id)
{
	imu_write_reg(SET_MST_ID, mst_id);
}

void imu_save_parameters()
{
	imu_write_reg(SAVE_PARAM, 0);
}

void imu_restore_settings()
{
	imu_write_reg(RESTORE_SETTING, 0);
}

void imu_request_accel()
{
	imu_read_reg(ACCEL_DATA);
}

void imu_request_gyro()
{
	imu_read_reg(GYRO_DATA);
}

void imu_request_euler()
{
	imu_read_reg(EULER_DATA);
}

void imu_request_quat()
{
	imu_read_reg(QUAT_DATA);
}

void IMU_UpdateAccel(uint8_t *pData)
{
	uint16_t accel[3];

	accel[0] = pData[3] << 8 | pData[2];
	accel[1] = pData[5] << 8 | pData[4];
	accel[2] = pData[7] << 8 | pData[6];

	imu.accel[0] = uint2_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
	imu.accel[1] = uint2_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
	imu.accel[2] = uint2_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
}

void IMU_UpdateGyro(uint8_t *pData)
{
	uint16_t gyro[3];

	gyro[0] = pData[3] << 8 | pData[2];
	gyro[1] = pData[5] << 8 | pData[4];
	gyro[2] = pData[7] << 8 | pData[6];

	imu.gyro[0] = uint2_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
	imu.gyro[1] = uint2_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
	imu.gyro[2] = uint2_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
}

void IMU_UpdateEuler(uint8_t *pData)
{
	int euler[3];

	euler[0] = pData[3] << 8 | pData[2];
	euler[1] = pData[5] << 8 | pData[4];
	euler[2] = pData[7] << 8 | pData[6];

	imu.pitch = uint2_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
	imu.yaw = uint2_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
	imu.roll = uint2_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
}

void IMU_UpdateQuaternion(uint8_t *pData)
{
	int w = pData[1] << 6 | ((pData[2] & 0xF8) >> 2);
	int x = (pData[2] & 0x03) << 12 | (pData[3] << 4) | ((pData[4] & 0xF0) >> 4);
	int y = (pData[4] & 0x0F) << 10 | (pData[5] << 2) | ((pData[6] & 0xC0) >> 6);
	int z = (pData[6] & 0x3F) << 8 | pData[7];

	imu.q[0] = uint2_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
	imu.q[1] = uint2_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
	imu.q[2] = uint2_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
	imu.q[3] = uint2_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);
}

void IMU_UpdateData(uint8_t *pData)
{
	switch (pData[0])
	{
		case 1:
			IMU_UpdateAccel(pData);
			break;
		case 2:
			IMU_UpdateGyro(pData);
			break;
		case 3:
			IMU_UpdateEuler(pData);
			break;
		case 4:
			IMU_UpdateQuaternion(pData);
			break;
		default:
			break;
	}
}
