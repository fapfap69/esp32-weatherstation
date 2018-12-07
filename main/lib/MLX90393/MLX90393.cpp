/*
 * MLX90393.cpp
 *
 *  Created on: Dec 3, 2018
GAIN_SEL	RES=0			RES=1			RES=2			RES=3
			SEXY	SEZ		SEXY	SEZ		SEXY	SEZ		SEXY	SEZ
0			0.805	1.468	1.610	2.936	3.220	5.872	6.440	11.744
1			0.644	1.174	1.288	2.349	2.576	4.698	5.152	9.395
2			0.483	0.881	0.966	1.762	1.932	3.523	3.864	7.046
3			0.403	0.734	0.805	1.468	1.610	2.936	3.220	5.872
4			0.322	0.587	0.644	1.174	1.288	2.349	2.576	4.698
5			0.268	0.489	0.537	0.979	1.073	1.957	2.147	3.915
6			0.215	0.391	0.429	0.783	0.859	1.566	1.717	3.132
7			0.161	0.294	0.322	0.587	0.644	1.174	1.288	2.349

 *      Author: fap
 */

#include "MLX90393.h"

#include "../i2c/i2c.h"

MLX90393::MLX90393(I2Cmaster *Wire) {

	i2c = Wire;
	i2cDevAddr = 0;


/*	// gain steps derived from datasheet section 15.1.4 tables
	gain_multipliers[0] = 5.f;
	gain_multipliers[1] = 4.f;
	gain_multipliers[2] = 3.f;
	gain_multipliers[3] = 2.5f;
	gain_multipliers[4] = 2.f;
	gain_multipliers[5] = 1.66666667f;
	gain_multipliers[6] = 1.33333333f;
	gain_multipliers[7] = 1.f;
*/
	// from datasheet
	// for hallconf = 0
	base_xy_sens_hc0 = 0.196f;
	base_z_sens_hc0 = 0.316f;
	// for hallconf = 0xc
	base_xy_sens_hc0xc = 0.150f;
	base_z_sens_hc0xc = 0.242f;


}

MLX90393::~MLX90393() {
	// TODO Auto-generated destructor stub
}

uint8_t MLX90393::init(uint8_t Address)
{

	i2cDevAddr = Address;

	uint8_t status1 = checkStatus(reset());
	uint8_t status2 = setGainSel(5);
	uint8_t status3 = setResolution(0, 0, 0);
    uint8_t status4 = setOverSampling(3);
	uint8_t status5 = setDigitalFiltering(7);
	uint8_t status6 = setTemperatureCompensation(0);

	return status1 | status2 | status3 | status4 | status5 | status6;
}
uint8_t MLX90393::readData(txyz& data)
{
	uint8_t status1 = startMeasurement(X_FLAG | Y_FLAG | Z_FLAG | T_FLAG);
	ets_delay_us( getMillisDelay() * 1000 ); //
    txyzRaw raw_txyz;
    uint8_t status2 = readMeasurement(X_FLAG | Y_FLAG | Z_FLAG | T_FLAG, raw_txyz);
	data = convertRaw(raw_txyz);
	return checkStatus(status1) | checkStatus(status2);
}

uint8_t MLX90393::readRawData(txyzRaw& data)
{
	uint8_t status1 = startMeasurement(X_FLAG | Y_FLAG | Z_FLAG | T_FLAG);
	ets_delay_us( getMillisDelay() * 1000 ); //
    uint8_t status2 = readMeasurement(X_FLAG | Y_FLAG | Z_FLAG | T_FLAG, data);
	return checkStatus(status1) | checkStatus(status2);
}

uint16_t MLX90393::getMillisDelay() {
	uint16_t data;
	readRegister(OSR_REG, &data);
	uint8_t osr = (data & OSR_MASK) >> OSR_SHIFT;
    uint8_t osr2 = (data & OSR2_MASK) >> OSR2_SHIFT;
    uint8_t dig_flt = (data & DIG_FLT_MASK) >> DIG_FLT_SHIFT;
    return ( 3 * (2 + (1 << dig_flt)) * (1 << osr) *0.064f + (1 << osr2) * 0.192f ) * 1.3f;  // 30% tolerance
}

uint8_t MLX90393::startMeasurement(uint8_t zyxt_flags)
{
  uint8_t cmd = CMD_START_MEASUREMENT | (zyxt_flags & 0xf);
  return sendCommand(cmd);
}

uint8_t MLX90393::readMeasurement(uint8_t zyxt_flags, txyzRaw& txyz_result)
{
  uint8_t cmd = CMD_READ_MEASUREMENT | (zyxt_flags & 0xf);
  if( sendCommand(cmd) == STATUS_ERROR) return STATUS_ERROR;

  uint8_t buffer[9];
  uint8_t count = 1 + (((zyxt_flags & Z_FLAG)?2:0) + ((zyxt_flags & Y_FLAG)?2:0) +
                       ((zyxt_flags & X_FLAG)?2:0) + ((zyxt_flags & T_FLAG)?2:0) );

  if( i2c->read(i2cDevAddr, buffer, count) != ESP_OK) return STATUS_ERROR;

  uint8_t i = 1;
  if (zyxt_flags & T_FLAG){
    txyz_result.t =  (uint16_t(buffer[i])<<8) | buffer[i+1];
    i += 2;
  } else {
    txyz_result.t = 0;
  }
  if (zyxt_flags & X_FLAG){
    txyz_result.x =  (uint16_t(buffer[i])<<8) | buffer[i+1];
    i += 2;
  } else {
    txyz_result.x = 0;
  }
  if (zyxt_flags & Y_FLAG){
    txyz_result.y =  (uint16_t(buffer[i])<<8) | buffer[i+1];
    i += 2;
  } else {
    txyz_result.y = 0;
  }
  if (zyxt_flags & Z_FLAG){
    txyz_result.z =  (uint16_t(buffer[i])<<8) | buffer[i+1];
    i += 2;
  } else {
    txyz_result.z = 0;
  }
  return buffer[0];
}


MLX90393::txyz MLX90393::convertRaw(MLX90393::txyzRaw raw)
{
	uint16_t data;
	readRegister(GAIN_SEL_REG, &data);
	uint8_t gain_sel = (data & GAIN_SEL_MASK) >> GAIN_SEL_SHIFT;
	//uint8_t hallconf = (data & HALLCONF_MASK) >> HALLCONF_SHIFT;
	readRegister(RES_XYZ_REG, &data);
	uint8_t res_xyz = (data & RES_XYZ_MASK) >> RES_XYZ_SHIFT;
	uint8_t resol[3];
	resol[0] = (res_xyz >> 0) & 0x3;
	resol[1] = (res_xyz >> 2) & 0x3;
	resol[2] = (res_xyz >> 4) & 0x3;
	readRegister(TCMP_EN_REG, &data);
	uint8_t tcmp_en = (data & TCMP_EN_MASK) >> TCMP_EN_SHIFT;

	uint16_t intval[3];
	intval[0] = raw.x;
	intval[1] = raw.y;
	intval[2] = raw.z;
	float gain[3];
	float val[3];
	gain[0] = gainXY[gain_sel][resol[0]];
	gain[1] = gainXY[gain_sel][resol[1]];
	gain[2] = gainZ[gain_sel][resol[2]];
	for(int i=0;i<3;i++) {
		if(!tcmp_en) {
			switch (resol[i]) {
			case 0:
			case 1:
				val[i] = intval[i] - (intval[i] > 32768 ? 65536: 0);
				break;
			case 2:
				val[i] = intval[i] - 32768;
				break;
			case 3:
				val[i] = intval[i] - 16384;
				break;
			}
		} else {
			val[i] = intval[i] - 32768;
		}
		val[i] = val[i] * gain[i] * (1 << resol[i]);
	}
	txyz engVal;
	engVal.x = val[0];
	engVal.y = val[1];
	engVal.z = val[2];
	engVal.t = 25 + (raw.t - 46244.f)/45.2f;
	return engVal;
}


uint8_t MLX90393::reset()
{
  //cache_invalidate();
  uint8_t status = sendCommand(CMD_RESET);
  ets_delay_us( 2000 ); // POR is 1.6ms max. Software reset time limit is not specified.
  return status;
}

uint8_t MLX90393::sendCommand(uint8_t cmd)
{
	uint8_t buf;
	if(i2c->write(i2cDevAddr, &cmd, 1) != ESP_OK) return STATUS_ERROR;
	if(i2c->read(i2cDevAddr, &buf, 1) != ESP_OK) return STATUS_ERROR;
	return buf;
}

uint8_t MLX90393::readRegister(uint8_t regAddr, uint16_t *data)
{
	uint8_t buf[3];
	buf[0] = CMD_READ_REGISTER;
	buf[1] = (regAddr & 0x3f)<<2;
	if(i2c->write(i2cDevAddr, buf, 2) != ESP_OK) return STATUS_ERROR;
	if(i2c->read(i2cDevAddr, buf, 3) != ESP_OK) return STATUS_ERROR;
	*data = (uint16_t(buf[1])<<8) | buf[2];
	return buf[0];
}

uint8_t MLX90393::writeRegister(uint8_t regAddr, uint16_t data)
{
	uint8_t buf[4];
	buf[0] = CMD_WRITE_REGISTER;
	buf[1] = (data & 0xff00) >> 8;
	buf[2] = data & 0x00ff;
	buf[3] = (regAddr & 0x3f) << 2;
	if(i2c->write(i2cDevAddr, buf, 4) != ESP_OK) return STATUS_ERROR;
	if(i2c->read(i2cDevAddr, buf, 1) != ESP_OK) return STATUS_ERROR;
	return buf[0];
}



uint8_t MLX90393::setGainSel(uint8_t gain_sel)
{
  uint16_t old_val;
  uint8_t status1 = readRegister(GAIN_SEL_REG, &old_val);
  uint8_t status2 = writeRegister(GAIN_SEL_REG, (old_val & ~GAIN_SEL_MASK) | ((uint16_t(gain_sel) << GAIN_SEL_SHIFT) & GAIN_SEL_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t MLX90393::getGainSel(uint8_t& gain_sel)
{
  uint16_t reg_val;
  uint8_t status = readRegister(GAIN_SEL_REG, &reg_val);
  gain_sel = (reg_val & GAIN_SEL_MASK) >> GAIN_SEL_SHIFT;
  return checkStatus(status);
}

uint8_t MLX90393::setResolution(uint8_t res_x, uint8_t res_y, uint8_t res_z)
{
  uint16_t res_xyz = ((res_z & 0x3)<<4) | ((res_y & 0x3)<<2) | (res_x & 0x3);
  uint16_t old_val;
  uint8_t status1 = readRegister(RES_XYZ_REG, &old_val);
  uint8_t status2 = writeRegister(RES_XYZ_REG, (old_val & ~RES_XYZ_MASK) | ((res_xyz << RES_XYZ_SHIFT) & RES_XYZ_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t MLX90393::getResolution(uint8_t& res_x, uint8_t& res_y, uint8_t& res_z)
{
  uint16_t reg_val;
  uint8_t status = readRegister(RES_XYZ_REG, &reg_val);
  uint8_t res_xyz = (reg_val & RES_XYZ_MASK) >> RES_XYZ_SHIFT;
  res_x = (res_xyz >> 0) & 0x3;
  res_y = (res_xyz >> 2) & 0x3;
  res_z = (res_xyz >> 4) & 0x3;
  return checkStatus(status);
}

uint8_t MLX90393::setOverSampling(uint8_t osr)
{
  uint16_t old_val;
  uint8_t status1 = readRegister(OSR_REG, &old_val);
  uint8_t status2 = writeRegister(OSR_REG, (old_val & ~OSR_MASK) | ((uint16_t(osr) << OSR_SHIFT) & OSR_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t MLX90393::getOverSampling(uint8_t& osr)
{
  uint16_t reg_val;
  uint8_t status = readRegister(OSR_REG, &reg_val);
  osr = (reg_val & OSR_MASK) >> OSR_SHIFT;
  return checkStatus(status);
}

uint8_t MLX90393::setTemperatureOverSampling(uint8_t osr2)
{
  uint16_t old_val;
  uint8_t status1 = readRegister(OSR2_REG, &old_val);
  uint8_t status2 = writeRegister(OSR2_REG, (old_val & ~OSR2_MASK) | ((uint16_t(osr2) << OSR2_SHIFT) & OSR2_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t MLX90393::getTemperatureOverSampling(uint8_t& osr2)
{
  uint16_t reg_val;
  uint8_t status = readRegister(OSR2_REG, &reg_val);
  osr2 = (reg_val & OSR2_MASK) >> OSR2_SHIFT;
  return checkStatus(status);
}

uint8_t MLX90393::setDigitalFiltering(uint8_t dig_flt)
{
  uint16_t old_val;
  uint8_t status1 = readRegister(DIG_FLT_REG, &old_val);
  uint8_t status2 = writeRegister(DIG_FLT_REG, (old_val & ~DIG_FLT_MASK) | ((uint16_t(dig_flt) << DIG_FLT_SHIFT) & DIG_FLT_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t MLX90393::getDigitalFiltering(uint8_t& dig_flt)
{
  uint16_t reg_val;
  uint8_t status = readRegister(DIG_FLT_REG, &reg_val);
  dig_flt = (reg_val & DIG_FLT_MASK) >> DIG_FLT_SHIFT;
  return checkStatus(status);
}

uint8_t MLX90393::setTemperatureCompensation(uint8_t enabled)
{
  uint8_t tcmp_en = enabled?1:0;
  uint16_t old_val;
  uint8_t status1 = readRegister(TCMP_EN_REG, &old_val);
  uint8_t status2 = writeRegister(TCMP_EN_REG,  (old_val & ~TCMP_EN_MASK) | ((uint16_t(tcmp_en) << TCMP_EN_SHIFT) & TCMP_EN_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t MLX90393::getTemperatureCompensation(uint8_t& enabled)
{
  uint16_t reg_val;
  uint8_t status = readRegister(TCMP_EN_REG, &reg_val);
  enabled = (reg_val & TCMP_EN_MASK) >> TCMP_EN_SHIFT;
  return checkStatus(status);
}

uint8_t MLX90393::setHallConf(uint8_t hallconf)
{
  uint16_t old_val;
  uint8_t status1 = readRegister(HALLCONF_REG, &old_val);
  uint8_t status2 = writeRegister(HALLCONF_REG, (old_val & ~HALLCONF_MASK) | ((uint16_t(hallconf) << HALLCONF_SHIFT) & HALLCONF_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t MLX90393::getHallConf(uint8_t& hallconf)
{
  uint16_t reg_val;
  uint8_t status = readRegister(HALLCONF_REG, &reg_val);
  hallconf = (reg_val & HALLCONF_MASK) >> HALLCONF_SHIFT;
  return checkStatus(status);
}

