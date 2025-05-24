/*
  xsns_24_si1132.ino - SI1132 UV Index / IR / Visible light sensor support for Tasmota

  Copyright (C) 2021  Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_I2C
#ifdef USE_SI1132
/*********************************************************************************************\
 * SI1132 - UV Index / IR / Visible light
 *
 * Based on library https://github.com/SeeedDocument/Grove-Sunlight_Sensor/
 *
 * I2C Addresses: 0x60
\*********************************************************************************************/

#define XSNS_24                             24
#define XI2C_19                             19  // See I2CDEVICES.md

#define SI113X_ADDR                         0X60
//
//commands
//
#define SI113X_QUERY                        0X80
#define SI113X_SET                          0XA0
#define SI113X_NOP                          0X00
#define SI113X_RESET                        0X01
#define SI113X_BUSADDR                      0X02
#define SI113X_GET_CAL                      0X12
#define SI113X_ALS_FORCE                    0X06
#define SI113X_ALS_PAUSE                    0X0A
#define SI113X_ALS_AUTO                     0X0E
//
//IIC REGISTERS
//
#define SI113X_PART_ID                      0X00
#define SI113X_REV_ID                       0X01
#define SI113X_SEQ_ID                       0X02
#define SI113X_INT_CFG                      0X03
#define SI113X_IRQ_ENABLE                   0X04
#define SI113X_IRQ_MODE1                    0x05
#define SI113X_IRQ_MODE2                    0x06
#define SI113X_HW_KEY                       0X07
#define SI113X_MEAS_RATE0                   0X08
#define SI113X_MEAS_RATE1                   0X09
#define SI113X_UCOEFF0                      0X13
#define SI113X_UCOEFF1                      0X14
#define SI113X_UCOEFF2                      0X15
#define SI113X_UCOEFF3                      0X16
#define SI113X_WR                           0X17
#define SI113X_COMMAND                      0X18
#define SI113X_RESPONSE                     0X20
#define SI113X_IRQ_STATUS                   0X21
#define SI113X_ALS_VIS_DATA0                0X22
#define SI113X_ALS_VIS_DATA1                0X23
#define SI113X_ALS_IR_DATA0                 0X24
#define SI113X_ALS_IR_DATA1                 0X25
#define SI113X_AUX_DATA0_UVINDEX0           0X2C
#define SI113X_AUX_DATA1_UVINDEX1           0X2D
#define SI113X_RD                           0X2E
#define SI113X_CHIP_STAT                    0X30
//
//Parameters
//
#define SI113X_CHLIST                       0X01
#define SI113X_CHLIST_ENUV                  0x80
#define SI113X_CHLIST_ENAUX                 0x40
#define SI113X_CHLIST_ENALSIR               0x20
#define SI113X_CHLIST_ENALSVIS              0x10

#define SI113X_ALS_ENCODE                   0X06
#define SI1132_ALS_VIS_ALIGN                0x10
#define SI1132_ALS_IR_ALIGN                 0x20

#define SI113X_ALS_IR_ADC_MUX               0X0E
#define SI113X_AUX_ADC_MUX                  0X0F

#define SI113X_ALS_VIS_ADC_COUNTER          0X10
#define SI113X_ALS_VIS_ADC_GAIN             0X11
#define SI113X_ALS_VIS_ADC_MISC             0X12

#define SI113X_ALS_IR_ADC_COUNTER           0X1D
#define SI113X_ALS_IR_ADC_GAIN              0X1E
#define SI113X_ALS_IR_ADC_MISC              0X1F
//
//USER SETTINGS DEFINE
//
//ADCMUX
#define SI113X_ADCMUX_SMALL_IR              0x00
#define SI113X_ADCMUX_VISIABLE              0x02
#define SI113X_ADCMUX_LARGE_IR              0x03
#define SI113X_ADCMUX_NO                    0x06
#define SI113X_ADCMUX_GND                   0x25
#define SI113X_ADCMUX_TEMPERATURE           0x65
#define SI113X_ADCMUX_VDD                   0x75
//ADC GAIN DIV
#define SI113X_ADC_GAIN_DIV1                0X00
#define SI113X_ADC_GAIN_DIV2                0X01
#define SI113X_ADC_GAIN_DIV4                0X02
#define SI113X_ADC_GAIN_DIV8                0X03
#define SI113X_ADC_GAIN_DIV16               0X04
#define SI113X_ADC_GAIN_DIV32               0X05
//Recovery period the  ADC takes before making a PS measurement
#define SI113X_ADC_COUNTER_1ADCCLK          0X00
#define SI113X_ADC_COUNTER_7ADCCLK          0X01
#define SI113X_ADC_COUNTER_15ADCCLK         0X02
#define SI113X_ADC_COUNTER_31ADCCLK         0X03
#define SI113X_ADC_COUNTER_63ADCCLK         0X04
#define SI113X_ADC_COUNTER_127ADCCLK        0X05
#define SI113X_ADC_COUNTER_255ADCCLK        0X06
#define SI113X_ADC_COUNTER_511ADCCLK        0X07
//ADC MISC
#define SI113X_ADC_MISC_LOWRANGE            0X00
#define SI113X_ADC_MISC_HIGHRANGE           0X20
#define SI113X_ADC_MISC_ADC_NORMALPROXIMITY 0X00
#define SI113X_ADC_MISC_ADC_RAWADC          0X04
//INT OE
#define SI113X_INT_CFG_INTOE                0X01
//IRQ ENABLE
#define SI113X_IRQEN_ALS                    0x01

uint16_t _vis_dark = 256;
uint16_t _ir_dark = 256;

uint16_t si1132_visible;
uint16_t si1132_infrared;
uint16_t si1132_uvindex;
//float si1132_lux;
uint16_t si1132_lux;

bool si1132_type = false;
uint8_t si1132_bus = 0;
uint8_t si1132_valid = 0;

/********************************************************************************************/

uint8_t Si1132ReadByte(uint8_t reg) {
  return I2cRead8(SI113X_ADDR, reg, si1132_bus);
}

uint16_t Si1132ReadHalfWord(uint8_t reg) {
  return I2cRead16LE(SI113X_ADDR, reg, si1132_bus);
}

void Si1132WriteByte(uint8_t reg, uint16_t val) {
  I2cWrite8(SI113X_ADDR, reg, val, si1132_bus);
}

uint8_t Si1132WriteParamData(uint8_t p, uint8_t v) {
  Si1132WriteByte(SI113X_WR, v);
  Si1132WriteByte(SI113X_COMMAND, p | SI113X_SET);
  return Si1132ReadByte(SI113X_RD);
}

/********************************************************************************************/

bool Si1132Present(void)
{
  return (Si1132ReadByte(SI113X_PART_ID) == 0X32);
}

void Si1132Reset(void)
{
  Si1132WriteByte(SI113X_MEAS_RATE0, 0);
  Si1132WriteByte(SI113X_MEAS_RATE1, 0);
  Si1132WriteByte(SI113X_IRQ_ENABLE, 0);
  Si1132WriteByte(SI113X_IRQ_MODE1, 0);
  Si1132WriteByte(SI113X_IRQ_MODE2, 0);
  Si1132WriteByte(SI113X_INT_CFG, 0);
  Si1132WriteByte(SI113X_IRQ_STATUS, 0xFF);

  Si1132WriteByte(SI113X_COMMAND, SI113X_RESET);
  delay(100);
  Si1132WriteByte(SI113X_HW_KEY, 0x17);
//  Si1132WriteByte(0x22,0x01);
  delay(100);
}

void Si1132DeInit(void)
{
  //ENABLE UV reading
  //these reg must be set to the fixed value
  //Si1132WriteByte(SI113X_UCOEFF0, 0x29);
  //Si1132WriteByte(SI113X_UCOEFF1, 0x89);
  //Si1132WriteByte(SI113X_UCOEFF2, 0x02);
  //Si1132WriteByte(SI113X_UCOEFF3, 0x00);
  Si1132WriteByte(SI113X_UCOEFF0, 0x7B);
  Si1132WriteByte(SI113X_UCOEFF1, 0x6B);
  Si1132WriteByte(SI113X_UCOEFF2, 0x01);
  Si1132WriteByte(SI113X_UCOEFF3, 0x00);
//  Si1132WriteParamData(SI113X_CHLIST, SI113X_CHLIST_ENUV | SI113X_CHLIST_ENAUX | SI113X_CHLIST_ENALSIR | SI113X_CHLIST_ENALSVIS);
  Si1132WriteParamData(SI113X_CHLIST, SI113X_CHLIST_ENUV | SI113X_CHLIST_ENALSIR | SI113X_CHLIST_ENALSVIS);
  //
  //interrupt enable
  //
  Si1132WriteByte(SI113X_INT_CFG, SI113X_INT_CFG_INTOE);
  Si1132WriteByte(SI113X_IRQ_ENABLE, SI113X_IRQEN_ALS);
  //
  //ALS ENCODING
  //
  Si1132WriteParamData(SI113X_ALS_ENCODE, SI1132_ALS_VIS_ALIGN | SI1132_ALS_IR_ALIGN);
  //
  //VIS ADC SETTING
  //
  Si1132WriteParamData(SI113X_ALS_VIS_ADC_COUNTER, SI113X_ADC_COUNTER_511ADCCLK);
  Si1132WriteParamData(SI113X_ALS_VIS_ADC_GAIN, SI113X_ADC_GAIN_DIV1);
  Si1132WriteParamData(SI113X_ALS_VIS_ADC_MISC, SI113X_ADC_MISC_HIGHRANGE);
  //
  //IR ADC SETTING
  //
  Si1132WriteParamData(SI113X_ALS_IR_ADC_COUNTER, SI113X_ADC_COUNTER_511ADCCLK);
  Si1132WriteParamData(SI113X_ALS_IR_ADC_GAIN, SI113X_ADC_GAIN_DIV1);
  Si1132WriteParamData(SI113X_ALS_IR_ADC_MISC, SI113X_ADC_MISC_HIGHRANGE);
  Si1132WriteParamData(SI113X_ALS_IR_ADC_MUX, SI113X_ADCMUX_SMALL_IR);
  //Si1132WriteParamData(SI113X_ALS_IR_ADC_MUX, SI113X_ADCMUX_LARGE_IR);
  Si1132WriteParamData(SI113X_AUX_ADC_MUX, SI113X_ADCMUX_TEMPERATURE);
  //
  //RATE SETTING
  //
  Si1132WriteByte(SI113X_MEAS_RATE0, 0xFF);
  //
  //ALS AUTO
  //
  Si1132WriteByte(SI113X_COMMAND, SI113X_ALS_AUTO);
  delay(100);
  //
}

bool Si1132Begin(void)
{
  if (!Si1132Present()) { return false; }

  Si1132Reset();
  Si1132DeInit();
  return true;
}

// returns the UV index * 100 (divide by 100 to get the index)
uint16_t Si1132ReadUV(void)
{
  return Si1132ReadHalfWord(SI113X_AUX_DATA0_UVINDEX0);
}

// returns visible+IR light levels
uint16_t Si1132ReadVisible(void)
{
  return Si1132ReadHalfWord(SI113X_ALS_VIS_DATA0);
}

// returns IR light levels
uint16_t Si1132ReadIR(void)
{
  return Si1132ReadHalfWord(SI113X_ALS_IR_DATA0);
}

// returns a calculated lux value as per SI144x AN523.6.
uint16_t calculateLux(uint16_t vis, uint16_t ir) {
  uint8_t gain = Si1132ReadByte(SI113X_ALS_VIS_ADC_GAIN);
  uint16_t lux = (((5.41f * vis) + (-0.08f * ir)) / (1 + 2 * gain))*100;
  return lux;
}

/********************************************************************************************/

bool Si1132Read(void)
{
  if (si1132_valid) { si1132_valid--; }

  if (!Si1132Present()) { return false; }

  si1132_visible = Si1132ReadVisible();
  if (si1132_visible < _vis_dark)
  _vis_dark = si1132_visible;
  si1132_visible -= _vis_dark;
 
  si1132_infrared = Si1132ReadIR();
  if (si1132_infrared < _ir_dark)
  _ir_dark = si1132_infrared;
  si1132_infrared -= _ir_dark;
 
  si1132_uvindex = Si1132ReadUV();
  si1132_lux = calculateLux(si1132_visible,si1132_infrared);
  si1132_valid = SENSOR_MAX_MISS;
  return true;
}

void Si1132Detect(void) {
  for (si1132_bus = 0; si1132_bus < 2; si1132_bus++) {
    if (!I2cSetDevice(SI113X_ADDR, si1132_bus)) { continue; }

    if (Si1132Begin()) {
      si1132_type = true;
      I2cSetActiveFound(SI113X_ADDR, "SI1132", si1132_bus);
      return;
    }
  }
}

void Si1132Update(void)
{
  if (!Si1132Read()) {
    AddLogMissed("SI1132", si1132_valid);
  }
}

#ifdef USE_WEBSERVER
const char HTTP_SNS_SI1132[] PROGMEM =
  "{s}SI1132 " D_LIGHT "{m}%d.%d " D_UNIT_LUX "{e}"     // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
  "{s}SI1132 " D_ILLUMINANCE "{m}%d " D_UNIT_LUX "{e}"
  "{s}SI1132 " D_INFRARED "{m}%d " D_UNIT_LUX "{e}"
  "{s}SI1132 " D_UV_INDEX "{m}%d.%d{e}";
#endif  // USE_WEBSERVER

void Si1132Show(bool json)
{
  if (si1132_valid) {
    if (json) {
      ResponseAppend_P(PSTR(",\"SI1132\":{\"" D_JSON_LIGHT "\":%d.%d,\"" D_JSON_ILLUMINANCE "\":%d,\"" D_JSON_INFRARED "\":%d,\"" D_JSON_UV_INDEX "\":%d.%d}"),
         si1132_lux /100, si1132_lux %100 , si1132_visible, si1132_infrared, si1132_uvindex /100, si1132_uvindex %100);
#ifdef USE_DOMOTICZ
      if (0 == TasmotaGlobal.tele_period) DomoticzSensor(DZ_ILLUMINANCE, si1132_visible);
#endif  // USE_DOMOTICZ
#ifdef USE_WEBSERVER
    } else {
      WSContentSend_PD(HTTP_SNS_SI1132, si1132_lux /100, si1132_lux %100, si1132_visible, si1132_infrared, si1132_uvindex /100, si1132_uvindex %100);
#endif
    }
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns24(uint32_t function)
{
  if (!I2cEnabled(XI2C_19)) { return false; }

  bool result = false;

  if (FUNC_INIT == function) {
    Si1132Detect();
  }
  else if (si1132_type) {
    switch (function) {
      case FUNC_EVERY_SECOND:
        Si1132Update();
        break;
      case FUNC_JSON_APPEND:
        Si1132Show(1);
        break;
  #ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        Si1132Show(0);
        break;
  #endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_SI1132
#endif  // USE_I2C
