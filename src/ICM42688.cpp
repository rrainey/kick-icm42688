/* 
 * 
 * ICM42688 class with interrupt, FIFO support, and enhanced
 * error reporting in a few critical locations.
 *
 * Based on code created by Kris Winer
 * 01/14/2022 Copyright Tlera Corporation
 * Library may be used freely and without limit with attribution.
 */
#include "ICM42688.h"

ICM42688::ICM42688( TwoWire * i2c )
{
  _i2c = i2c;
  _i2c_address = ICM42688_ADDRESS;
  _useSPI = false;

  _fifo_packet_config = 0;
  _gyro_on_time_us = TIMESPEC_OFF;
  _accel_on_time_us = TIMESPEC_OFF;
  _cache_PWR_MGMT0 = 0;

  _bank = 0;
}

ICM42688::ICM42688(SPIClass &bus, uint8_t csPin, uint32_t _spiHSRate) {
    _spi = &bus;
    _csPin = csPin;
    _useSPI = true;
    _useSPIHS = true;
    _spiHSClockRate = _spiHSRate;

    _fifo_packet_config = 0;
    _gyro_on_time_us = TIMESPEC_OFF;
    _accel_on_time_us = TIMESPEC_OFF;
    _cache_PWR_MGMT0 = 0;
}

uint8_t ICM42688::getChipID()
{
  uint8_t temp = 0;
  int status = readRegisters(ICM42688_WHO_AM_I, 1, &temp);
  if (status == -1) {
    temp = ICM42688_RETURN_ERR;
  }
  return temp;
}


float ICM42688::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
    case AFS_2G:
         _aRes = 2.0f/32768.0f;
         break;
    case AFS_4G:
         _aRes = 4.0f/32768.0f;
         break;
    case AFS_8G:
         _aRes = 8.0f/32768.0f;
         break;
    case AFS_16G:
         _aRes = 16.0f/32768.0f;
         break;
  }
  return _aRes;
         
}

float ICM42688::getGres(uint8_t Gscale) {
  switch (Gscale & 0x7)
  {
  // Possible gyro scales (and their register bit settings) are:
     case GFS_15_625DPS:
          _gRes = 15.625f/32768.0f;
          break;
    case GFS_31_25DPS:
          _gRes = 31.25f/32768.0f;
          break;
    case GFS_62_50DPS:
          _gRes = 62.5f/32768.0f;
          break;
    case GFS_125DPS:
          _gRes = 125.0f/32768.0f;
          break;
    case GFS_250DPS:
          _gRes = 250.0f/32768.0f;
          break;
    case GFS_500DPS:
          _gRes = 500.0f/32768.0f;
          break;
    case GFS_1000DPS:
         _gRes = 1000.0f/32768.0f;
         break;
    case GFS_2000DPS:
          _gRes = 2000.0f/32768.0f;
         break;
  }
  return _gRes;
}

void ICM42688::reset() {
    setBank(0);
    writeRegister(ICM42688_DEVICE_CONFIG, ICM42688_DEVICE_CONFIG_SOFT_RESET_CONFIG);
    delay(1);
}

uint8_t ICM42688::flushFifo() {
    return writeRegister(ICM42688_SIGNAL_PATH_RESET,
                     ICM42688_SIGNAL_PATH_RESET_FIFO_FLUSH);
}

uint8_t ICM42688::latchTimestamp() {
    return writeRegister(ICM42688_SIGNAL_PATH_RESET,
                     ICM42688_SIGNAL_PATH_RESET_TMST_STROBE);
}

uint8_t ICM42688::readIntStatus() {
    uint8_t temp;
    int status = readRegisters(ICM42688_INT_STATUS, 1, &temp);
    if (status != 1) {
        Serial.println("Interrupt status read failed");
    }
    return temp;
}

uint8_t ICM42688::setSensorState(icm42688AccelPowerMode newAccelStatus,
                                 icm42688GyroPowerMode newGyroStatus) {
    icm42688AccelPowerMode curAccelMode =
        (icm42688AccelPowerMode)(_cache_PWR_MGMT0 & 0x3);
    icm42688GyroPowerMode curGyroMode =
        (icm42688GyroPowerMode)((_cache_PWR_MGMT0 >> 2) & 0x3);

    bool powerStateChanged = false;

    uint32_t curTime = micros();

    if (curAccelMode != newAccelStatus) {
        powerStateChanged = true;
        switch (curAccelMode) {
            // "OFF" modes
            case aMode_OFF:
            case aMode_SBY:
                switch (newAccelStatus) {
                    // "OFF" modes
                    case aMode_OFF:
                    case aMode_SBY:
                        // no change to "on" time
                        break;
                    // "On" Modes
                    case aMode_LN:
                    case aMode_LP: {
                        _accel_on_time_us = curTime;
                    } break;
                }
                break;

            case aMode_LN:
            case aMode_LP:
                switch (newAccelStatus) {
                    // "OFF" modes
                    case aMode_OFF:
                    case aMode_SBY:
                        _accel_on_time_us = TIMESPEC_OFF;
                        break;
                    // "On" Modes
                    case aMode_LN:
                    case aMode_LP:
                        // must be switching modes; record time
                        _accel_on_time_us = curTime;
                        break;
                }
                break;
        }
    }

    if (curGyroMode != newGyroStatus) {
        powerStateChanged = true;

        switch (curGyroMode) {
            // "OFF" modes
            case gMode_OFF:
            case gMode_SBY:
                switch (newAccelStatus) {
                    // "OFF" modes
                    case gMode_OFF:
                    case gMode_SBY:
                        // no change to "on" time
                        break;
                    // "On" Modes
                    case gMode_LN:
                    case gMode_LP: {
                        _gyro_on_time_us = curTime;
                    } break;
                }
                break;

            case gMode_LN:
            case gMode_LP:
                switch (newAccelStatus) {
                    // "OFF" modes
                    case gMode_OFF:
                    case gMode_SBY:
                        _gyro_on_time_us = TIMESPEC_OFF;
                        break;
                    // "On" Modes
                    case gMode_LN:
                    case gMode_LP:
                        // must be switching modes; record time
                        _gyro_on_time_us = curTime;
                        break;
                }
                break;
        }
    }

    if (powerStateChanged) {
        _cache_PWR_MGMT0 =
            (uint8_t)newGyroStatus << 2 | (uint8_t)newAccelStatus;

        writeRegister(ICM42688_PWR_MGMT0, _cache_PWR_MGMT0);
        delay(1);
    }
    return ICM42688_RETURN_OK;
}

int ICM42688::begin() {
  if (_useSPI) {  // using SPI for communication
        // use low speed SPI for register setting
        _useSPIHS = false;
        // setting CS pin to output
        pinMode(_csPin, OUTPUT);
        // setting CS pin high
        digitalWrite(_csPin, HIGH);
        // begin SPI communication
        _spi->begin();
    } else {  // using I2C for communication
        // starting the I2C bus
        //_i2c->begin();
    }
    return 0;
}

void ICM42688::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR,
                    icm42688AccelPowerMode aMode, icm42688GyroPowerMode gMode,
                    bool CLKIN) {
    setBank(0);

    /*
     * "Gyroscope needs to be kept ON for a minimum of 45ms. When transitioning
     * from OFF to any of the other modes, do not issue any register writes for
     * 200µs"
     */
    setSensorState(aMode, gMode);

    writeRegister(ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR);

    writeRegister(ICM42688_GYRO_CONFIG0, Gscale << 5 | GODR);

    // RBR: used defaults since we're using LN data rates
    // writeRegister( ICM42688_GYRO_ACCEL_CONFIG0,  0x44); // set gyro and accel
    // bandwidth to ODR/10

    // interrupt handling
    writeRegister(ICM42688_INT_CONFIG,
              0x18 | 0x03);  // push-pull, pulsed, active HIGH interrupts

    // The rules for clearing an interrupt are configurable.
    //
    // Require both an interrupt status register read and a data read to clear these two interrupts
    // The default is to only require a interrupt status register read.
    // Configuration of other registers will only permit one of these two interrupt types
    // to be enabled at the same time.
    writeRegister(ICM42688_INT_CONFIG0,
              ICM42688_INT_CONFIG0_UI_DRDY_INT_CLEAR_STATUS_AND_READ  | 
              ICM42688_INT_CONFIG0_FIFO_THS_INT_CLEAR_STATUS_AND_READ ); 

    uint8_t temp;
    int status = readRegisters(ICM42688_INT_CONFIG1, 1, &temp);
    (void) status;

    temp &= ~(1 << 4);  // clear bit 4 to allow async interrupt reset (required
                        // for proper interrupt operation)
    if (AODR <= AODR_4kHz || GODR <= GODR_4kHz) {
        temp |= (1 << 5) |
                (1 << 6);  // shorten interrupt pulse/deassertion duration for
                           // higher data rates (required; see [1] 14.5)
    }
    writeRegister(ICM42688_INT_CONFIG1, temp);
    writeRegister(ICM42688_INT_SOURCE0, ICM42688_INT_SOURCE0_UI_DRDY_INT1_EN);

    // Use external clock source
    if (CLKIN) {
        //writeRegister(ICM42688_REG_BANK_SEL, 0x00);  // select register bank 0
        setBank(0);

        writeRegister(ICM42688_INTF_CONFIG1, 0x95);  // enable RTC

        //writeRegister(ICM42688_REG_BANK_SEL, 0x01);  // select register bank 1
        setBank(1);

        writeRegister(ICM42688_INTF_CONFIG5, 0x04);  // use CLKIN as clock source
    }

    //writeRegister(ICM42688_REG_BANK_SEL, 0x00);  // select register bank 0
    setBank(0);
}

void ICM42688::selfTest(int16_t * accelDiff, int16_t * gyroDiff, float * ratio)
{
  int8_t temp1[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelSTest[3] = {0, 0, 0}, gyroSTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

  setBank(0);
 
  // GYRO AND ACCEL TO LOW-NOISE MODE
  writeRegister( ICM42688_PWR_MGMT0,  
    ICM42688_PWR_MGMT0_GYRO_MODE_LN | ICM42688_PWR_MGMT0_ACCEL_MODE_LN );
  delay(1);

  writeRegister( ICM42688_ACCEL_CONFIG0, AFS_4G << 5 | AODR_1kHz);  // FS = 2
  
  writeRegister( ICM42688_GYRO_CONFIG0,  GFS_250DPS << 5 | GODR_1kHz);  // FS = 3
 
  writeRegister( ICM42688_GYRO_ACCEL_CONFIG0,  0x44); // set gyro and accel bandwidth to ODR/10

  readData(temp);
  accelNom[0] = temp[1];
  accelNom[1] = temp[2];
  accelNom[2] = temp[3];
  gyroNom[0]  = temp[4];
  gyroNom[1]  = temp[5];
  gyroNom[2]  = temp[6];
  
  // configure accel self test
  writeRegister( ICM42688_SELF_TEST_CONFIG, 
    ICM42688_SELF_TEST_CONFIG_ACCEL_ST_POWER |
    ICM42688_SELF_TEST_CONFIG_EN_AX_ST |
    ICM42688_SELF_TEST_CONFIG_EN_AY_ST |
    ICM42688_SELF_TEST_CONFIG_EN_AZ_ST );
  delay(100);
  readData(temp);
  accelSTest[0] = temp[1];
  accelSTest[1] = temp[2];
  accelSTest[2] = temp[3];

  // turn off accel self test power and configure for gyro test
  writeRegister( ICM42688_SELF_TEST_CONFIG, 
    ICM42688_SELF_TEST_CONFIG_EN_GX_ST |
    ICM42688_SELF_TEST_CONFIG_EN_GY_ST |
    ICM42688_SELF_TEST_CONFIG_EN_GZ_ST);
  delay(100);
  readData(temp);
  gyroSTest[0] = temp[4];
  gyroSTest[1] = temp[5];
  gyroSTest[2] = temp[6];

  // disable self tests
  writeRegister( ICM42688_SELF_TEST_CONFIG, 0x00);

  accelDiff[0] = accelSTest[0] - accelNom[0];
  if(accelDiff[0] < 0) accelDiff[0] *= -1;        // make sure difference values are positive
  accelDiff[1] = accelSTest[1] - accelNom[1];
  if(accelDiff[1] < 0)accelDiff[1] *= -1;
  accelDiff[2] = accelSTest[2] - accelNom[2];
  if(accelDiff[2] < 0) accelDiff[2] *= -1;
  gyroDiff[0] = gyroSTest[0] - gyroNom[0];
  if(gyroDiff[0] < 0) gyroDiff[0] *= -1;
  gyroDiff[1] = gyroSTest[1] - gyroNom[1];
  if(gyroDiff[1] < 0) gyroDiff[1] *= -1;
  gyroDiff[2] = gyroSTest[2] - gyroNom[2];
  if(gyroDiff[2] < 0) gyroDiff[2] *= -1;
  
  setBank(1);

  int status;

  // Read self-test data from factory test

  status = readRegisters(ICM42688_XG_ST_DATA, 1, (uint8_t *) &temp1[4]); // gyro self-test output generated during manufacturing tests
  status = readRegisters(ICM42688_YG_ST_DATA, 1, (uint8_t *) &temp1[5]);
  status = readRegisters(ICM42688_ZG_ST_DATA, 1, (uint8_t *) &temp1[6]);

  writeRegister( ICM42688_REG_BANK_SEL, 0x02); // select register bank 2

  status = readRegisters(ICM42688_XA_ST_DATA, 1, (uint8_t *) &temp1[1]);  // accel self-test output generated during manufacturing tests
  status = readRegisters(ICM42688_YA_ST_DATA, 1, (uint8_t *) &temp1[2]);
  status = readRegisters(ICM42688_ZA_ST_DATA, 1, (uint8_t *) &temp1[3]);

  (void) status;

  ratio[1] = accelDiff[0] / (1310.0f * powf(1.01f, temp[1] - 1) + 0.5f);
  ratio[2] = accelDiff[1] / (1310.0f * powf(1.01f, temp[2] - 1) + 0.5f);
  ratio[3] = accelDiff[2] / (1310.0f * powf(1.01f, temp[3] - 1) + 0.5f);
  ratio[4] = gyroDiff[0]  / (2620.0f * powf(1.01f, temp[4] - 1) + 0.5f);
  ratio[5] = gyroDiff[1] /  (2620.0f * powf(1.01f, temp[5] - 1) + 0.5f);
  ratio[6] = gyroDiff[2] /  (2620.0f * powf(1.01f, temp[6] - 1) + 0.5f);
  
  setBank(0);
}


void ICM42688::calibrateOffsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};
    
  for (int ii = 0; ii < 128; ii++)
  {
    readData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    delay(50);
  }

  dest1[0] = sum[1]*_aRes/128.0f;
  dest1[1] = sum[2]*_aRes/128.0f;
  dest1[2] = sum[3]*_aRes/128.0f;
  dest2[0] = sum[4]*_gRes/128.0f;
  dest2[1] = sum[5]*_gRes/128.0f;
  dest2[2] = sum[6]*_gRes/128.0f;

  if(dest1[0] > 0.8f)  {dest1[0] -= 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest1[0] < -0.8f) {dest1[0] += 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest1[1] > 0.8f)  {dest1[1] -= 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest1[1] < -0.8f) {dest1[1] += 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest1[2] > 0.8f)  {dest1[2] -= 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
  if(dest1[2] < -0.8f) {dest1[2] += 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
/*
  // load offset biases into offset registers (optional, comment out if not desired)
  temp[0] = (int16_t) (-dest1[0] / 0.00048828125f); // Ax 0.5 mg resolution
  temp[1] = (int16_t) (-dest1[1] / 0.00048828125f); // Ay
  temp[2] = (int16_t) (-dest1[2] / 0.00048828125f); // Az
  temp[3] = (int16_t) (-dest2[0] / 0.03125f);       // Gx 1/32 dps resolution
  temp[4] = (int16_t) (-dest2[1] / 0.03125f);       // Gy
  temp[5] = (int16_t) (-dest2[2] / 0.03125f);       // Gz

  writeRegister( ICM42688_REG_BANK_SEL, 0x04); // select register bank 4

  writeRegister( ICM42688_OFFSET_USER5,  temp[0] & 0x00FF); // lower Ax byte
  writeRegister( ICM42688_OFFSET_USER6,  temp[1] & 0x00FF); // lower Ay byte
  writeRegister( ICM42688_OFFSET_USER8,  temp[2] & 0x00FF); // lower Az byte
  writeRegister( ICM42688_OFFSET_USER2,  temp[4] & 0x00FF); // lower Gy byte
  writeRegister( ICM42688_OFFSET_USER3,  temp[5] & 0x00FF); // lower Gz byte
  writeRegister( ICM42688_OFFSET_USER0,  temp[3] & 0x00FF); // lower Gx byte
  writeRegister( ICM42688_OFFSET_USER4,  (temp[0] & 0x0F00) >> 4 | (temp[5] & 0x0F00) >> 8); // upper Ax and Gz bytes
  writeRegister( ICM42688_OFFSET_USER7,  (temp[2] & 0x0F00) >> 4 | (temp[1] & 0x0F00) >> 8); // upper Az and Ay bytes
  writeRegister( ICM42688_OFFSET_USER1,  (temp[4] & 0x0F00) >> 4 | (temp[3] & 0x0F00) >> 8); // upper Gy and Gx bytes
  
  writeRegister( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
  */
}


void ICM42688::readData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  readRegisters(ICM42688_TEMP_DATA1, 14, &rawData[0]);
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;   
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;  
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;  
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ; 

#if ICM42688_DEBUG
  char pbuf[80];
  sprintf(pbuf, "%02.2x %02.2x  %02.2x %02.2x  %02.2x %02.2x", rawData[2], rawData[3], rawData[4], rawData[5],  rawData[6], rawData[7]);
  Serial.println(pbuf);
  sprintf(pbuf, "%02.2x %02.2x  %02.2x %02.2x  %02.2x %02.2x", rawData[8], rawData[9], rawData[10], rawData[11],  rawData[12], rawData[13]);
  Serial.println(pbuf);
#endif
}

uint8_t ICM42688::startFifoSampling(uint8_t mode) {

  uint8_t data;
  uint8_t actual;

  _fifo_packet_config = 0;

  if (mode <=2 || mode > 4) {
    return ICM42688_RETURN_ERR;
  }

  /*
   * Enable Acc, gyro, and temp sensors in FIFO.
   * Enable threshold interrupts.
   * Partial reads are enabled (since Wire API can't read more than 32 bytes in one chunk)
   */
  actual = readRegisters(ICM42688_FIFO_CONFIG1, 1, &data);
  if (actual != 1) {
    Serial.println("assertion error: cannot read ICM42688_FIFO_CONFIG1");
    data = 0;
  }
  data |= (uint8_t)ICM426XX_FIFO_CONFIG1_ACCEL_EN;
	data |= (uint8_t)ICM426XX_FIFO_CONFIG1_GYRO_EN;
  data |= (uint8_t)ICM426XX_FIFO_CONFIG1_TEMP_EN;
  data |= (uint8_t)ICM426XX_FIFO_CONFIG1_WM_GT_TH_EN;

  // partial reads are needed as the Arduino Wire I/F is limited to 32 byte requests
  data |= (uint8_t)BIT_FIFO_CONFIG1_RESUME_PARTIAL_RD_MASK;

  //data |= (uint8_t)ICM426XX_FIFO_CONFIG1_TMST_FSYNC_EN;

  switch (mode) {
    case 3:
      data &= ~((uint8_t)ICM426XX_FIFO_CONFIG1_HIRES_EN);
      _fifo_packet_size = 16;
      _fifo_packet_config = 3;
      break;
    case 4:
      data |= (uint8_t)ICM426XX_FIFO_CONFIG1_HIRES_EN;
      _fifo_packet_size = 20;
      _fifo_packet_config = 4;
      break;
    default:
      return ICM42688_RETURN_ERR;
  }
  writeRegister(ICM42688_FIFO_CONFIG1, data);

  // set FIFO watermark to 2 records
  uint16_t watermark = 2 /** _fifo_packet_size*/;
  writeRegister(ICM42688_FIFO_CONFIG2, watermark & 0xFF);
  writeRegister(ICM42688_FIFO_CONFIG3, watermark >> 8);

  // switch to FIFO watermark and RESET for INT1
  writeRegister(ICM42688_INT_SOURCE0, 
    ICM42688_INT_SOURCE0_RESET_DONE_INT1_EN | ICM42688_INT_SOURCE0_FIFO_THS_INT1_EN);

  // enable FIFO streaming (i.e., data now comes via FIFO)
  writeRegister(ICM42688_FIFO_CONFIG, ICM426XX_FIFO_CONFIG_STREAM);

  return ICM42688_RETURN_OK;
}

uint8_t ICM42688::readFiFo(icm42688::fifo_packet3 *pBuf, uint16_t *pPacketCount) {
  uint8_t actual; 
  uint16_t bytes;
  uint8_t buffer[2];
  icm42688::fifo_xfer_packet3 buf;
  icm42688::fifo_xfer_packet3 *p3;
  icm42688::fifo_packet3 *p3Dest;

  *pPacketCount = 0;

  if (_fifo_packet_config != 3) {
    return ICM42688_RETURN_ERR;
  }

  buffer[0] = buffer[1] = 0;

  actual = readRegisters(ICM42688_FIFO_COUNTH | (1<<7), 2, buffer);
  if (actual != 2) {
    return ICM42688_RETURN_ERR;
  }
  bytes = ((uint16_t)buffer[0] <<8) | buffer[1];

  if (bytes == 0) {
    return ICM42688_RETURN_OK;
  }

  uint16_t packets = bytes / _fifo_packet_size;
  uint16_t actualPackets = 0;

  if (bytes % _fifo_packet_size != 0) {
    Serial.print("assertion error: avalable bytes not a multiple of FIFO packet size: ");
    Serial.println(bytes);
  }

  if (packets > 139) {
    Serial.print("assertion error: greater than 139 packets in FIFO: ");
    Serial.println(packets);
    packets = 130;
  }

  p3Dest = pBuf;

  for (int i = 0; i < packets; ++i) {
      p3 = &buf;

      // It's tempting to read the whole available FIFO in a single request, but the standard
      // Arduino Wire class is limited to 32 bytes per request -- so, we'll instead read
      // one packet at a time.
      actual = readRegisters(ICM42688_FIFO_DATA, _fifo_packet_size, (uint8_t*) p3);
      if (actual == _fifo_packet_size) {

        p3Dest->ax = ((int16_t)p3->ax_high << 8) | p3->ax_low;
        p3Dest->ay = ((int16_t)p3->ay_high << 8) | p3->ay_low;
        p3Dest->az = ((int16_t)p3->az_high << 8) | p3->az_low;
        p3Dest->gx = ((int16_t)p3->gx_high << 8) | p3->gx_low;
        p3Dest->gy = ((int16_t)p3->gy_high << 8) | p3->gy_low;
        p3Dest->gz = ((int16_t)p3->gz_high << 8) | p3->gz_low;
        p3Dest->timestamp =
            (uint16_t)((uint16_t)p3->timestamp_high << 8 | p3->timestamp_low);
        p3Dest->header = p3->header;
        p3Dest->temp = p3->temp;

        ++p3Dest;

        ++ actualPackets;
      }
      else {
          Serial.print("read request of a FIFO packet failed: requested ");
          Serial.print( _fifo_packet_size );
          Serial.print(", actual ");
          Serial.print( actual );

          // experimental: just return what we have after an error
          if (actual == 0) {
            *pPacketCount = actualPackets;
            return ICM42688_RETURN_OK;
          }
      }
  }

  *pPacketCount = actualPackets;

  return ICM42688_RETURN_OK;
}


void ICM42688::setTiltDetect()
{
  writeRegister( ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
  writeRegister( ICM42688_APEX_CONFIG4, 0x00); // immediately interrupt on tilt

  writeRegister( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
  writeRegister( ICM42688_APEX_CONFIG0, 0x02); // DMP power save off, DMP ODR at 50 Hz
  writeRegister( ICM42688_SIGNAL_PATH_RESET, 0x20); // DMP memory reset
  delay(1);
  writeRegister( ICM42688_SIGNAL_PATH_RESET, 0x40); // DMP enable
  writeRegister( ICM42688_APEX_CONFIG0, 0x12); // Tilt detect enable
  delay(2000); // wait for tilt to stabilize

  writeRegister( ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
  writeRegister( ICM42688_INT_SOURCE7, 0x08); // route tilt interrupt to INT2

  writeRegister( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
}


void ICM42688::setWakeonMotion()
{
  writeRegister( ICM42688_REG_BANK_SEL, 0x04); // select register bank 4
  writeRegister( ICM42688_ACCEL_WOM_X_THR, 0x50); // 80 x 3.9 mg is ~312 mg
  writeRegister( ICM42688_ACCEL_WOM_Y_THR, 0x50); // 80 x 3.9 mg is ~312 mg
  writeRegister( ICM42688_ACCEL_WOM_Z_THR, 0x50); // 80 x 3.9 mg is ~312 mg

  writeRegister( ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
  writeRegister( ICM42688_INT_SOURCE4, 0x07);  // route wake on motion for any axis to INT2
  writeRegister( ICM42688_SMD_CONFIG, 0x05);   // differential mode, WOM interrupt ORed with other APEX interrupts, enable WOM function (bit 0)
}


uint16_t ICM42688::APEXStatus()
{
  int status;
  uint8_t status2, status3;
  status = readRegisters(ICM42688_INT_STATUS2, 1, &status2);
  status = readRegisters(ICM42688_INT_STATUS3, 1, &status3);
  (void) status;
  uint16_t temp = ((uint16_t) status2 << 8) | status3; 
  return temp;
}

int ICM42688::writeRegister(uint8_t subAddress, uint8_t data) {

    uint8_t buffer;
    /* write data to device */
    if (_useSPI) {
        _spi->beginTransaction(SPISettings(
            SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));  // begin the transaction
        digitalWrite(_csPin, LOW);                // select the ICM42688 chip
        _spi->transfer(subAddress);               // write the register address
        _spi->transfer(data);                     // write the data
        digitalWrite(_csPin, HIGH);               // deselect the ICM42688 chip
        _spi->endTransaction();                   // end the transaction
    } else {
        _i2c->beginTransmission(_i2c_address);    // open the device
        _i2c->write(subAddress);            // write the register address
        _i2c->write(data);                  // write the data
        _i2c->endTransmission();
    }

    delay(2);

    /* read back the register */
    readRegisters(subAddress, 1, &buffer);
    /* check the read back register against the written register */
    if (buffer == data) {
        return 1;
    } else {
        return -1;
    }
}

int ICM42688::readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest) {
    if (_useSPI) {
        // begin the transaction
        if (_useSPIHS) {
            _spi->beginTransaction(
                SPISettings(_spiHSClockRate, MSBFIRST, SPI_MODE3));
        } else {
            _spi->beginTransaction(
                SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
        }
        digitalWrite(_csPin, LOW);
        _spi->transfer(subAddress |
                       0x80);
  
        _spi->transfer(dest, count);
        digitalWrite(_csPin, HIGH);
        _spi->endTransaction();
        return count;
    } else {
        _i2c->beginTransmission(_i2c_address);  
        _i2c->write(subAddress);  // specify the starting register address
        _i2c->endTransmission(false);
        _numBytes = _i2c->requestFrom(
            _i2c_address, count);  // specify the number of bytes to receive
        if (_numBytes == count) {
            for (uint8_t i = 0; i < count; i++) {
                dest[i] = _i2c->read();
            }
            return count;
        } else {
            return -1;
        }
    }
}

int ICM42688::setBank(uint8_t bank) {
  // if we are already on this bank, bail
  if (_bank == bank) return 1;

  _bank = bank;

  return writeRegister(ICM42688_REG_BANK_SEL, bank);
}