#ifndef _MPU9250_SPI_H_
#define _MPU9250_SPI_H_
#include "mbed.h"
#include "math.h"
#include "rtos.h"

#define USE_ISR 1 

#define USE_ISR 1  // poll or data ready interrupt   
 
// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C<<1
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value
 
#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02
 
/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */
 
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F
 
#define SELF_TEST_A      0x10
 
#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   
 
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
 
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E
 
#define READ_FLAG   0x80 // for SPI read flag

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};
 
enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};
 
enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};
 
uint8_t Ascale = AFS_4G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_1000DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR  
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
 
uint8_t accel_raw[6];
uint8_t gyro_raw[6];
uint8_t mag_raw[6];

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];

int delt_t = 0; // used to control display output rate
int count = 0;  // used to control display output rate
 
// parameters for 6 DoF sensor fusion calculations
float PI = 3.14159265358979323846f;
float GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f
 
float pitch, yaw, roll;
float deltat = 0.0f;                             // integration interval for both filter schemes
int lastUpdate = 0, firstUpdate = 0, Now = 0;    // used to calculate integration interval                               // used to calculate integration interval
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};              // vector to hold integral error for Mahony method
 
class MPU9250_SPI{
private:

    SPI& spi_;
    DigitalOut cs_;
private:
    void select(){
        //Set CS low to start transmission (interrupts conversion)
        cs_ = 0;
    };

    void deselect(){
        //Set CS high to stop transmission (restarts conversion)
        cs_ = 1;
    };
public:
    MPU9250_SPI(SPI& spi, PinName cs) : spi_(spi), cs_(cs){ };

  //===================================================================================================================
//====== Set of useful function to access acceleratio, gyroscope, and temperature data
//===================================================================================================================
 
    uint8_t writeByte(uint8_t subAddress, uint8_t data)
    {
        uint8_t response;
        select();
        spi_.write(subAddress);
        response = spi_.write(data);
        deselect();
        wait_us(50);
        return response;
    };
    
    uint8_t readByte(uint8_t subAddress)
    {
        uint8_t response;
        select();
        spi_.write(subAddress | READ_FLAG);
        response = spi_.write(0x00);
        deselect();
        wait_us(50);
        return response;
    }
    
    void readBytes(uint8_t subAddress, uint8_t count, uint8_t * dest)
    {     
        unsigned int i = 0;
        select();
        spi_.write(subAddress|READ_FLAG);
        for(i = 0; i < count; ++i){
            dest[i] = spi_.write(0x00);
        }
        deselect();
        wait_us(50);
    };
    
 
    void getMres() {
        switch (Mscale)
        {
            // Possible magnetometer scales (and their register bit settings) are:
            // 14 bit resolution (0) and 16 bit resolution (1)
            case MFS_14BITS:
                mRes = 10.0*4219.0/8190.0; // Proper scale to return milliGauss
                break;
            case MFS_16BITS:
                mRes = 10.0*4219.0/32760.0; // Proper scale to return milliGauss
                break;
        }
    }
    
    
    void getGres() {
        switch (Gscale)
        {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
                // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case GFS_250DPS:
                gRes = 250.0/32768.0;
                break;
            case GFS_500DPS:
                gRes = 500.0/32768.0;
                break;
            case GFS_1000DPS:
                gRes = 1000.0/32768.0;
                break;
            case GFS_2000DPS:
                gRes = 2000.0/32768.0;
                break;
        }
    }
    
    
    void getAres() {
        switch (Ascale)
        {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
                // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case AFS_2G:
                aRes = 2.0/32768.0;
                break;
            case AFS_4G:
                aRes = 4.0/32768.0;
                break;
            case AFS_8G:
                aRes = 8.0/32768.0;
                break;
            case AFS_16G:
                aRes = 16.0/32768.0;
                break;
        }
    }
    
    void readAccGyroData(uint8_t* acc_dest, uint8_t* gyro_dest){
        uint8_t rawData[14];
        readBytes(ACCEL_XOUT_H, 14, &rawData[0]);
        acc_dest[0] = rawData[0];
        acc_dest[1] = rawData[1];
        acc_dest[2] = rawData[2];
        acc_dest[3] = rawData[3];
        acc_dest[4] = rawData[4];
        acc_dest[5] = rawData[5];

        gyro_dest[0] = rawData[8];
        gyro_dest[1] = rawData[9];
        gyro_dest[2] = rawData[10];
        gyro_dest[3] = rawData[11];
        gyro_dest[4] = rawData[12];
        gyro_dest[5] = rawData[13];
    };

    void readAccelData(int16_t * destination)
    {
        uint8_t rawData[6];  // x/y/z accel register data stored here
        readBytes(ACCEL_XOUT_H, 6, &rawData[0]);
        destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    }

    void readAccelData(int16_t * destination, uint8_t * dest_8)
    {
        uint8_t rawData[6];  // x/y/z accel register data stored here
        readBytes(ACCEL_XOUT_H, 6, &rawData[0]);
        destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
        dest_8[0] = rawData[0];
        dest_8[1] = rawData[1];
        dest_8[2] = rawData[2];
        dest_8[3] = rawData[3];
        dest_8[4] = rawData[4];
        dest_8[5] = rawData[5];
    }
    void readGyroData(int16_t * destination)
    {
        uint8_t rawData[6];  // x/y/z gyro register data stored here
        readBytes(GYRO_XOUT_H, 6, &rawData[0]);
        destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    }
    void readGyroData(int16_t * destination, uint8_t * dest_8)
    {
        uint8_t rawData[6];  // x/y/z gyro register data stored here
        readBytes(GYRO_XOUT_H, 6, &rawData[0]);
        destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
        
        dest_8[0] = rawData[0];
        dest_8[1] = rawData[1];
        dest_8[2] = rawData[2];
        dest_8[3] = rawData[3];
        dest_8[4] = rawData[4];
        dest_8[5] = rawData[5];
    }

    void readMagData(int16_t * destination)
    {
        uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
        if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
            readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
            uint8_t c = rawData[6]; // End data read by reading ST2 register
                if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
                destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
                destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]);  // Data stored as little Endian
                destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]); 
            }
        }
    }

    


};
#endif