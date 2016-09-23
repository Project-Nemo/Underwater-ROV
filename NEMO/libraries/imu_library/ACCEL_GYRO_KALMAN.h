#include "Arduino.h"

// ADXL345 ACCELEROMETER FIELDS
#define ACCEL_ADDRESS			(0x53)

#define REG_DEVID 				(0x00)
#define REG_THRESH_TAP			(0x1D) // 1
#define REG_DUR					(0x21) // 2
#define REG_LATENT				(0x22) // 3
#define REG_WINDOW				(0x23) // 4
#define REG_THRESH_ACT			(0x24) // 5
#define REG_THRESH_INACT		(0x25) // 6
#define REG_TIME_INACT			(0x26) // 7
#define REG_ACT_INACT_CTL		(0x27)
#define REG_THRESH_FF			(0x28) // 8
#define REG_TIME_FF				(0x29) // 9
#define REG_TAP_AXES			(0x2A)
#define REG_BW_RATE				(0x2C)
#define REG_POWER_CTL			(0x2D)
#define REG_DATA_FORMAT			(0x31)
#define REG_DATA_X0				(0x32)
#define REG_DATA_Y0				(0x34)
#define REG_DATA_Z0				(0x36)

#define GRAVITY_EARTH        9.80665f

// L3G4200D GYROSCOPE FIELDS
#define GYRO_ADDRESS			(0x69)
#define REG_WHO_AM_I			(0x0F)

#define REG_CTRL_REG1			(0x20)
#define REG_CTRL_REG2			(0x21)
#define REG_CTRL_REG3			(0x22)
#define REG_CTRL_REG4			(0x23)
#define REG_CTRL_REG5			(0x24)

#define REG_OUT_X_L				(0x28)

struct Vector{
    float x_axis;
    float y_axis;
    float z_axis;
};

typedef enum{
    DATARATE_3200HZ		= 0b1111,
    DATARATE_1600HZ		= 0b1110,
    DATARATE_800HZ		= 0b1101,
    DATARATE_400HZ		= 0b1100,
    DATARATE_200HZ		= 0b1011,
    DATARATE_100HZ		= 0b1010,
    DATARATE_50HZ		= 0b1001,
    DATARATE_25HZ		= 0b1000,
    DATARATE_12_5HZ		= 0b0111,
    DATARATE_6_25HZ		= 0b0110,
    DATARATE_3_13HZ		= 0b0101,
    DATARATE_1_56HZ		= 0b0100,
    DATARATE_0_78HZ		= 0b0011,
    DATARATE_0_39HZ		= 0b0010,
    DATARATE_0_20HZ		= 0b0001,
    DATARATE_0_10HZ		= 0b0000
} Accel_Data_Rate;

typedef enum
{
    DATARATE_800HZ_110	= 0b1111,
    DATARATE_800HZ_50	= 0b1110,
    DATARATE_800HZ_35	= 0b1101,
    DATARATE_800HZ_30	= 0b1100,
    DATARATE_400HZ_110	= 0b1011,
    DATARATE_400HZ_50	= 0b1010,
    DATARATE_400HZ_25	= 0b1001,
    DATARATE_400HZ_20	= 0b1000,
    DATARATE_200HZ_70	= 0b0111,
    DATARATE_200HZ_50	= 0b0110,
    DATARATE_200HZ_25	= 0b0101,
    DATARATE_200HZ_12_5	= 0b0100,
    DATARATE_100HZ_25	= 0b0001,
    DATARATE_100HZ_12_5	= 0b0000
} Gyro_Data_Rate;

typedef enum{
	RANGE_16G			= 0b11,
    RANGE_8G        	= 0b10,
    RANGE_4G			= 0b01,
    RANGE_2G		    = 0b00
} Accel_Range;

typedef enum{
    SCALE_250DPS		= 0b00,
    SCALE_500DPS		= 0b01,
    SCALE_2000DPS		= 0b10
} Gyro_Scale;

class ACCEL{
    public:
		bool begin(void);
		void clear_settings(void);
		void set_range(Accel_Range range);
		
		Vector read_normalised(float gravity_factor = GRAVITY_EARTH);
		Vector read_raw(void);
		
	private:
		Vector filtered;
		Vector normalised;
		Vector raw;
		
		uint8_t fast_register(uint8_t reg);
		void write_register(uint8_t reg, uint8_t value);
		uint8_t read_register_8(uint8_t reg);
		int16_t read_register_16(uint8_t reg);
};

class GYRO{
    public:
		bool begin(Gyro_Scale scale, Gyro_Data_Rate data_rate);
		void calibrate(uint8_t samples = 50);
		void set_threshold(uint8_t multiple = 1);
		
		Vector read_normalised();
		Vector read_raw(void);
	
	private:
		Vector normalised;
		Vector threshold;
		Vector raw;
		Vector delta;
		
		bool use_calibrate;
		float actual_threshold;
		float dps_per_digit;
		float threshold_x;
		float threshold_y;
		float threshold_z;
		
		uint8_t fast_register(uint8_t reg);
		void write_register(uint8_t reg, uint8_t value);
};