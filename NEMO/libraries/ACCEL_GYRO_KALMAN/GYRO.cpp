#include <Wire.h>
#include <math.h>

#include <ACCEL_GYRO_KALMAN.h>

bool GYRO::begin(Gyro_Scale scale, Gyro_Data_Rate data_rate)
{
    // Reset calibrate values
    delta.x_axis = 0;
    delta.y_axis = 0;
    delta.z_axis = 0;
    use_calibrate = false;

    // Reset threshold values
    threshold.x_axis = 0;
    threshold.y_axis = 0;
    threshold.z_axis = 0;
    actual_threshold = 0;

    // Check L3G4200D Who Am I Register
    if (fast_register(REG_WHO_AM_I) != 0xD3){
		return false;
    }

    // Enable all axis and setup normal mode + Output Data Range & Bandwidth
    uint8_t reg = 0x00;
    reg |= 0x0F; // Enable all axis and setup normal mode
    reg |= (data_rate << 4); // Set output data rate & bandwidh
    write_register(REG_CTRL_REG1, reg);

    // Disable high pass filter
    write_register(REG_CTRL_REG2, 0x00);

    // Generata data ready interrupt on INT2
    write_register(REG_CTRL_REG3, 0x08);

    // Set full scale selection in continous mode
    write_register(REG_CTRL_REG4, scale << 4);

    switch(scale){
		case SCALE_250DPS:
			dps_per_digit = .00875f;
			break;
		case SCALE_500DPS:
			dps_per_digit = .0175f;
			break;
		case SCALE_2000DPS:
			dps_per_digit = .07f;
			break;
		default:
			break;
    }

    // Boot in normal mode, disable FIFO, HPF disabled
    write_register(REG_CTRL_REG5, 0x00);

    return true;
}

// Calibrate algorithm
void GYRO::calibrate(uint8_t samples){
    // Set calibrate
    use_calibrate = true;

    // Reset values
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;
    float sigma_x = 0;
    float sigma_y = 0;
    float sigma_z = 0;

    // Read n-samples
    for (uint8_t i = 0; i < samples; ++i){
		read_raw();
		sum_x += raw.x_axis;
		sum_y += raw.y_axis;
		sum_z += raw.z_axis;

		sigma_x += raw.x_axis * raw.x_axis;
		sigma_y += raw.y_axis * raw.y_axis;
		sigma_z += raw.z_axis * raw.z_axis;
		
		delay(5);
    }

    // Calculate delta vectors
    delta.x_axis = sum_x / samples;
    delta.y_axis = sum_y / samples;
    delta.z_axis = sum_z / samples;

    // Calculate threshold vectors
    threshold_x = sqrt((sigma_x / samples) - (delta.x_axis * delta.x_axis));
    threshold_y = sqrt((sigma_y / samples) - (delta.y_axis * delta.y_axis));
    threshold_z = sqrt((sigma_z / samples) - (delta.z_axis * delta.z_axis));

    // If already set threshold, recalculate threshold vectors
    //if (actual_threshold > 0){
	//	set_threshold(actual_threshold);
    //}
	set_threshold(actual_threshold);
}

// Set treshold value
void GYRO::set_threshold(uint8_t multiple){
    if (multiple > 0){
		// If not calibrated, need calibrate
		if (!use_calibrate){
			calibrate();
		}
		
		// Calculate threshold vectors
		threshold.x_axis = threshold_x * multiple;
		threshold.y_axis = threshold_y * multiple;
		threshold.z_axis = threshold_z * multiple;
	} else {
		// No threshold
		threshold.x_axis = 0;
		threshold.y_axis = 0;
		threshold.z_axis = 0;
    }

    // Remember old threshold value
    actual_threshold = multiple;
}

// Read normalized values
Vector GYRO::read_normalised(){
    read_raw();

    if (use_calibrate){
		normalised.x_axis = (raw.x_axis - delta.x_axis) * dps_per_digit;
		normalised.y_axis = (raw.y_axis - delta.y_axis) * dps_per_digit;
		normalised.z_axis = (raw.z_axis - delta.z_axis) * dps_per_digit;
    } else {
		normalised.x_axis = raw.x_axis * dps_per_digit;
		normalised.y_axis = raw.y_axis * dps_per_digit;
		normalised.z_axis = raw.z_axis * dps_per_digit;
		normalised.z_axis = raw.z_axis * dps_per_digit;
    }

    if (actual_threshold > 0){
		if (abs(normalised.x_axis) < threshold.x_axis) normalised.x_axis = 0;
		if (abs(normalised.y_axis) < threshold.y_axis) normalised.y_axis = 0;
		if (abs(normalised.z_axis) < threshold.z_axis) normalised.z_axis = 0;
    }

    return normalised;
}

// Read raw values
Vector GYRO::read_raw(){
    Wire.beginTransmission(GYRO_ADDRESS);
	Wire.write(REG_OUT_X_L | (1 << 7)); 
    Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDRESS, 6);

    while (Wire.available() < 6);

	uint8_t xla = Wire.read();
	uint8_t xha = Wire.read();
	uint8_t yla = Wire.read();
	uint8_t yha = Wire.read();
	uint8_t zla = Wire.read();
	uint8_t zha = Wire.read();

    raw.x_axis = xha << 8 | xla;
    raw.y_axis = yha << 8 | yla;
    raw.z_axis = zha << 8 | zla;

    return raw;
}

// Read byte to register
uint8_t GYRO::fast_register(uint8_t reg){
    uint8_t value;
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(GYRO_ADDRESS, 1);
    value = Wire.read();
    Wire.endTransmission();

    return value;
}


void GYRO::write_register(uint8_t reg, uint8_t value){
	Wire.beginTransmission(GYRO_ADDRESS);
	Wire.write(reg);
	Wire.write(value);
    Wire.endTransmission();
}