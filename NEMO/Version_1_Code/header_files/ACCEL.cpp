#include <Wire.h>

#include <ACCEL_GYRO_KALMAN.h>

bool ACCEL::begin(){
    filtered.x_axis = 0;
    filtered.y_axis = 0;
    filtered.y_axis = 0;

    // Check ADXL345 REG DEVID
    if (fast_register(REG_DEVID) != 0xE5){
        return false;
    }

    // Enable measurement mode (0b00001000)
    write_register(REG_POWER_CTL, 0x08);

    clear_settings();

    return true;
}

void ACCEL::clear_settings(void){
    set_range(RANGE_2G);
	write_register(REG_BW_RATE, DATARATE_100HZ);

    write_register(REG_THRESH_TAP, 0x00);
    write_register(REG_DUR, 0x00);
    write_register(REG_LATENT, 0x00);
    write_register(REG_WINDOW, 0x00);
    write_register(REG_THRESH_ACT, 0x00);
    write_register(REG_THRESH_INACT, 0x00);
    write_register(REG_TIME_INACT, 0x00);
    write_register(REG_THRESH_FF, 0x00);
    write_register(REG_TIME_FF, 0x00);

    uint8_t value;

    value = read_register_8(REG_ACT_INACT_CTL);
    value &= 0b10001000;
    write_register(REG_ACT_INACT_CTL, value);

    value = read_register_8(REG_TAP_AXES);
    value &= 0b11111000;
    write_register(REG_TAP_AXES, value);
}

// Set Range
void ACCEL::set_range(Accel_Range range){
  // Get actual value register
  uint8_t value = read_register_8(REG_DATA_FORMAT);

  // Update the data rate
  // (&) 0b11110000 (0xF0 - Leave HSB)
  // (|) 0b0000xx?? (range - Set range)
  // (|) 0b00001000 (0x08 - Set Full Res)
  value &= 0xF0;
  value |= range;
  value |= 0x08;

  write_register(REG_DATA_FORMAT, value);
}

// Read normalized values
Vector ACCEL::read_normalised(float gravity_factor){
    read_raw();

    // (4 mg/LSB scale factor in Full Res) * gravity factor
    normalised.x_axis = raw.x_axis * 0.004 * gravity_factor;
    normalised.y_axis = raw.y_axis * 0.004 * gravity_factor;
    normalised.z_axis = raw.z_axis * 0.004 * gravity_factor;

    return normalised;
}

// Read raw values
Vector ACCEL::read_raw(void){
    raw.x_axis = read_register_16(REG_DATA_X0);
    raw.y_axis = read_register_16(REG_DATA_Y0);
    raw.z_axis = read_register_16(REG_DATA_Z0);
    return raw;
}

// Read byte to register
uint8_t ACCEL::fast_register(uint8_t reg){
    uint8_t value;
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(ACCEL_ADDRESS, 1);
    value = Wire.read();
    Wire.endTransmission();

    return value;
}

// Write byte to register
void ACCEL::write_register(uint8_t reg, uint8_t value){
    Wire.beginTransmission(ACCEL_ADDRESS);
	Wire.write(reg);
	Wire.write(value);
    Wire.endTransmission();
}

// Read byte from register
uint8_t ACCEL::read_register_8(uint8_t reg){
    uint8_t value;
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.requestFrom(ACCEL_ADDRESS, 1);
    while(!Wire.available()) {};
    value = Wire.read();
    Wire.endTransmission();

    return value;
}

// Read word from register
int16_t ACCEL::read_register_16(uint8_t reg){
    int16_t value;
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(ACCEL_ADDRESS, 2);
    while(!Wire.available()) {};
    uint8_t vla = Wire.read();
    uint8_t vha = Wire.read();
    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}