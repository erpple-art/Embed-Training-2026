#include "BNO055.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

/*
 registers used:
   0x00 = chip ID
   0x08 = start of acceleration data
   0x14 = start of gyro data
   0x3D = mode
   0x3E = power mode
   0x3F = reset
 
 Mode values:
   0x00 = config mode (settings only)
   0x0C = 9DOF mode (everything on)
*/

// CONSTRUCTORS

// no reset pin
BNO055::BNO055(I2C &i2c, uint8_t addr) noexcept
    : _i2c(i2c), _res(NC)
{
    _i2c_p = &i2c;
    chip_addr = addr;
    cantReadDataCount = 0;
}

// yes reset pin
BNO055::BNO055(I2C &i2c, uint8_t addr, PinName p_reset) noexcept
    : _i2c(i2c), _res(p_reset)
{
    _i2c_p = &i2c;
    chip_addr = addr;
    cantReadDataCount = 0;
}

// HELPER FUNCTIONS
// write one byte to one register
// reg = what register, value = what to put in register
// aaasduhaugeyrhiughfgbsudhiruegj
void BNO055::writeRegister(char reg, char value) {
    char data[2] = {reg, value};
    _i2c.write(chip_addr, data, 2, false); 
}

void BNO055::readRegisters(char startReg, char *buf, int length) {
    char reg[1] = {startReg};               // make one byte array w/ register we are starting from
    _i2c.write(chip_addr, reg, 1, true);        // sends one byte which points chip at starting register
    _i2c.read(chip_addr, buf, length, false);   // reads length bytes from chip into buf; chip moves to next register after each byte
}

// RESET
void BNO055::reset() noexcept {
    writeRegister(BNO055_SYS_TRIGGER, 0x20);
}


// CHANGE MODE
void BNO055::change_fusion_mode(uint8_t mode) {
    writeRegister(BNO055_OPR_MODE, MODE_CONFIG); // put chip in config mode
    ThisThread::sleep_for(30ms);                            
    writeRegister(BNO055_OPR_MODE, mode);        // put chip in desired mode
    ThisThread::sleep_for(30ms);                            

}


// INIT
void BNO055::init() noexcept {
    // check chip ID
    uint8_t id;
    readRegisters(BNO055_CHIP_ID, reinterpret_cast<char *>(&id), 1);
    // check if chip ID is correct
    if (id != 0xA0) {
        printf("BNO055: Chip ID mismatch. Expected 0xA0, got 0x%02X\n", id);
        return;
    }

    // reset chip
    reset();
    ThisThread::sleep_for(650ms);

    // set power mode to normal
    writeRegister(BNO055_PWR_MODE, 0x00);
    ThisThread::sleep_for(10ms);

    // set fusion mode to NDOF
    change_fusion_mode(MODE_NDOF);
}

// ACCEL
void BNO055::get_accel(BNO055_VECTOR_TypeDef *acc) {
    uint8_t buf[6];   // x low, x high, y low, y high, z low, z high

    // read 6 registers in a row, starting at the accel data register
    readRegisters(BNO055_ACCEL_DATA, (char *)buf, 6);

    // combine high byte and low byte into one 16-bit number
    int16_t x = (int16_t)((buf[1] << 8) | buf[0]);  // i feel like we should stop calling donut holes donut holes
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);  // because they're not holes, they're more like plugs
    int16_t z = (int16_t)((buf[5] << 8) | buf[4]);  // donut plugs

    // convert to m/s^2 (datasheet: 1 m/s^2 = 100 LSB)
    acc->x = x / 100.0;
    acc->y = y / 100.0;
    acc->z = z / 100.0;
}

// GYRO
void BNO055::get_gyro(BNO055_VECTOR_TypeDef *gyro) {
    uint8_t buf[6];

    readRegisters(BNO055_GYRO_DATA, (char *)buf, 6);

    int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t z = (int16_t)((buf[5] << 8) | buf[4]);

    gyro->x = x / 16.0; // convert to degrees/s (datasheet: 1 deg/s = 16 LSB)
    gyro->y = y / 16.0;
    gyro->z = z / 16.0;
}

/*
We added these two functions since they're definitely beyond what we expect from you as recruits
I'd be impressed if you understood the math behind them.
Basically just know that quats are 4 axis (1 real, 3 imaginary) numbers that allow us 
to nicely talk about rotation, and the functions spit out pitch roll and yaw. 
*/
void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *result)
{
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t w,x,y,z;

    dt[0] = BNO055_QUATERNION_W_LSB;
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult)  {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 8, false);
        w = dt[1] << 8 | dt[0];
        x = dt[3] << 8 | dt[2];
        y = dt[5] << 8 | dt[4];
        z = dt[7] << 8 | dt[6];

        result->w = (double)w / 16384.0f;
        result->x = (double)x / 16384.0f;
        result->y = (double)y / 16384.0f;
        result->z = (double)z / 16384.0f;
    } else {
        cantReadDataCount++;
    }
}

void BNO055::get_angular_position_quat(IMU::EulerAngles *result){

    BNO055_QUATERNION_TypeDef q;
    get_quaternion(&q);

    float roll  = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / PI;
    float pitch = asin(2 * q.w * q.y - q.x * q.z) * 180 / PI;
    float yaw   = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / PI;

    memcpy(&result->roll, &roll, sizeof(float));
    memcpy(&result->pitch, &pitch, sizeof(float));
    memcpy(&result->yaw, &yaw, sizeof(float));
}

IMU::EulerAngles BNO055::read() {
    get_angular_position_quat(&imuAngles);
    return imuAngles;
}

IMU::EulerAngles BNO055::getImuAngles() {
    return imuAngles;
}