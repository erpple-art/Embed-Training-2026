#include "BNO055.h"

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
BNO055(I2C &i2c, uint8_t addr) noexcept
    : _i2c(i2c), _res(NC)
{
    _i2c_p = &i2C;
    chip_addr = addr;
    cantReadDataCount = 0;
}

// yes reset pin
BNO055(I2C &i2c, uint8_t addr, PinName p_reset) noexcept
    : _i2c(i2c), _res(p_reset)
{
    _i2c_p = &i2C;
    chip_addr = addr;
    cantReadDataCount = 0;
}


// HELPER FUNCTIONS
// write one byte to one register
void BNO055::writeRegister(char reg, char value) {
    
}


// RESET



// INIT



// ACCEL & GYRO



// PITCH, ROLL, YAW

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
