/*
MPU6050.nut - MPU6050 Triple Axis Gyroscope & Accelerometer Arduino Library.
Version: 1.0.3
(c) 2014-2015 Korneliusz Jarzebski
www.jarzebski.pl

Port to Esquilo 20161219 Leeland Heins

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

const MPU6050_ADDRESS             = 0x68;  // 0x69 when AD0 pin to Vcc

const MPU6050_REG_ACCEL_XOFFS_H     = 0x06;
const MPU6050_REG_ACCEL_XOFFS_L     = 0x07;
const MPU6050_REG_ACCEL_YOFFS_H     = 0x08;
const MPU6050_REG_ACCEL_YOFFS_L     = 0x09;
const MPU6050_REG_ACCEL_ZOFFS_H     = 0x0A;
const MPU6050_REG_ACCEL_ZOFFS_L     = 0x0B;
const MPU6050_REG_GYRO_XOFFS_H      = 0x13;
const MPU6050_REG_GYRO_XOFFS_L      = 0x14;
const MPU6050_REG_GYRO_YOFFS_H      = 0x15;
const MPU6050_REG_GYRO_YOFFS_L      = 0x16;
const MPU6050_REG_GYRO_ZOFFS_H      = 0x17;
const MPU6050_REG_GYRO_ZOFFS_L      = 0x18;
const MPU6050_REG_CONFIG            = 0x1A;
const MPU6050_REG_GYRO_CONFIG       = 0x1B;  // Gyroscope Configuration
const MPU6050_REG_ACCEL_CONFIG      = 0x1C;  // Accelerometer Configuration
const MPU6050_REG_FF_THRESHOLD      = 0x1D;
const MPU6050_REG_FF_DURATION       = 0x1E;
const MPU6050_REG_MOT_THRESHOLD     = 0x1F;
const MPU6050_REG_MOT_DURATION      = 0x20;
const MPU6050_REG_ZMOT_THRESHOLD    = 0x21;
const MPU6050_REG_ZMOT_DURATION     = 0x22;
const MPU6050_REG_INT_PIN_CFG       = 0x37;  // INT Pin. Bypass Enable Configuration
const MPU6050_REG_INT_ENABLE        = 0x38;  // INT Enable
const MPU6050_REG_INT_STATUS        = 0x3A;
const MPU6050_REG_ACCEL_XOUT_H      = 0x3B;
const MPU6050_REG_ACCEL_XOUT_L      = 0x3C;
const MPU6050_REG_ACCEL_YOUT_H      = 0x3D;
const MPU6050_REG_ACCEL_YOUT_L      = 0x3E;
const MPU6050_REG_ACCEL_ZOUT_H      = 0x3F;
const MPU6050_REG_ACCEL_ZOUT_L      = 0x40;
const MPU6050_REG_TEMP_OUT_H        = 0x41;
const MPU6050_REG_TEMP_OUT_L        = 0x42;
const MPU6050_REG_GYRO_XOUT_H       = 0x43;
const MPU6050_REG_GYRO_XOUT_L       = 0x44;
const MPU6050_REG_GYRO_YOUT_H       = 0x45;
const MPU6050_REG_GYRO_YOUT_L       = 0x46;
const MPU6050_REG_GYRO_ZOUT_H       = 0x47;
const MPU6050_REG_GYRO_ZOUT_L       = 0x48;
const MPU6050_REG_MOT_DETECT_STATUS = 0x61;
const MPU6050_REG_MOT_DETECT_CTRL   = 0x69;
const MPU6050_REG_USER_CTRL         = 0x6A;  // User Control
const MPU6050_REG_PWR_MGMT_1        = 0x6B;  // Power Management 1
const MPU6050_REG_WHO_AM_I          = 0x75;  // Who Am I


// Clocksource
const MPU6050_CLOCK_KEEP_RESET      = 0b111;
const MPU6050_CLOCK_EXTERNAL_19MHZ  = 0b101;
const MPU6050_CLOCK_EXTERNAL_32KHZ  = 0b100;
const MPU6050_CLOCK_PLL_ZGYRO       = 0b011;
const MPU6050_CLOCK_PLL_YGYRO       = 0b010;
const MPU6050_CLOCK_PLL_XGYRO       = 0b001;
const MPU6050_CLOCK_INTERNAL_8MHZ   = 0b000;

// dps
const MPU6050_SCALE_2000DPS         = 0b11;
const MPU6050_SCALE_1000DPS         = 0b10;
const MPU6050_SCALE_500DPS          = 0b01;
const MPU6050_SCALE_250DPS          = 0b00;

// range
const MPU6050_RANGE_16G             = 0b11;
const MPU6050_RANGE_8G              = 0b10;
const MPU6050_RANGE_4G              = 0b01;
const MPU6050_RANGE_2G              = 0b00;

// delay
const MPU6050_DELAY_3MS             = 0b11;
const MPU6050_DELAY_2MS             = 0b10;
const MPU6050_DELAY_1MS             = 0b01;
const MPU6050_NO_DELAY              = 0b00;

// dhpf
const MPU6050_DHPF_HOLD             = 0b111;
const MPU6050_DHPF_0_63HZ           = 0b100;
const MPU6050_DHPF_1_25HZ           = 0b011;
const MPU6050_DHPF_2_5HZ            = 0b010;
const MPU6050_DHPF_5HZ              = 0b001;
const MPU6050_DHPF_RESET            = 0b000;

// dlpf
const MPU6050_DLPF_6                = 0b110;
const MPU6050_DLPF_5                = 0b101;
const MPU6050_DLPF_4                = 0b100;
const MPU6050_DLPF_3                = 0b011;
const MPU6050_DLPF_2                = 0b010;
const MPU6050_DLPF_1                = 0b001;
const MPU6050_DLPF_0                = 0b000;


class MPU6050
{
    i2c = null;
    addr = null;

    // calibrate values
    dg = { 'XAxis':0, 'YAxis':0, ZAxis:0 };
    useCalibrate = false;

    // threshold values
    tg = { 'XAxis':0, 'YAxis':0, ZAxis:0 };
    th = { 'XAxis':0, 'YAxis':0, ZAxis:0 };
    actualThreshold = 0;

    // Activities
    isOverflow = 0;
    isFreeFall = 0;
    isInactivity = 0;
    isActivity = 0;
    isPosActivityOnX = 0;
    isPosActivityOnY = 0;
    isPosActivityOnZ = 0;
    isNegActivityOnX = 0;
    isNegActivityOnY = 0;
    isNegActivityOnZ = 0;
    isDataReady = 0;

    rangePerDigit = .000061f;
    dpsPerDigit = .007633f;

    constructor(_i2c, _addr)
    {
        i2c = _i2c;
        addr = _addr;
    }
};


function MPU6050::begin(scale, range, mpua)
{
    // Set Address
    i2c.address(addr);

    // Reset calibrate values
    dg.XAxis = 0;
    dg.YAxis = 0;
    dg.ZAxis = 0;
    useCalibrate = false;

    // Reset threshold values
    tg.XAxis = 0;
    tg.YAxis = 0;
    tg.ZAxis = 0;
    actualThreshold = 0;

    // Check MPU6050 Who Am I Register
    if (fastRegister8(MPU6050_REG_WHO_AM_I) != 0x68) {
        return false;
    }

    // Set Clock Source
    setClockSource(MPU6050_CLOCK_PLL_XGYRO);

    // Set Scale & Range
    setScale(scale);
    setRange(range);

    // Disable Sleep Mode
    setSleepEnabled(false);

    return true;
}

function MPU6050::setScale(scale)
{
    local value;

    switch (scale) {
        case MPU6050_SCALE_250DPS:
            dpsPerDigit = .007633f;
            break;
        case MPU6050_SCALE_500DPS:
            dpsPerDigit = .015267f;
            break;
        case MPU6050_SCALE_1000DPS:
            dpsPerDigit = .030487f;
            break;
        case MPU6050_SCALE_2000DPS:
            dpsPerDigit = .060975f;
            break;
        default:
            break;
    }

    value = readRegister8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);

    writeRegister8(MPU6050_REG_GYRO_CONFIG, value);
}

function MPU6050::getScale()
{
    local value;

    value = readRegister8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b00011000;
    value >>= 3;

    return (mpu6050_dps_t)value;
}

function MPU6050::setRange(range)
{
    local value;

    switch (range) {
        case MPU6050_RANGE_2G:
            rangePerDigit = .000061f;
            break;
        case MPU6050_RANGE_4G:
            rangePerDigit = .000122f;
            break;
        case MPU6050_RANGE_8G:
            rangePerDigit = .000244f;
            break;
        case MPU6050_RANGE_16G:
            rangePerDigit = .0004882f;
            break;
        default:
            break;
    }

    value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);

    writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
}

function MPU6050::getRange()
{
    local value;

    value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b00011000;
    value >>= 3;

    return (mpu6050_range_t)value;
}

function MPU6050::setDHPFMode(dhpf)
{
    local value;

    value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11111000;
    value |= dhpf;

    writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
}

function MPU6050::setDLPFMode(dlpf)
{
    local value;

    value = readRegister8(MPU6050_REG_CONFIG);
    value &= 0b11111000;
    value |= dlpf;

    writeRegister8(MPU6050_REG_CONFIG, value);
}

function MPU6050::setClockSource(source)
{
    local value;

    value = readRegister8(MPU6050_REG_PWR_MGMT_1);
    value &= 0b11111000;
    value |= source;

    writeRegister8(MPU6050_REG_PWR_MGMT_1, value);
}

function MPU6050::getClockSource()
{
    local value;

    value = readRegister8(MPU6050_REG_PWR_MGMT_1);
    value &= 0b00000111;

    return (mpu6050_clockSource_t)value;
}

function MPU6050::getSleepEnabled()
{
    return readRegisterBit(MPU6050_REG_PWR_MGMT_1, 6);
}

function MPU6050::setSleepEnabled(state)
{
    writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 6, state);
}

function MPU6050::getIntZeroMotionEnabled()
{
    return readRegisterBit(MPU6050_REG_INT_ENABLE, 5);
}

function MPU6050::setIntZeroMotionEnabled(state)
{
    writeRegisterBit(MPU6050_REG_INT_ENABLE, 5, state);
}

function MPU6050::getIntMotionEnabled()
{
    return readRegisterBit(MPU6050_REG_INT_ENABLE, 6);
}

function MPU6050::setIntMotionEnabled(state)
{
    writeRegisterBit(MPU6050_REG_INT_ENABLE, 6, state);
}

function MPU6050::getIntFreeFallEnabled()
{
    return readRegisterBit(MPU6050_REG_INT_ENABLE, 7);
}

function MPU6050::setIntFreeFallEnabled(state)
{
    writeRegisterBit(MPU6050_REG_INT_ENABLE, 7, state);
}

function MPU6050::getMotionDetectionThreshold()
{
    return readRegister8(MPU6050_REG_MOT_THRESHOLD);
}

function MPU6050::setMotionDetectionThreshold(threshold)
{
    writeRegister8(MPU6050_REG_MOT_THRESHOLD, threshold);
}

function MPU6050::getMotionDetectionDuration()
{
    return readRegister8(MPU6050_REG_MOT_DURATION);
}

function MPU6050::setMotionDetectionDuration(duration)
{
    writeRegister8(MPU6050_REG_MOT_DURATION, duration);
}

function MPU6050::getZeroMotionDetectionThreshold()
{
    return readRegister8(MPU6050_REG_ZMOT_THRESHOLD);
}

function MPU6050::setZeroMotionDetectionThreshold(threshold)
{
    writeRegister8(MPU6050_REG_ZMOT_THRESHOLD, threshold);
}

function MPU6050::getZeroMotionDetectionDuration()
{
    return readRegister8(MPU6050_REG_ZMOT_DURATION);
}

function MPU6050::setZeroMotionDetectionDuration(duration)
{
    writeRegister8(MPU6050_REG_ZMOT_DURATION, duration);
}

function MPU6050::getFreeFallDetectionThreshold()
{
    return readRegister8(MPU6050_REG_FF_THRESHOLD);
}

function MPU6050::setFreeFallDetectionThreshold(threshold)
{
    writeRegister8(MPU6050_REG_FF_THRESHOLD, threshold);
}

function MPU6050::getFreeFallDetectionDuration()
{
    return readRegister8(MPU6050_REG_FF_DURATION);
}

function MPU6050::setFreeFallDetectionDuration(duration)
{
    writeRegister8(MPU6050_REG_FF_DURATION, duration);
}

function MPU6050::getI2CMasterModeEnabled()
{
    return readRegisterBit(MPU6050_REG_USER_CTRL, 5);
}

function MPU6050::setI2CMasterModeEnabled(state)
{
    writeRegisterBit(MPU6050_REG_USER_CTRL, 5, state);
}

function MPU6050::setI2CBypassEnabled(state)
{
    return writeRegisterBit(MPU6050_REG_INT_PIN_CFG, 1, state);
}

function MPU6050::getI2CBypassEnabled()
{
    return readRegisterBit(MPU6050_REG_INT_PIN_CFG, 1);
}

function MPU6050::setAccelPowerOnDelay(delay)
{
    local value;

    value = readRegister8(MPU6050_REG_MOT_DETECT_CTRL);
    value &= 0b11001111;
    value |= (delay << 4);

    writeRegister8(MPU6050_REG_MOT_DETECT_CTRL, value);
}

function MPU6050::getAccelPowerOnDelay()
{
    local value;

    value = readRegister8(MPU6050_REG_MOT_DETECT_CTRL);
    value &= 0b00110000;

    return (mpu6050_onDelay_t)(value >> 4);
}

function MPU6050::getIntStatus()
{
    return readRegister8(MPU6050_REG_INT_STATUS);
}

Activites MPU6050::readActivites()
{
    local data = readRegister8(MPU6050_REG_INT_STATUS);

    isOverflow = ((data >> 4) & 1);
    isFreeFall = ((data >> 7) & 1);
    isInactivity = ((data >> 5) & 1);
    isActivity = ((data >> 6) & 1);
    isDataReady = ((data >> 0) & 1);

    data = readRegister8(MPU6050_REG_MOT_DETECT_STATUS);

    isNegActivityOnX = ((data >> 7) & 1);
    isPosActivityOnX = ((data >> 6) & 1);

    isNegActivityOnY = ((data >> 5) & 1);
    isPosActivityOnY = ((data >> 4) & 1);

    isNegActivityOnZ = ((data >> 3) & 1);
    isPosActivityOnZ = ((data >> 2) & 1);
}

function MPU6050::readRawAccel()
{
    local writeBlob = blob(1);
    local readBlob = blob(6);

    writeBlob[0] = MPU6050_REG_ACCEL_XOUT_H;
    readBlob[0] = 0;
    readBlob[1] = 0;
    readBlob[2] = 0;
    readBlob[3] = 0;
    readBlob[4] = 0;
    readBlob[5] = 0;

    i2c.address(addr);

    i2c.xfer(writeBlob, readBlob);

    local xha = readBlob[0];
    local xla = readBlob[1];
    local yha = readBlob[2];
    local yla = readBlob[3];
    local zha = readBlob[4];
    local zla = readBlob[5];

    ra.XAxis = xha << 8 | xla;
    ra.YAxis = yha << 8 | yla;
    ra.ZAxis = zha << 8 | zla;

    return ra;
}

function MPU6050::readNormalizeAccel()
{
    local na = { 'XAxis':0, 'YAxis':0, ZAxis:0 };

    local ra = readRawAccel();

    na.XAxis = ra.XAxis * rangePerDigit * 9.80665f;
    na.YAxis = ra.YAxis * rangePerDigit * 9.80665f;
    na.ZAxis = ra.ZAxis * rangePerDigit * 9.80665f;

    return na;
}

function MPU6050::readScaledAccel()
{
    local na = { 'XAxis':0, 'YAxis':0, ZAxis:0 };
    local ra = { 'XAxis':0, 'YAxis':0, ZAxis:0 };

    ra = readRawAccel();

    na.XAxis = ra.XAxis * rangePerDigit;
    na.YAxis = ra.YAxis * rangePerDigit;
    na.ZAxis = ra.ZAxis * rangePerDigit;

    return na;
}

function MPU6050::readRawGyro()
{
    local rg;
    local writeBlob = blob(1);
    local readBlob = blob(6);

    rg = { 'XAxis':0, 'YAxis':0, ZAxis:0 };

    writeBlob[0] = MPU6050_REG_GYRO_XOUT_H;
    readBlob[0] = 0;
    readBlob[1] = 0;
    readBlob[2] = 0;
    readBlob[3] = 0;
    readBlob[4] = 0;
    readBlob[5] = 0;

    i2c.address(addr);
    i2c.xfer(writeBlob, readBlob);

    local xha = readBlob[0];
    local xla = readBlob[1];
    local yha = readBlob[2];
    local yla = readBlob[3];
    local zha = readBlob[4];
    local zla = readBlob[5];

    rg.XAxis = xha << 8 | xla;
    rg.YAxis = yha << 8 | yla;
    rg.ZAxis = zha << 8 | zla;

    return rg;
}

function MPU6050::readNormalizeGyro()
{
    local rg = readRawGyro();
    local ng = { 'XAxis':0, 'YAxis':0, ZAxis:0 };

    if (useCalibrate) {
        ng.XAxis = (rg.XAxis - dg.XAxis) * dpsPerDigit;
        ng.YAxis = (rg.YAxis - dg.YAxis) * dpsPerDigit;
        ng.ZAxis = (rg.ZAxis - dg.ZAxis) * dpsPerDigit;
    } else {
        ng.XAxis = rg.XAxis * dpsPerDigit;
        ng.YAxis = rg.YAxis * dpsPerDigit;
        ng.ZAxis = rg.ZAxis * dpsPerDigit;
    }

    if (actualThreshold) {
        if (abs(ng.XAxis) < tg.XAxis) ng.XAxis = 0;
        if (abs(ng.YAxis) < tg.YAxis) ng.YAxis = 0;
        if (abs(ng.ZAxis) < tg.ZAxis) ng.ZAxis = 0;
    }

    return ng;
}

function MPU6050::readTemperature()
{
    local T;

    T = readRegister16(MPU6050_REG_TEMP_OUT_H);

    return T / 340 + 36.53;
}

function MPU6050::getGyroOffsetX()
{
    return readRegister16(MPU6050_REG_GYRO_XOFFS_H);
}

function MPU6050::getGyroOffsetY()
{
    return readRegister16(MPU6050_REG_GYRO_YOFFS_H);
}

function MPU6050::getGyroOffsetZ()
{
    return readRegister16(MPU6050_REG_GYRO_ZOFFS_H);
}

function MPU6050::setGyroOffsetX(offset)
{
    writeRegister16(MPU6050_REG_GYRO_XOFFS_H, offset);
}

function MPU6050::setGyroOffsetY(offset)
{
    writeRegister16(MPU6050_REG_GYRO_YOFFS_H, offset);
}

function MPU6050::setGyroOffsetZ(offset)
{
    writeRegister16(MPU6050_REG_GYRO_ZOFFS_H, offset);
}

function MPU6050::getAccelOffsetX()
{
    return readRegister16(MPU6050_REG_ACCEL_XOFFS_H);
}

function MPU6050::getAccelOffsetY()
{
    return readRegister16(MPU6050_REG_ACCEL_YOFFS_H);
}

function MPU6050::getAccelOffsetZ()
{
    return readRegister16(MPU6050_REG_ACCEL_ZOFFS_H);
}

function MPU6050::setAccelOffsetX(offset)
{
    writeRegister16(MPU6050_REG_ACCEL_XOFFS_H, offset);
}

function MPU6050::setAccelOffsetY(offset)
{
    writeRegister16(MPU6050_REG_ACCEL_YOFFS_H, offset);
}

function MPU6050::setAccelOffsetZ(offset)
{
    writeRegister16(MPU6050_REG_ACCEL_ZOFFS_H, offset);
}

// Calibrate algorithm
function MPU6050::calibrateGyro(samples)
{
    // Set calibrate
    useCalibrate = true;

    // Reset values
    local sumX = 0;
    local sumY = 0;
    local sumZ = 0;
    local sigmaX = 0;
    local sigmaY = 0;
    local sigmaZ = 0;

    local i;

    // Read n-samples
    for (i = 0; i < samples; ++i) {
        local rg = readRawGyro();
        sumX += rg.XAxis;
        sumY += rg.YAxis;
        sumZ += rg.ZAxis;

        sigmaX += rg.XAxis * rg.XAxis;
        sigmaY += rg.YAxis * rg.YAxis;
        sigmaZ += rg.ZAxis * rg.ZAxis;

        delay(5);
    }

    // Calculate delta vectors
    dg.XAxis = sumX / samples;
    dg.YAxis = sumY / samples;
    dg.ZAxis = sumZ / samples;

    // Calculate threshold vectors
    th.XAxis = sqrt((sigmaX / 50) - (dg.XAxis * dg.XAxis));
    th.YAxis = sqrt((sigmaY / 50) - (dg.YAxis * dg.YAxis));
    th.ZAxis = sqrt((sigmaZ / 50) - (dg.ZAxis * dg.ZAxis));

    // If already set threshold, recalculate threshold vectors
    if (actualThreshold > 0) {
        setThreshold(actualThreshold);
    }
}

// Get current threshold value
function MPU6050::getThreshold()
{
    return actualThreshold;
}

// Set treshold value
function MPU6050::setThreshold(multiple)
{
    if (multiple > 0) {
        // If not calibrated, need calibrate
        if (!useCalibrate) {
            calibrateGyro();
        }

        // Calculate threshold vectors
        tg.XAxis = th.XAxis * multiple;
        tg.YAxis = th.YAxis * multiple;
        tg.ZAxis = th.ZAxis * multiple;
    } else {
        // No threshold
        tg.XAxis = 0;
        tg.YAxis = 0;
        tg.ZAxis = 0;
    }

    // Remember old threshold value
    actualThreshold = multiple;
}

// Fast read 8-bit from register
function MPU6050::fastRegister8(reg)
{
    local value;
    local writeBlob = blob(1);
    local readBlob = blob(1);

    writeBlob[0] = reg;
    readBlob[0] = 0;

    i2c.address(addr);

    i2c.xfer(writeBlob, readBlob);

    value = readBlob[0]

    return value;
}

// Read 8-bit from register
function MPU6050::readRegister8(reg)
{
    local value;
    local writeBlob = blob(1);
    local readBlob = blob(1);

    writeBlob[0] = reg;
    readBlob[0] = 0;

    i2c.address(addr);

    i2c.xfer(writeBlob, readBlob);

    value = readBlob[0];

    return value;
}

// Write 8-bit to register
function MPU6050::writeRegister8(reg, value)
{
    local writeBlob = blob(2);

    writeBlob[0] = reg;
    writeBlob[1] = value;

    i2c.address(addr);

    i2c.write(writeBlob);
}

function MPU6050::readRegister16(reg)
{
    local value;
    local writeBlob = blob(1);
    local readBlob = blob(2);

    writeBlob[0] = reg;
    readBlob[0] = 0;
    readBlob[1] = 0;

    i2c.address();

    i2c.xfer(writeBlob, readBlob);

    local vha = readBlob[0];
    local vla = readBlob[1];

    value = vha << 8 | vla;

    return value;
}

function MPU6050::writeRegister16(reg, value)
{
    local writeBlob = blob(3);

    writeBlob[0] = reg;
    writeBlob[1] = (value >> 8);
    writeBlob[2] = value & 0xff;

    i2c.address(addr);
    i2c.write(writeBlob);
}

// Read register bit
function MPU6050::readRegisterBit(reg, pos)
{
    local value;
    value = readRegister8(reg);
    return ((value >> pos) & 1);
}

// Write register bit
function MPU6050::writeRegisterBit(reg, pos, state)
{
    local value;
    value = readRegister8(reg);

    if (state) {
        value |= (1 << pos);
    } else {
        value &= ~(1 << pos);
    }

    writeRegister8(reg, value);
}

