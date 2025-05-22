//#define SERIAL_OUTPUT
#if defined(SERIAL_OUTPUT)
#include <HardwareSerial.h>
#endif

#include "IMU_BNO085.h"
#include <cassert>
#include <cstring>

namespace { // use anonymous namespace to make items local to this translation unit
constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float ACC_16G_RES { 16.0 / 32768.0 };
} // end namespace

enum { BUS_TIMEOUT_MS = 100 };

enum {
    REPORT_ID_COMMAND_RESPONSE = 0xF1,
    REPORT_ID_COMMAND_REQUEST = 0xF2,
    REPORT_ID_FRS_READ_RESPONSE = 0xF3,
    REPORT_ID_FRS_READ_REQUEST = 0xF4,
    REPORT_ID_FRS_WRITE_RESPONSE = 0xF5,
    REPORT_ID_FRS_WRITE_DATA_REQUEST = 0xF6,
    REPORT_ID_FRS_WRITE_REQUEST = 0xF7,
    REPORT_ID_PRODUCT_ID_RESPONSE = 0xF8,
    REPORT_ID_PRODUCT_ID_REQUEST = 0xF9,
    REPORT_ID_TIMESTAMP_REBASE = 0xFA,
    REPORT_ID_BASE_TIMESTAMP_REFERENCE= 0xFB
};

enum { EXECUTABLE_RESET_COMPLETE = 0x1 };

enum {
    COMMAND_REPORT_ERRORS = 0x01,
    COMMAND_COUNTER_COMMANDS = 0x02, // sub-commands specified in P0
    COMMAND_TARE = 0x03,
    COMMAND_INITIALIZE = 0x04,
    COMMAND_SAVE_DYNAMIC_CALIBRATION_DATA = 0x06, // DCD
    COMMAND_CALIBRATE_MOTION_ENGINE = 0x07,
    COMMAND_CONFIGURE_PERIODIC_DCD_SAVE = 0x09,
    COMMAND_GET_OSCILLATOR_TYPE = 0x0A,
    COMMAND_CLEAR_DCD_AND_RESET = 0x0B
};

// System orientation rotation quaternions.
// The system orientation FRS record (0x2D3E) applies a rotation to the sensor outputs and all the derived outputs.
// The record is a unit quaternion, with each coordinate represented as a 32-bit fixed point number with a Q-point of 30.

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

#if defined(USE_IMU_BNO085_SPI)
IMU_BNO085::IMU_BNO085(axis_order_t axisOrder, uint32_t frequency, BUS_SPI::spi_index_t SPI_index, const BUS_SPI::pins_t& pins) :
    IMU_Base(axisOrder, _bus),
    _bus(frequency, SPI_index, pins),
    _axisOrderQuaternion(axisOrientations[axisOrder])
{
}
#else
IMU_BNO085::IMU_BNO085(axis_order_t axisOrder, BUS_I2C::i2c_index_t I2C_index, const BUS_I2C::pins_t& pins, uint8_t I2C_address) :
    IMU_Base(axisOrder, _bus),
    _bus(I2C_address, I2C_index, pins),
    _axisOrderQuaternion(axisOrientations[axisOrder])
{
}
#endif

int IMU_BNO085::init(uint32_t outputDataRateHz, gyro_sensitivity_t gyroSensitivity, acc_sensitivity_t accSensitivity, void* i2cMutex)
{
    assert(outputDataRateHz <= 400);
    (void)gyroSensitivity;
    (void)accSensitivity;
#if defined(I2C_MUTEX_REQUIRED)
    _i2cMutex = static_cast<SemaphoreHandle_t>(i2cMutex);
#else
    (void)i2cMutex;
#endif

    // transmit reset byte on channel 1
    _shtpPacket.data[0] = 1; //Reset
    sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte
    delayMs(50);
    //Read all incoming data and flush it
    while (readPacketAndParse()) { delayMs(1); }

    _shtpPacket.data[0] = REPORT_ID_PRODUCT_ID_REQUEST;
    _shtpPacket.data[1] = 0;
    sendPacket(CHANNEL_SENSOR_HUB_CONTROL, 2);
    delayMs(50);
    while (readPacketAndParse()) { delayMs(1); }

    // default to update every 2500 microseconds, 400Hz, which is highest supported rate
    setFeatureCommand(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, outputDataRateHz == 0 ? 2500 : 1000000 / outputDataRateHz, 0);
    delayMs(100);
    while (readPacketAndParse()) { delayMs(1); }

    return 0;
}

void IMU_BNO085::setFeatureCommand(uint8_t reportID, uint32_t timeBetweenReportsUs, uint32_t specificConfig)
{
    _shtpPacket.data[0] = SENSOR_REPORTID_SET_FEATURE_COMMAND;                          // Set feature command. Reference page 55
    _shtpPacket.data[1] = reportID;                                                     // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    _shtpPacket.data[2] = 0;                                                            // Feature flags
    _shtpPacket.data[3] = 0;                                                            // Change sensitivity (LSB)
    _shtpPacket.data[4] = 0;                                                            // Change sensitivity (MSB)
    _shtpPacket.data[5] = static_cast<uint8_t>((timeBetweenReportsUs >> 0) & 0xFFU);    // Report interval (LSB) in microseconds. 0x7A120 = 500ms
    _shtpPacket.data[6] = static_cast<uint8_t>((timeBetweenReportsUs >> 8) & 0xFFU);    // Report interval
    _shtpPacket.data[7] = static_cast<uint8_t>((timeBetweenReportsUs >> 16) & 0xFFU);   // Report interval
    _shtpPacket.data[8] = static_cast<uint8_t>((timeBetweenReportsUs >> 24) & 0xFFU);   // Report interval (MSB)
    _shtpPacket.data[9] = 0;                                                            // Batch Interval (LSB)
    _shtpPacket.data[10] = 0;                                                           // Batch Interval
    _shtpPacket.data[11] = 0;                                                           // Batch Interval
    _shtpPacket.data[12] = 0;                                                           // Batch Interval (MSB)
    _shtpPacket.data[13] = static_cast<uint8_t>((specificConfig >> 0) & 0xFFU);         // Sensor-specific config (LSB)
    _shtpPacket.data[14] = static_cast<uint8_t>((specificConfig >> 8) & 0xFFU);         // Sensor-specific config
    _shtpPacket.data[15] = static_cast<uint8_t>((specificConfig >> 16) & 0xFFU);        // Sensor-specific config
    _shtpPacket.data[16] = static_cast<uint8_t>((specificConfig >> 24) & 0xFFU);        // Sensor-specific config (MSB)

    sendPacket(CHANNEL_SENSOR_HUB_CONTROL, 17);
}

IMU_Base::xyz_int32_t IMU_BNO085::readGyroRaw()
{
    IMU_Base::xyz_int32_t ret {};
    return ret;
}

IMU_Base::xyz_int32_t IMU_BNO085::readAccRaw()
{
    IMU_Base::xyz_int32_t ret {};
    return ret;
}

Quaternion IMU_BNO085::readOrientation()
{
    if (!_orientationAvailable) {
        readPacketAndParse();
    }
    _orientationAvailable = false;
    // for SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A // Q point = 14 for orientation, Q point = 10 for gyro
    constexpr unsigned int Q_point = 14;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    const Quaternion integratedRotationVector(
        static_cast<float>(_gyroIntegratedRotationVector.real) * multiplier,
        static_cast<float>(_gyroIntegratedRotationVector.i) * multiplier,
        static_cast<float>(_gyroIntegratedRotationVector.j) * multiplier,
        static_cast<float>(_gyroIntegratedRotationVector.k) * multiplier
    );
    return _axisOrderQuaternion * integratedRotationVector;
}

xyz_t IMU_BNO085::readGyroRPS()
{
    if (!_gyroAvailable) {
        readPacketAndParse();
    }
    _gyroAvailable = false;
    // for SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A // Q point = 14 for orientation, Q point = 10 for gyro
    constexpr unsigned int Q_point = 10;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    return xyz_t {
        .x = static_cast<float>(_gyroIntegratedRotationVector.x) * multiplier,
        .y = static_cast<float>(_gyroIntegratedRotationVector.y) * multiplier,
        .z = static_cast<float>(_gyroIntegratedRotationVector.z) * multiplier
    };
}

xyz_t IMU_BNO085::getAcc() const
{
    constexpr unsigned int Q_point = 8;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    return xyz_t {
        .x = static_cast<float>(_acc.x) * multiplier,
        .y = static_cast<float>(_acc.y) * multiplier,
        .z = static_cast<float>(_acc.z) * multiplier
    };
}


xyz_t IMU_BNO085::getAccLinear() const
{
    constexpr unsigned int Q_point = 8;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    return xyz_t {
        .x = static_cast<float>(_accLinear.x) * multiplier,
        .y = static_cast<float>(_accLinear.y) * multiplier,
        .z = static_cast<float>(_accLinear.z) * multiplier
    };
}

xyz_t IMU_BNO085::getGyroRPS() const
{
    constexpr unsigned int Q_point = 9;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    return xyz_t {
        .x = static_cast<float>(_gyroRPS.x) * multiplier,
        .y = static_cast<float>(_gyroRPS.y) * multiplier,
        .z = static_cast<float>(_gyroRPS.z) * multiplier
    };
}

xyz_t IMU_BNO085::getMag() const
{
    constexpr unsigned int Q_point = 4;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    return xyz_t {
        .x = static_cast<float>(_mag.x) * multiplier,
        .y = static_cast<float>(_mag.y) * multiplier,
        .z = static_cast<float>(_mag.z) * multiplier
    };
}

xyz_t IMU_BNO085::getGravity() const
{
    constexpr unsigned int Q_point = 8;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    return xyz_t {
        .x = static_cast<float>(_gravity.x) * multiplier,
        .y = static_cast<float>(_gravity.y) * multiplier,
        .z = static_cast<float>(_gravity.z) * multiplier
    };
}

uint16_t IMU_BNO085::parseCommandResponse(const SHTP_Packet& packet)
{
    const uint8_t reportID = packet.data[0];
    if (reportID == REPORT_ID_COMMAND_RESPONSE) {
        //The BNO085 responds with this report to command requests.
        const uint8_t command = packet.data[2];
        if (command == COMMAND_CALIBRATE_MOTION_ENGINE) {
            _calibrationStatus = packet.data[0]; //R0 - Status (0 = success, non-zero = fail)
        }
        return reportID;
    }

    return 0;
}


/*!
The gyro integrated rotation vector input reports are sent via the special gyro channel.
They DO NOT INCLUDE the usual reportID, sequenceNumber, status, and delay fields
Rates of up to 400 Hz are supported.
*/
uint16_t IMU_BNO085::parseGyroIntegratedRotationVectorReport(const SHTP_Packet& packet)
{
    _orientationAvailable = true;
    _gyroAvailable = true;
    _gyroIntegratedRotationVector.i    = static_cast<uint16_t>(packet.data[1])  << 8U | packet.data[0];
    _gyroIntegratedRotationVector.j    = static_cast<uint16_t>(packet.data[3])  << 8U | packet.data[2];
    _gyroIntegratedRotationVector.k    = static_cast<uint16_t>(packet.data[5])  << 8U | packet.data[4];
    _gyroIntegratedRotationVector.real = static_cast<uint16_t>(packet.data[7])  << 8U | packet.data[6];
    _gyroIntegratedRotationVector.x    = static_cast<uint16_t>(packet.data[9])  << 8U | packet.data[8];
    _gyroIntegratedRotationVector.y    = static_cast<uint16_t>(packet.data[11]) << 8U | packet.data[10];
    _gyroIntegratedRotationVector.z    = static_cast<uint16_t>(packet.data[13]) << 8U | packet.data[12];
    //Serial.printf("gyro:%d,%d,%d\r\n", _gyroIntegratedRotationVector.x, _gyroIntegratedRotationVector.y, _gyroIntegratedRotationVector.z);

    return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
}

/*!
Parse the input sensor report packet.
Packet format is:
packet.header       4 byte header
packet.data[0:4]    5 byte timestamp
packet.data[5:8]    4 bytes of data containing reportID, sequenceNumber, status, and delay.
packet.date[9..]    sensor data starts
*/
uint16_t IMU_BNO085::parseInputSensorReport(const SHTP_Packet& packet)
{
    if (packet.data[0] == REPORT_ID_BASE_TIMESTAMP_REFERENCE) {
        _timestamp = (static_cast<uint32_t>(packet.data[4]) << 24U)
            | (static_cast<uint32_t>(packet.data[3]) << 16U)
            | (static_cast<uint32_t>(packet.data[2]) << 8U)
            | static_cast<uint32_t>(packet.data[1]);
    }

    /*
    Bits 1:0 - indicate the status of a sensor.
    0 - Unreliable
    1 - Accuracy low
    2 - Accuracy medium
    3 - Accuracy high
    Bits 7:2 - Delay upper bits: 6 most-significant bits of report delay
    */
    const uint8_t reportID = packet.data[5];
    const uint8_t sequenceNumber = packet.data[6];
    const uint8_t status = packet.data[7]; //Get status bits
    const uint8_t accuracy = status & 0x03U;
    const uint16_t delay = static_cast<uint16_t>(status & 0xFCU) << 6U | packet.data[8];

    const uint16_t dataX = static_cast<uint16_t>(packet.data[10]) << 8U | packet.data[9];
    const uint16_t dataY = static_cast<uint16_t>(packet.data[12]) << 8U | packet.data[11];
    const uint16_t dataZ = static_cast<uint16_t>(packet.data[14]) << 8U | packet.data[13];

    switch (reportID) {
    case SENSOR_REPORTID_ACCELEROMETER:
        _acc.x = dataX;
        _acc.y = dataY;
        _acc.z = dataZ;
        _acc.accuracy = accuracy;
        _acc.sequenceNumber = sequenceNumber;
        _acc.delay = delay;
        break;
    case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
        _gyroRPS.x = dataX;
        _gyroRPS.y = dataY;
        _gyroRPS.z = dataZ;
        _gyroRPS.accuracy = accuracy;
        _gyroRPS.sequenceNumber = sequenceNumber;
        _gyroRPS.delay = delay;
        break;
    case SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED:
        _mag.x = dataX;
        _mag.y = dataY;
        _mag.z = dataZ;
        _mag.accuracy = accuracy;
        _mag.sequenceNumber = sequenceNumber;
        _mag.delay = delay;
        break;
    case SENSOR_REPORTID_LINEAR_ACCELERATION:
        _accLinear.x = dataX;
        _accLinear.y = dataY;
        _accLinear.z = dataZ;
        _accLinear.accuracy = accuracy;
        _accLinear.sequenceNumber = sequenceNumber;
        _accLinear.delay = delay;
        break;
    case SENSOR_REPORTID_GRAVITY:
        _gravity.x = dataX;
        _gravity.y = dataY;
        _gravity.z = dataZ;
        _gravity.accuracy = accuracy;
        _gravity.sequenceNumber = sequenceNumber;
        _gravity.delay = delay;
        break;
    case SENSOR_REPORTID_GYROSCOPE_UNCALIBRATED:
        _gyroUncalibratedRPS.x = dataX;
        _gyroUncalibratedRPS.y = dataY;
        _gyroUncalibratedRPS.z = dataZ;
        _gyroUncalibratedRPS.accuracy = accuracy;
        _gyroUncalibratedRPS.sequenceNumber = sequenceNumber;
        _gyroUncalibratedRPS.delay = delay;
        _gyroUncalibratedRPS.biasX  = static_cast<uint16_t>(packet.data[16]) << 8U | packet.data[15];
        _gyroUncalibratedRPS.biasY  = static_cast<uint16_t>(packet.data[18]) << 8U | packet.data[17];
        _gyroUncalibratedRPS.biasZ  = static_cast<uint16_t>(packet.data[20]) << 8U | packet.data[19];
        break;
    case SENSOR_REPORTID_ROTATION_VECTOR: // NOLINT(bugprone-branch-clone) false positive
        [[fallthrough]];
    case SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR:
        [[fallthrough]];
    case SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR:
        _rotationVector.radianAccuracy = static_cast<uint16_t>(packet.data[18]) << 8U | packet.data[17];
        [[fallthrough]];
    // the GAME rotation vectors do not report radianAccuracy
    case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
        [[fallthrough]];
    case SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR:
        _rotationVector.i = dataX;
        _rotationVector.j = dataY;
        _rotationVector.k = dataZ;
        _rotationVector.real = static_cast<uint16_t>(packet.data[16]) << 8U | packet.data[15];
        _rotationVector.accuracy = accuracy;
        break;
    case SENSOR_REPORTID_RAW_ACCELEROMETER:
        _accRaw.x = dataX;
        _accRaw.y = dataY;
        _accRaw.z = dataZ;
        _accRaw.accuracy = accuracy;
        break;
    case SENSOR_REPORTID_RAW_GYROSCOPE:
        _gyroRaw.x = dataX;
        _gyroRaw.y = dataY;
        _gyroRaw.z = dataZ;
        _gyroRaw.accuracy = accuracy;
        break;
    case SENSOR_REPORTID_RAW_MAGNETOMETER:
        _magRaw.x = dataX;
        _magRaw.y = dataY;
        _magRaw.z = dataZ;
        _magRaw.accuracy = accuracy;
        break;
    case REPORT_ID_COMMAND_RESPONSE:
        //The BNO085 responds with this report to command requests. It's up to us to remember which command we issued.
        if (packet.data[2] == COMMAND_CALIBRATE_MOTION_ENGINE) {
            _calibrationStatus = packet.data[10]; //R0 - Status (0 = success, non-zero = fail)
        }
        break;
    default:
        //This sensor report ID is unhandled.
        //See reference manual to add additional feature reports as needed
        return 0;
    }

    return reportID;
}

bool IMU_BNO085::readPacketAndParse()
{
    static_assert(sizeof(SHTP_Header) == 4);
    if (readPacket()) {
        //Check to see if this packet is a sensor reporting its data to us
        switch (_shtpPacket.header.channel) {
        case CHANNEL_SENSOR_HUB_CONTROL: // NOLINT(bugprone-branch-clone) false positive
            parseCommandResponse(_shtpPacket);
            break;
        case CHANNEL_INPUT_SENSOR_REPORTS:
            parseInputSensorReport(_shtpPacket);
            break;
        case CHANNEL_GYRO_INTEGRATED_ROTATION_VECTOR_REPORT:
            parseGyroIntegratedRotationVectorReport(_shtpPacket);
            break;
        }
        return true;
    }
    return false;
}

bool IMU_BNO085::readPacket()
{
    // No interrupt pin set then we rely on receivePacket() to timeout. Strictly speaking this does not follow the SH-2 transport protocol.
    if (_bus.readBytesWithTimeout(reinterpret_cast<uint8_t*>(&_shtpPacket.header), sizeof(SHTP_Header), BUS_TIMEOUT_MS) == false) { // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        return false;
    }

    uint16_t dataLength = ((static_cast<uint16_t>(_shtpPacket.header.lengthMSB)) << 8) | (static_cast<uint16_t>(_shtpPacket.header.lengthLSB));
    dataLength &= 0x7FFFU; // Clear the most significant bit.
    //Serial.printf("dataLengthMSB:%0x\r\n", _shtpPacket.header.lengthMSB);
    //Serial.printf("dataLengthLSB:%0x\r\n", _shtpPacket.header.lengthLSB);
    //Serial.printf("dataLengthA:%0x\r\n", dataLength);
    if (dataLength <= sizeof(SHTP_Header)) {
        //Packet is empty
        return false;
    }
#if defined(SERIAL_OUTPUT)
    Serial.printf("\r\nDATALENGTH:%3d(0x%3X) CH:%d, SN:%d\r\n", dataLength, dataLength, _shtpPacket.header.channel, _shtpPacket.header.sequenceNumber);
#endif
    readData(dataLength - sizeof(SHTP_Header));
    //Serial.printf("timeStamp:0x%02x:%02x:%02x:%02x:%02x\r\n", _shtpPacket.timestamp[0], _shtpPacket.timestamp[1], _shtpPacket.timestamp[2], _shtpPacket.timestamp[3], _shtpPacket.timestamp[4]);
    //Serial.printf("data: %02x,%02x,%02x,%02x,%02x\r\n", _shtpPacket.data[0], _shtpPacket.data[1], _shtpPacket.data[2], _shtpPacket.data[3], _shtpPacket.data[4]);

    // Check for a reset complete packet
    if (_shtpPacket.header.channel == CHANNEL_EXECUTABLE && _shtpPacket.data[0] == EXECUTABLE_RESET_COMPLETE) {
        //Serial.printf("Reset\r\n");
        _resetCompleteReceived = true;
    }
    return true;
}

/*!
Perform multiple reads until all bytes are read
The SHTP_Packet data buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
*/
bool IMU_BNO085::readData(size_t readLength)
{
    size_t index = 0;
    int bytesToRead = static_cast<int>(readLength);

    //Setup a series of chunked 32 byte reads
    while (bytesToRead > 0) {
        //Serial.printf("bytesToRead:%d\r\n", bytesToRead);
        int readCount = bytesToRead;
        if (readCount + sizeof(SHTP_Header) > MAX_I2C_READ_LENGTH) {
            readCount = MAX_I2C_READ_LENGTH - sizeof(SHTP_Header);
        }

        std::array<uint8_t, MAX_I2C_READ_LENGTH> data;
        if (_bus.readBytesWithTimeout(reinterpret_cast<uint8_t*>(&data[0]), readCount + sizeof(SHTP_Header), BUS_TIMEOUT_MS) == false) { // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            return false;
        }

#if defined(SERIAL_OUTPUT)
        const int dataLength = (~0x8000U) & ((static_cast<uint16_t>(data[1]) << 8U) | static_cast<uint16_t>(data[0]));
        Serial.printf("dataLength:%3d(0x%3X) CH:%d, SN:%d\r\n", dataLength, dataLength, data[2], data[3]);
#endif

        //Serial.printf("readCount:%d\r\n", readCount);
        // Read a chunk of data
        if (index + readCount <= MAX_PACKET_SIZE) {
            memcpy(&_shtpPacket.data[index], &data[4], readCount);
#if defined(SERIAL_OUTPUT)
            for (int ii = 0; ii < readCount; ++ii) {
                Serial.printf("%02x ", _shtpPacket.data[index + ii]);
            }
            if (readLength > 0) {
                Serial.printf("\r\n");
            }
#endif
            index += readCount;
        } else {
            // no room for the data, so just throw it away
        }
        bytesToRead -= readCount;
    }
    return true;
}

// NOTE: Arduino has a maximum 32 byte send.
bool IMU_BNO085::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
    const uint8_t packetLength = dataLength + sizeof(_shtpPacket.header); // Add four bytes for the header
    _shtpPacket.header.lengthLSB = packetLength & 0xFF;
    _shtpPacket.header.lengthMSB = packetLength >> 8;
    _shtpPacket.header.channel = channelNumber;
    _shtpPacket.header.sequenceNumber = _sequenceNumber[channelNumber]++;
    _bus.writeBytes(reinterpret_cast<uint8_t*>(&_shtpPacket), packetLength); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)

    return true;
}

bool IMU_BNO085::sendCommandCalibrateMotionEngine()
{
    _commandMessage.P.fill(0);
    _commandMessage.P[0] = 1; // acc calibration enabled
    _commandMessage.P[1] = 1; // gyro calibration enabled
    // all other calibration disabled
    return sendCommand(COMMAND_CALIBRATE_MOTION_ENGINE);
}

bool IMU_BNO085::sendCommandSaveDynamicCalibrationData()
{
    _commandMessage.P.fill(0);
    return sendCommand(COMMAND_SAVE_DYNAMIC_CALIBRATION_DATA);
}

bool IMU_BNO085::sendCommand(uint8_t command)
{
    _commandMessage.reportID = REPORT_ID_COMMAND_REQUEST;
    _commandMessage.command = command;
    ++_commandMessage.sequenceNumber;
    _bus.writeBytes(reinterpret_cast<uint8_t*>(&_commandMessage), sizeof(_commandMessage)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)

    return true;
}
// NOLINTEND(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
