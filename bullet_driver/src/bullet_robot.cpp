#include "bullet_robot.h"


// ===================================================================================================================
// ============================================= CONSTRUCTOR =========================================================
// ===================================================================================================================

BulletRobot::BulletRobot() : bullet_spi("/dev/spidev0.1", 500000)
{
    bullet_stm_mode = BULLET_MODE_PREOPERATIONAL;
    changeBulletSTMMode(BULLET_MODE_PREOPERATIONAL);
}
BulletRobot::~BulletRobot()
{
    changeBulletSTMMode(BULLET_MODE_PREOPERATIONAL);
}

// ===================================================================================================================
// =============================================== HELP METHODS ======================================================
// ===================================================================================================================

BulletStatus BulletRobot::readByte(const BulletSPICommand command, const BulletReadWriteInstruction instruction, const uint8_t servo_id, uint8_t &data)
{
    bullet_spi << uint8_t(command) << uint8_t(instruction) << servo_id;
    bullet_spi.calc_msg_out();

    BulletStatus ret = bullet_spi.transfer(3);
    data = bullet_spi.message_in.at(1);
    bullet_spi.clear();

    return ret;
}
BulletStatus BulletRobot::readWord(const BulletSPICommand command, const BulletReadWriteInstruction instruction, const uint8_t servo_id, uint16_t &data)
{
    bullet_spi << uint8_t(command) << uint8_t(instruction) << servo_id;
    bullet_spi.calc_msg_out();

    BulletStatus ret = bullet_spi.transfer(4);
    data = bullet_spi.message_in.at(1) | (bullet_spi.message_in.at(2) << 8);
    bullet_spi.clear();

    return ret;
}

BulletStatus BulletRobot::writeByte(const BulletSPICommand command, const BulletReadWriteInstruction instruction, const uint8_t servo_id, const uint8_t data)
{
    bullet_spi << uint8_t(command) << uint8_t(instruction) << servo_id << data;
    bullet_spi.calc_msg_out();

    BulletStatus ret = bullet_spi.transfer(2);
    bullet_spi.clear();

    return ret;
}
BulletStatus BulletRobot::writeWord(const BulletSPICommand command, const BulletReadWriteInstruction instruction, const uint8_t servo_id, const uint16_t data)
{
    bullet_spi << uint8_t(command) << uint8_t(instruction) << uint8_t(servo_id) << (uint8_t)data << uint8_t(data>>8);
    bullet_spi.calc_msg_out();
    bullet_spi.print_message_out();

    BulletStatus ret = bullet_spi.transfer(2);
    bullet_spi.clear();

    return ret;
}

// ===================================================================================================================
// ================================================ PING =============================================================
// ===================================================================================================================

BulletStatus BulletRobot::getBulletSTMMode(BulletSTMMode &mode)
{
    bullet_spi << uint8_t(BULLET_COMMAND_PING) << uint8_t(BULLET_INSTRUCTION_COMMUNICATION);
    bullet_spi.calc_msg_out();

    BulletStatus ret = bullet_spi.transfer(4);
    bullet_spi.print_message_in();
    if (ret != BULLET_SUCCESS) exit(-1);
    mode = (BulletSTMMode)bullet_spi.message_in.at(1);
    bullet_spi.clear();

    return ret;
}
BulletStatus BulletRobot::pingServo(uint8_t servo)
{
    bullet_spi << uint8_t(BULLET_COMMAND_PING) << uint8_t(BULLET_INSTRUCTION_SERVO) << servo;
    bullet_spi.calc_msg_out();

    BulletStatus ret = bullet_spi.transfer(2);
    bullet_spi.clear();

    return ret;
}
BulletStatus BulletRobot::pingAllServos()
{
    bullet_spi << uint8_t(BULLET_COMMAND_PING) << uint8_t(BULLET_INSTRUCTION_SERVOS_ALL);
    bullet_spi.calc_msg_out();

    BulletStatus ret = bullet_spi.transfer(2);
    bullet_spi.clear();

    return ret;
}

// ===================================================================================================================
// ================================================ CHANGE MODE ======================================================
// ===================================================================================================================

BulletStatus BulletRobot::changeBulletSTMMode(const BulletSTMMode mode)
{
    bullet_spi << uint8_t(BULLET_COMMAND_CHANGE_MODE) << uint8_t(0x00) << uint8_t(mode);
    bullet_spi.calc_msg_out();

    BulletStatus ret = bullet_spi.transfer(2);
    if (ret == BULLET_SUCCESS) bullet_stm_mode = mode;
    bullet_spi.clear();

    return ret;
}

// ===================================================================================================================
// ================================================== READ EEPROM ====================================================
// ===================================================================================================================

BulletStatus BulletRobot::readEEpromModelNumber(const uint8_t servo_id, uint16_t &model_number)
{
    if (bullet_stm_mode == BULLET_MODE_OPERATIONAL) return BULLET_ERROR_MODE_MISMATCH;
    return readWord(BULLET_COMMAND_READ, BULLET_INSTRUCTION_MODEL_NUMBER, servo_id, model_number);
}
BulletStatus BulletRobot::readEEpromFirmwareVersion(const uint8_t servo_id, uint8_t &firmware_version)
{
    if (bullet_stm_mode == BULLET_MODE_OPERATIONAL) return BULLET_ERROR_MODE_MISMATCH;
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_FIRMWARE_VERSION, servo_id, firmware_version);
}
BulletStatus BulletRobot::readEEpromReturnDelayTime(const uint8_t servo_id, uint8_t &return_delay_time)
{
    if (bullet_stm_mode == BULLET_MODE_OPERATIONAL) return BULLET_ERROR_MODE_MISMATCH;
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_RETURN_DELAY_TIME, servo_id, return_delay_time);
}
BulletStatus BulletRobot::readEEpromCWAngleLimit(const uint8_t servo_id, uint16_t &cw_angle_limit)
{
    return readWord(BULLET_COMMAND_READ, BULLET_INSTRUCTION_CW_ANGLE_LIMIT, servo_id, cw_angle_limit);
}
BulletStatus BulletRobot::readEEpromCCWAngleLimit(const uint8_t servo_id, uint16_t &ccw_angle_limit)
{
    return readWord(BULLET_COMMAND_READ, BULLET_INSTRUCTION_CCW_ANGLE_LIMIT, servo_id, ccw_angle_limit);
}
BulletStatus BulletRobot::readEEpromHighestLimitTemperature(const uint8_t servo_id, uint8_t &highest_limit_temperature)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_HIGHEST_LIMIT_TEMPERATURE, servo_id, highest_limit_temperature);
}
BulletStatus BulletRobot::readEEpromLowestLimitVoltage(const uint8_t servo_id, uint8_t &lowest_limit_voltage)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_LOWEST_LIMIT_VOLTAGE, servo_id, lowest_limit_voltage);
}
BulletStatus BulletRobot::readEEpromHighestLimitVoltage(const uint8_t servo_id, uint8_t &highest_limit_voltage)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_HIGHEST_LIMIT_VOLTAGE, servo_id, highest_limit_voltage);
}
BulletStatus BulletRobot::readEEpromMaxTorque(const uint8_t servo_id, uint16_t &max_torque)
{
    return readWord(BULLET_COMMAND_READ, BULLET_INSTRUCTION_MAX_TORQUE, servo_id, max_torque);
}
BulletStatus BulletRobot::readEEpromStatusReturnLevel(const uint8_t servo_id, uint8_t &status_return_level)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_STATUS_RETURN_LEVEL, servo_id, status_return_level);
}
BulletStatus BulletRobot::readEEpromAlarmLED(const uint8_t servo_id, uint8_t &alarm_led)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_ALARM_LED, servo_id, alarm_led);
}
BulletStatus BulletRobot::readEEpromAlarmShutdown(const uint8_t servo_id, uint8_t &alarm_shutdown)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_ALARM_SHUTDOWN, servo_id, alarm_shutdown);
}

// ===================================================================================================================
// ================================================= WRITE EEPROM ====================================================
// ===================================================================================================================

BulletStatus BulletRobot::writeEEpromID(const uint8_t new_id)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_ID, BULLET_BROADCAST_ID, new_id);
}
BulletStatus BulletRobot::writeEEpromBaudrate(const uint8_t new_baudrate)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_BAUDRATE, BULLET_BROADCAST_ID, new_baudrate);
}
BulletStatus BulletRobot::writeEEpromReturnDelayTime(const uint8_t servo_id, const uint8_t new_return_delay_time)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_ID, servo_id, new_return_delay_time);
}
BulletStatus BulletRobot::writeEEpromCWAngleLimit(const uint8_t servo_id, const uint16_t new_cw_angle_limit)
{
    return writeWord(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_CW_ANGLE_LIMIT, servo_id, new_cw_angle_limit);
}
BulletStatus BulletRobot::writeEEpromCCWAngleLimit(const uint8_t servo_id, const uint16_t new_ccw_angle_limit)
{
    return writeWord(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_CCW_ANGLE_LIMIT, servo_id, new_ccw_angle_limit);
}
BulletStatus BulletRobot::writeEEpromHighestLimitTemperature(const uint8_t servo_id, const uint8_t new_highest_limit_temperature)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_HIGHEST_LIMIT_TEMPERATURE, servo_id, new_highest_limit_temperature);
}
BulletStatus BulletRobot::writeEEpromLowestLimitVoltage(const uint8_t servo_id, const uint8_t new_lowest_limit_voltage)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_LOWEST_LIMIT_VOLTAGE, servo_id, new_lowest_limit_voltage);
}
BulletStatus BulletRobot::writeEEpromHighestLimitVoltage(const uint8_t servo_id, const uint8_t new_highest_limit_voltage)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_HIGHEST_LIMIT_VOLTAGE, servo_id, new_highest_limit_voltage);
}
BulletStatus BulletRobot::writeEEpromMaxTorque(const uint8_t servo_id, const uint16_t new_max_torque)
{
    return writeWord(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_MAX_TORQUE, servo_id, new_max_torque);
}
BulletStatus BulletRobot::writeEEpromStatusReturnLevel(const uint8_t servo_id, const uint8_t new_status_return_level)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_STATUS_RETURN_LEVEL, servo_id, new_status_return_level);
}
BulletStatus BulletRobot::writeEEpromAlarmLED(const uint8_t servo_id, const uint8_t new_alarm_led)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_ALARM_LED, servo_id, new_alarm_led);
}
BulletStatus BulletRobot::writeEEpromAlarmShutdown(const uint8_t servo_id, const uint8_t new_alarm_shutdown)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_ALARM_SHUTDOWN, servo_id, new_alarm_shutdown);
}

// ===================================================================================================================
// =================================================== READ RAM ======================================================
// ===================================================================================================================

BulletStatus BulletRobot::readRamTorqueEnable(const uint8_t servo_id, uint8_t &torque_enable)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_TORQUE_ENABLE, servo_id, torque_enable);
}
BulletStatus BulletRobot::readRamLED(const uint8_t servo_id, uint8_t &led)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_LED, servo_id, led);
}
BulletStatus BulletRobot::readRamCWComplianceMargin(const uint8_t servo_id, uint8_t &cw_compliance_margin)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_CW_COMPLIANCE_MARGIN, servo_id, cw_compliance_margin);
}
BulletStatus BulletRobot::readRamCCWComplianceMargin(const uint8_t servo_id, uint8_t &ccw_compliance_margin)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_CCW_COMPLIANCE_MARGIN, servo_id, ccw_compliance_margin);
}
BulletStatus BulletRobot::readRamCWComplianceSlope(const uint8_t servo_id, uint8_t &cw_compliance_slope)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_CW_COMPLIANCE_SLOPE, servo_id, cw_compliance_slope);
}
BulletStatus BulletRobot::readRamCCWComplianceSlope(const uint8_t servo_id, uint8_t &ccw_compliance_slope)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_CCW_COMPLIANCE_SLOPE, servo_id, ccw_compliance_slope);
}
BulletStatus BulletRobot::readRamGoalPosition(const uint8_t servo_id, uint16_t &goal_position)
{
    return readWord(BULLET_COMMAND_READ, BULLET_INSTRUCTION_GOAL_POSITION, servo_id, goal_position);
}
BulletStatus BulletRobot::readRamMovingSpeed(const uint8_t servo_id, uint16_t &moving_speed)
{
    return readWord(BULLET_COMMAND_READ, BULLET_INSTRUCTION_MOVING_SPEED, servo_id, moving_speed);
}
BulletStatus BulletRobot::readRamTorqueLimit(const uint8_t servo_id, uint16_t &torque_limit)
{
    return readWord(BULLET_COMMAND_READ, BULLET_INSTRUCTION_TORQUE_LIMIT, servo_id, torque_limit);
}
BulletStatus BulletRobot::readRamPresentPosition(const uint8_t servo_id, uint16_t &present_position)
{
    return readWord(BULLET_COMMAND_READ, BULLET_INSTRUCTION_PRESENT_POSITION, servo_id, present_position);
}
BulletStatus BulletRobot::readRamPresentSpeed(const uint8_t servo_id, uint16_t &present_speed)
{
    return readWord(BULLET_COMMAND_READ, BULLET_INSTRUCTION_PRESENT_SPEED, servo_id, present_speed);
}
BulletStatus BulletRobot::readRamPresentLoad(const uint8_t servo_id, uint16_t &present_load)
{
    return readWord(BULLET_COMMAND_READ, BULLET_INSTRUCTION_PRESENT_LOAD, servo_id, present_load);
}
BulletStatus BulletRobot::readRamPresentVoltage(const uint8_t servo_id, uint8_t &present_voltage)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_PRESENT_VOLTAGE, servo_id, present_voltage);
}
BulletStatus BulletRobot::readRamPresentTemperature(const uint8_t servo_id, uint8_t &present_temperature)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_PRESENT_TEMPERATURE, servo_id, present_temperature);
}
BulletStatus BulletRobot::readRamRegisteredInstruction(const uint8_t servo_id, uint8_t &registered_instruction)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_REGISTERED_INSTRUCTION, servo_id, registered_instruction);
}
BulletStatus BulletRobot::readRamMoving(const uint8_t servo_id, uint8_t &moving)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_MOVING, servo_id, moving);
}
BulletStatus BulletRobot::readRamLock(const uint8_t servo_id, uint8_t &lock)
{
    return readByte(BULLET_COMMAND_READ, BULLET_INSTRUCTION_LOCK, servo_id, lock);
}
BulletStatus BulletRobot::readRamPunch(const uint8_t servo_id, uint16_t &punch)
{
    return readWord(BULLET_COMMAND_READ, BULLET_INSTRUCTION_PUNCH, servo_id, punch);
}

// ===================================================================================================================
// =================================================== WRITE RAM =====================================================
// ===================================================================================================================

BulletStatus BulletRobot::writeRamTorqueEnable(const uint8_t servo_id, const uint8_t new_torque_enable)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_TORQUE_ENABLE, servo_id, new_torque_enable);
}
BulletStatus BulletRobot::writeRamLED(const uint8_t servo_id, const uint8_t new_led)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_LED, servo_id, new_led);
}
BulletStatus BulletRobot::writeRamCWComplianceMargin(const uint8_t servo_id, const uint8_t new_cw_compliance_margin)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_CW_COMPLIANCE_MARGIN, servo_id, new_cw_compliance_margin);
}
BulletStatus BulletRobot::writeRamCCWComplianceMargin(const uint8_t servo_id, const uint8_t new_ccw_compliance_margin)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_CCW_COMPLIANCE_MARGIN, servo_id, new_ccw_compliance_margin);
}
BulletStatus BulletRobot::writeRamCWComplianceSlope(const uint8_t servo_id, const uint8_t new_cw_compliance_slope)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_CW_COMPLIANCE_SLOPE, servo_id, new_cw_compliance_slope);
}
BulletStatus BulletRobot::writeRamCCWComplianceSlope(const uint8_t servo_id, const uint8_t new_ccw_compliance_slope)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_CCW_COMPLIANCE_SLOPE, servo_id, new_ccw_compliance_slope);
}
BulletStatus BulletRobot::writeRamGoalPosition(const uint8_t servo_id, const uint16_t new_goal_position)
{
    return writeWord(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_GOAL_POSITION, servo_id, new_goal_position);
}
BulletStatus BulletRobot::writeRamMovingSpeed(const uint8_t servo_id, const uint16_t new_moving_speed)
{
    return writeWord(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_MOVING_SPEED, servo_id, new_moving_speed);
}
BulletStatus BulletRobot::writeRamTorqueLimit(const uint8_t servo_id, const uint16_t new_torque_limit)
{
    return writeWord(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_TORQUE_LIMIT, servo_id, new_torque_limit);
}
BulletStatus BulletRobot::writeRamRegisteredInstruction(const uint8_t servo_id, const uint8_t new_registered_instruction)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_REGISTERED_INSTRUCTION, servo_id, new_registered_instruction);
}
BulletStatus BulletRobot::writeRamLock(const uint8_t servo_id, const uint8_t new_lock)
{
    return writeByte(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_LOCK, servo_id, new_lock);
}
BulletStatus BulletRobot::writeRamPunch(const uint8_t servo_id, const uint16_t new_punch)
{
    return writeWord(BULLET_COMMAND_WRITE, BULLET_INSTRUCTION_PUNCH, servo_id, new_punch);
}

// ===================================================================================================================
// =================================================== READ ALL ======================================================
// ===================================================================================================================

BulletStatus BulletRobot::readAllPositionSpeedTorque(uint16_t* position, uint16_t *speed, uint16_t *torque)
{
    bullet_spi << uint8_t(BULLET_COMMAND_READ_ALL) << uint8_t(0x00);
    bullet_spi.calc_msg_out();

    BulletStatus ret = bullet_spi.transfer(110);
    for (int i=0; i<18; ++i)
    {
        position[i] = bullet_spi.message_in.at(6*i+1) | (bullet_spi.message_in.at(6*i+2) << 8);
        speed[i]    = bullet_spi.message_in.at(6*i+3) | (bullet_spi.message_in.at(6*i+4) << 8);
        torque[i]   = bullet_spi.message_in.at(6*i+5) | (bullet_spi.message_in.at(6*i+6) << 8);
    }
    bullet_spi.clear();

    return ret;
}
BulletStatus BulletRobot::writeAllPosition(uint16_t *position)
{
    bullet_spi << uint8_t(BULLET_COMMAND_WRITE_ALL) << uint8_t(0x00);
    for (int i=0; i<18; ++i) bullet_spi << (uint8_t)position[i] << (uint8_t)(position[i]>>8);
    bullet_spi.calc_msg_out();

    BulletStatus ret = bullet_spi.transfer(2);
    bullet_spi.clear();

    return ret;
}


