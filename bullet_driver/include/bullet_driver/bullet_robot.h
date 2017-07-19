#ifndef BULLET_ROBOT_H
#define BULLET_ROBOT_H


#include "bullet_datatypes.h"
#include "bullet_spi.h"


class BulletRobot
{
private:

    //========================================================
    //===================== VARIABLES ========================
    //========================================================

    /**
     * @brief bullet_spi            SPI object for communicating with the Bullet robot.
     */
    BulletSPI bullet_spi;


    /**
     * @brief bullet_stm_mode       Mode of the STM of the Bullet robot.
     */
    BulletSTMMode bullet_stm_mode;


    //========================================================
    //==================== HELP METHODS ======================
    //========================================================

    /**
     * @brief readByte              Read a byte from the Bullet robot.
     * @param command               Command (see @BulletSPICommand)
     * @param instruction           Instruction (see @BulletReadWriteInstruction)
     * @param servo_id              Servo id.
     * @param data                  Variable to store data to.
     * @return                      BulletStatus type.
     */
    BulletStatus readByte(const BulletSPICommand command, const BulletReadWriteInstruction instruction, const uint8_t servo_id, uint8_t &data);

    /**
     * @brief readByte              Read a word from the Bullet robot.
     * @param command               Command (see @BulletSPICommand)
     * @param instruction           Instruction (see @BulletReadWriteInstruction)
     * @param servo_id              Servo id.
     * @param data                  Variable to store data to.
     * @return                      BulletStatus type.
     */
    BulletStatus readWord(const BulletSPICommand command, const BulletReadWriteInstruction instruction, const uint8_t servo_id, uint16_t &data);

    /**
     * @brief writeByte             Write a byte to the Bullet robot.
     * @param command               Command (see @BulletSPICommand)
     * @param instruction           Instruction (see @BulletReadWriteInstruction)
     * @param servo_id              Servo id.
     * @param data                  Data to send.
     * @return                      BulletStatus type.
     */
    BulletStatus writeByte(const BulletSPICommand command, const BulletReadWriteInstruction instruction, const uint8_t servo_id, const uint8_t data);

    /**
     * @brief writeByte             Write a word to the Bullet robot.
     * @param command               Command (see @BulletSPICommand)
     * @param instruction           Instruction (see @BulletReadWriteInstruction)
     * @param servo_id              Servo id.
     * @param data                  Data to send.
     * @return                      BulletStatus type.
     */
    BulletStatus writeWord(const BulletSPICommand command, const BulletReadWriteInstruction instruction, const uint8_t servo_id, const uint16_t data);


public:

    //========================================================
    //==================== CONSTRUCTOR =======================
    //========================================================

    /**
     * @brief BulletRobot           Default constructor of the class. Initializing SPI device inside.
     */
    BulletRobot();
    ~BulletRobot();

    //========================================================
    //======================= PING ===========================
    //========================================================

    /**
     * @brief getBulletSTMMode      Ask STM about it's mode.
     * @return                      BulletSTMMode type.
     */
     BulletStatus getBulletSTMMode(BulletSTMMode &mode);

    /**
     * @brief pingServo             Ping perticular servo.
     * @param servo                 Servo id.
     * @return                      BulletStatus type.
     */
    BulletStatus pingServo(uint8_t servo);

    /**
     * @brief pingAllServos         Ping all servos.
     * @return                      BulletStatus type.
     */
    BulletStatus pingAllServos();

    //========================================================
    //==================== CHANGE MODE =======================
    //========================================================

    BulletStatus changeBulletSTMMode(const BulletSTMMode mode);

    //========================================================
    //==================== EEPROM READ =======================
    //========================================================

    /**
     * @brief readEEpromModelNumber     Read model number of specified servo.
     * @param servo_id                  Servo id.
     * @param model_number              Model number.
     * @return                          BulletStatus type.
     */
    BulletStatus readEEpromModelNumber(const uint8_t servo_id, uint16_t &model_number);

    /**
     * @brief readEEpromFirmwareVersion Read firmware version of specified servo.
     * @param servo_id                  Servo id.
     * @param firmware_version          Firmware version.
     * @return                          BulletStatus type.
     */
    BulletStatus readEEpromFirmwareVersion(const uint8_t servo_id, uint8_t &firmware_version);

    /**
     * @brief readEEpromReturnDelayTime Read return delay time of specified servo. Returned value
     *                                  should be multiplied with 2us.
     * @param servo_id                  Servo id.
     * @param return_delay_time         Return Delay Time.
     * @return                          BulletStatus type.
     */
    BulletStatus readEEpromReturnDelayTime(const uint8_t servo_id, uint8_t &return_delay_time);

    /**
     * @brief readEEpromCWAngleLimit    Read lower angle limit of specified servo.
     * @param servo_id                  Servo id.
     * @param cw_angle_limit            Lower angle limit.
     * @return                          BulletStatus type.
     */
    BulletStatus readEEpromCWAngleLimit(const uint8_t servo_id, uint16_t &cw_angle_limit);

    /**
     * @brief readEEpromCCWAngleLimit   Read upper angle limit of specified servo.
     * @param servo_id                  Servo id.
     * @param ccw_angle_limit           Upper angle limit.
     * @return                          BulletStatus type.
     */
    BulletStatus readEEpromCCWAngleLimit(const uint8_t servo_id, uint16_t &ccw_angle_limit);

    /**
     * @brief readEEpromHighestLimitTemperature Read upper limit of servo operating temperature (Celsius degrees).
     * @param servo_id                          Servo id.
     * @param highest_limit_temperature         Upper temperature limit in Celsius degrees.
     * @return                                  BulletStatus type.
     */
    BulletStatus readEEpromHighestLimitTemperature(const uint8_t servo_id, uint8_t &highest_limit_temperature);

    /**
     * @brief readEEpromLowestLimitVoltage      Read lowest voltage limit of specified servo (multiplied with 10).
     * @param servo_id                          Servo id.
     * @param lowest_limit_voltage              Lowest voltage limit multiply with 10.
     * @return                                  BulletStatus type.
     */
    BulletStatus readEEpromLowestLimitVoltage(const uint8_t servo_id, uint8_t &lowest_limit_voltage);

    /**
     * @brief readEEpromHighestLimitVoltage     Read highest voltage limit of specified servo (multiplied with 10).
     * @param servo_id                          Servo id.
     * @param highest_limit_voltage             Highest voltage limit multiply with 10.
     * @return                                  BulletStatus type.
     */
    BulletStatus readEEpromHighestLimitVoltage(const uint8_t servo_id, uint8_t &highest_limit_voltage);

    /**
     * @brief readEEpromMaxTorque       Read the maximum output torque of specified servo. If it is set to 0 then
     *                                  servo is in free running mode. Possible values (0-1023)
     * @param servo_id                  Servo id.
     * @param max_torque                Max torque.
     * @return                          BulletStatus type.
     */
    BulletStatus readEEpromMaxTorque(const uint8_t servo_id, uint16_t &max_torque);

    /**
     * @brief readEEpromStatusReturnLevel       Read return status of specified servo. Possible values:
     *                                          0 - Do not respond to any instructions.
     *                                          1 - Respond only to read instructions.
     *                                          2 - Respond to all instructions
     * @param servo_id                          Servo id.
     * @param status_return_level               Status return level as specified above.
     * @return                                  BulletStatus type.
     */
    BulletStatus readEEpromStatusReturnLevel(const uint8_t servo_id, uint8_t &status_return_level);

    /**
     * @brief readEEpromAlarmLED        Read alarm led status of specified servo. Possible bit values:
     *                                  bit 0 - input voltage error
     *                                  bit 1 - angle limit error
     *                                  bit 2 - overheating error
     *                                  bit 3 - range error
     *                                  bit 4 - checksum error
     *                                  bit 5 - overload error
     *                                  bit 6 - instruction error
     * @param servo_id                  Seevo id.
     * @param alarm_led                 Alarm LED values as specified above.
     * @return                          BulletStatus type.
     */
    BulletStatus readEEpromAlarmLED(const uint8_t servo_id, uint8_t &alarm_led);

    /**
     * @brief readEEpromAlarmShutdown   Read alarm shutdown of specified servo. Torque is off if specified
     *                                  bit is set (as listed in @readEEpromAlarmLED).
     * @param servo_id                  Servo id.
     * @param alarm_shutdown            Alarm shutdown value as specified above.
     * @return                          BulletStatus type.
     */
    BulletStatus readEEpromAlarmShutdown(const uint8_t servo_id, uint8_t &alarm_shutdown);

    //========================================================
    //=================== EEPROM WRITE =======================
    //========================================================

    /**
     * @brief writeEEpromID     Write new servo id (it should be in range 0x00 - 0xFC). This message is
     *                          send in broadcasting mode so there should be only one servo connected to
     *                          STM board.
     * @param new_id            New servo id value (0x00 - 0xFC)
     * @return                  BulletStatus type.
     */
    BulletStatus writeEEpromID(const uint8_t new_id);

    /**
     * @brief writeEEpromBaudrate   Write new baudrate of STM <-> Dynamixel communication. It is done by
     *                              the following formula: Speed (BPS) = 2000000 / (baudrate + 1). Default
     *                              value is 1 -> 2000000 / (1 + 1) = 1 MBPS
     * @param new_baudrate          Baudrate value as specified above.
     * @return                      BulletStatus type.
     */
    BulletStatus writeEEpromBaudrate(const uint8_t new_baudrate);

    /**
     * @brief writeEEpromReturnDelayTime    Write return delay time of specified servo. The real value
     *                                      is calculated as follows: RDT (us) = return_delay_time * 2us
     * @param new_return_delay_time         Return delay time as specified above.
     * @return                              BulletStatus type.
     */
    BulletStatus writeEEpromReturnDelayTime(const uint8_t servo_id, const uint8_t new_return_delay_time);

    /**
     * @brief writeEEpromCWAngleLimit       Write lower angle limit of specified servo.
     * @param new_cw_angle_limit            Lower angle limit value.
     * @return                              BulletStatus type.
     */
    BulletStatus writeEEpromCWAngleLimit(const uint8_t servo_id, const uint16_t new_cw_angle_limit);

    /**
     * @brief writeEEpromCCWAngleLimit      Write upper angle limit of specified servo.
     * @param new_ccw_angle_limit           Upper angle limit value.
     * @return                              BulletStatus type.
     */
    BulletStatus writeEEpromCCWAngleLimit(const uint8_t servo_id, const uint16_t new_ccw_angle_limit);

    /**
     * @brief writeEEpromHighestLimitTemperature    Write upper limit of servo operating temperature
     *                                              - value is presented in Celsius degrees.
     * @param new_highest_limit_temperature         Upper temperature limit in Celsius degrees.
     * @return                                      BulletStatus type.
     */
    BulletStatus writeEEpromHighestLimitTemperature(const uint8_t servo_id, const uint8_t new_highest_limit_temperature);

    /**
     * @brief writeEEpromLowestLimitVoltage         Write lowest voltage limit of specified servo.
     *                                              Real value should be multiplied with 10.
     * @param new_lowest_limit_voltage              Lower voltage limit multiplied with 10.
     * @return                                      BulletStatus type.
     */
    BulletStatus writeEEpromLowestLimitVoltage(const uint8_t servo_id, const uint8_t new_lowest_limit_voltage);

    /**
     * @brief writeEEpromHighestLimitVoltage        Write upper voltage limit of specified servo.
     *                                              Real value should be multiplied with 10.
     * @param new_highest_limit_voltage             Upper voltage limit multiplied with 10.
     * @return                                      Success value.
     */
    BulletStatus writeEEpromHighestLimitVoltage(const uint8_t servo_id, const uint8_t new_highest_limit_voltage);

    /**
     * @brief writeEEpromMaxTorque          Write the maximum output torque of specified servo. If it's set
     *                                      to 0 then servo is in free running mode. Possible values (0-1023)
     * @param new_max_torque                Max torque value.
     * @return                              BulletStatus type.
     */
    BulletStatus writeEEpromMaxTorque(const uint8_t servo_id, const uint16_t new_max_torque);

    /**
     * @brief writeEEpromStatusReturnLevel  Write return status of specified servo. Possible values:
     *                                      0 - Do not respond to any instructions.
     *                                      1 - Respond only to read instructions.
     *                                      2 - Respond to all instructions
     * @param new_status_return_level       Status return level value as specified above.
     * @return                              BulletStatus type.
     */
    BulletStatus writeEEpromStatusReturnLevel(const uint8_t servo_id, const uint8_t new_status_return_level);

    /**
     * @brief writeEEpromAlarmLED           Write alarm led status of specified servo. Possible bit values:
     *                                      bit 0 - input voltage error
     *                                      bit 1 - angle limit error
     *                                      bit 2 - overheating error
     *                                      bit 3 - range error
     *                                      bit 4 - checksum error
     *                                      bit 5 - overload error
     *                                      bit 6 - instruction error
     * @param new_alarm_led                 Alarm LED value as specified above.
     * @return                              BulletStatus type.
     */
    BulletStatus writeEEpromAlarmLED(const uint8_t servo_id, const uint8_t new_alarm_led);

    /**
     * @brief writeEEpromAlarmShutdown      Wirte alarm shutdown of specified servo. Torque is off if specified
     *                                      bit is set (as listed in @writeEEpromAlarmLED).
     * @param new_alarm_shutdown            Alarm shutdown value as specified above.
     * @return                              BulletStatus type.
     */
    BulletStatus writeEEpromAlarmShutdown(const uint8_t servo_id, const uint8_t new_alarm_shutdown);

    //========================================================
    //====================== RAM READ ========================
    //========================================================

    /**
     * @brief readRamTorqueEnable       When the power is first turned on, the servo enters the Torque Free Run mode.
     *                                  If this parameter is set to 1 the torque is enable.
     * @param servo_id                  Servo id.
     * @param torque_enable             Torque enable value as specified above.
     * @return                          BulletStatus type.
     */
    BulletStatus readRamTorqueEnable(const uint8_t servo_id, uint8_t &torque_enable);

    /**
     * @brief readRamLED        Read wheter servo's led is turned on (1) or off (0).
     * @param servo_id          Servo id.
     * @param led               Led value as specified aboce.
     * @return                  BulletStatus type.
     */
    BulletStatus readRamLED(const uint8_t servo_id, uint8_t &led);

    /**
     * @brief readRamCWComplianceMargin     The compliance of the dynamixel actuator. Please read the documentation
     *                                      for more information.
     * @param servo_id                      Servo id.
     * @param cw_compliance_margin          Lower compliance margin value.
     * @return                              BulletStatus type.
     */
    BulletStatus readRamCWComplianceMargin(const uint8_t servo_id, uint8_t &cw_compliance_margin);

    /**
     * @brief readRamCCWComplianceMargin    The compliance of the dynamixel actuator. Please read the documentation
     *                                      for more information.
     * @param servo_id                      Servo id.
     * @param ccw_compliance_margin         Upper compliance margin value.
     * @return                              BulletStatus type.
     */
    BulletStatus readRamCCWComplianceMargin(const uint8_t servo_id, uint8_t &ccw_compliance_margin);

    /**
     * @brief readRamCWComplianceSlope      The compliance of the dynamixel actuator. Please read the documentation
     *                                      for more information.
     * @param servo_id                      Servo id.
     * @param cw_compliance_slope           Lower compliance slope value.
     * @return                              BulletStatus type.
     */
    BulletStatus readRamCWComplianceSlope(const uint8_t servo_id, uint8_t &cw_compliance_slope);

    /**
     * @brief readRamCCWComplianceSlope     The compliance of the dynamixel actuator. Please read the documentation
     *                                      for more information.
     * @param servo_id                      Servo id.
     * @param ccw_compliance_slope          Upper compliance slope value.
     * @return                              BulletStatus type.
     */
    BulletStatus readRamCCWComplianceSlope(const uint8_t servo_id, uint8_t &ccw_compliance_slope);

    /**
     * @brief readRamGoalPosition       Read requested angluar position of servo (0x000-0x3ff)
     * @param servo_id                  Servo id.
     * @param goal_position             Goal position value as specified above.
     * @return                          BulletStatus type.
     */
    BulletStatus readRamGoalPosition(const uint8_t servo_id, uint16_t &goal_position);

    /**
     * @brief readRamMovingSpeed        Read angular velocity of the specified servo. Maximum value (1023) corresponds with
     *                                  114 RPM . When set to 0, the veocity if the largest possible for the supplied voltage.
     * @param servo_id                  Servo id.
     * @param moving_speed              Moving speed value as specified above.
     * @return                          BulletStatus type.
     */
    BulletStatus readRamMovingSpeed(const uint8_t servo_id, uint16_t &moving_speed);

    /**
     * @brief readRamTorqueLimit        Read the maximum output torque of specified servo. If it's set
     *                                  to 0 then servo is in free running mode (this value is copied
     *                                  from EEprom space when the power is applied to the servo.
     *                                  Possible values (0-1023)
     * @param servo_id                  Servo id.
     * @param torque_limit              Torque limit value as specified above.
     * @return
     */
    BulletStatus readRamTorqueLimit(const uint8_t servo_id, uint16_t &torque_limit);

    /**
     * @brief readRamPresentPosition    Read current position of servo (0-1023).
     * @param servo_id                  Servo id.
     * @param present_position          Present position value.
     * @return                          BulletStatus type.
     */
    BulletStatus readRamPresentPosition(const uint8_t servo_id, uint16_t &present_position);

    /**
     * @brief readRamPresentSpeed       Read current angular velocity of servo (0-1023)
     * @param servo_id                  Servo id.
     * @param present_speed             Present angular speed value.
     * @return                          BulletStatus type.
     */
    BulletStatus readRamPresentSpeed(const uint8_t servo_id, uint16_t &present_speed);

    /**
     * @brief readRamPresentLoad        Read current load of servo. For more information please read the dynamixel documantation.
     * @param servo_id                  Servo id.
     * @param present_load              Present load.
     * @return                          BulletStatus type.
     */
    BulletStatus readRamPresentLoad(const uint8_t servo_id, uint16_t &present_load);

    /**
     * @brief readRamPresentVoltage     Read current voltage of servo (value multiply by 10).
     * @param servo_id                  Servo id.
     * @param present_voltage           Present voltage value.
     * @return                          BulletStatus type.
     */
    BulletStatus readRamPresentVoltage(const uint8_t servo_id, uint8_t &present_voltage);

    /**
     * @brief readRamPresentTemperature     Read current temperature of servo in Celsjus degrees.
     * @param servo_id                      Servo id.
     * @param present_temperature           Present temperature value.
     * @return                              BulletStatus type.
     */
    BulletStatus readRamPresentTemperature(const uint8_t servo_id, uint8_t &present_temperature);

    /**
     * @brief readRamRegisteredInstruction  Read if any instruction is assigned by REG_WRITE command (read more
     *                                      in dynamixel documentation).
     * @param servo_id                      Servo id.
     * @param registered_instruction        Register instruction value.
     * @return                              BulletStatus type.
     */
    BulletStatus readRamRegisteredInstruction(const uint8_t servo_id, uint8_t &registered_instruction);

    /**
     * @brief readRamMoving     Read if servo is moving by it's own power.
     * @param servo_id          Servo id.
     * @param moving            Moving value (0-1).
     * @return                  BulletStatus type.
     */
    BulletStatus readRamMoving(const uint8_t servo_id, uint8_t &moving);

    /**
     * @brief readRamLock       Read if lock is set to 1 - if so, only methods from Torque Enable to Torque Limit in
     *                          RAM area is writtable.
     * @param servo_id          Servo id.
     * @param lock              Lock parameter value.
     * @return                  BulletStatus type.
     */
    BulletStatus readRamLock(const uint8_t servo_id, uint8_t &lock);

    /**
     * @brief readRamPunch      Read the minimum current supplied to the motor during operation. The initial value is
     *                          32 and the maximum value is 1023.
     * @param servo_id          Servo id.
     * @param punch             Punch parameter value.
     * @return                  BulletStatus type.
     */
    BulletStatus readRamPunch(const uint8_t servo_id, uint16_t &punch);

    //========================================================
    //====================== RAM WRITE =======================
    //========================================================

    /**
     * @brief writeRamTorqueEnable      When the power is first turned on, the servo enters the Torque Free Run mode.
     *                                  Setting this parameter to 1 enables the torque.
     * @param servo_id                  Servo id.
     * @param new_torque_enable         New torque enable value as specified above.
     * @return                          BulletStatus type.
     */
    BulletStatus writeRamTorqueEnable(const uint8_t servo_id, const uint8_t new_torque_enable);

    /**
     * @brief writeRamLED       Turn the servo's led on (1) or off (0).
     * @param servo_id          Servo id.
     * @param new_led           New led value as specified above.
     * @return                  BulletStatus type.
     */
    BulletStatus writeRamLED(const uint8_t servo_id, const uint8_t new_led);

    /**
     * @brief writeRamCWComplianceMargin    The compliance of the dynamixel actuator. Please read the documentation
     *                                      for more information.
     * @param servo_id                      Servo id.
     * @param new_cw_compliance_margin      New lower compliance margin.
     * @return                              BulletStatus type.
     */
    BulletStatus writeRamCWComplianceMargin(const uint8_t servo_id, const uint8_t new_cw_compliance_margin);

    /**
     * @brief writeRamCCWComplianceMargin   The compliance of the dynamixel actuator. Please read the documentation
     *                                      for more information.
     * @param servo_id                      Servo id.
     * @param new_ccw_compliance_margin     New upper compliance margin.
     * @return                              BulletStatus type.
     */
    BulletStatus writeRamCCWComplianceMargin(const uint8_t servo_id, const uint8_t new_ccw_compliance_margin);

    /**
     * @brief writeRamCWComplianceSlope     The compliance of the dynamixel actuator. Please read the documentation
     *                                      for more information.
     * @param servo_id                      Servo id.
     * @param new_cw_compliance_slope       New lower compliance slope.
     * @return                              BulletStatus type.
     */
    BulletStatus writeRamCWComplianceSlope(const uint8_t servo_id, const uint8_t new_cw_compliance_slope);

    /**
     * @brief writeRamCCWComplianceSlope    The compliance of the dynamixel actuator. Please read the documentation
     *                                      for more information.
     * @param servo_id                      Servo id
     * @param new_ccw_compliance_slope      New upper compliance slope.
     * @return                              BulletStatus type.
     */
    BulletStatus writeRamCCWComplianceSlope(const uint8_t servo_id, const uint8_t new_ccw_compliance_slope);

    /**
     * @brief readRamGoalPosition       Read requested angluar position of servo (0x000-0x3ff)
     * @param servo_id                  Servo id.
     * @param goal_position             Goal position value as specified above.
     * @return                          BulletStatus type.
     */

    /**
     * @brief writeRamGoalPosition      Write angluar position of servo (0x000-0x3ff)
     * @param servo_id                  Servo id.
     * @param new_goal_position         New goal position value as specified above.
     * @return                          BulletStatus type.
     */
    BulletStatus writeRamGoalPosition(const uint8_t servo_id, const uint16_t new_goal_position);

    /**
     * @brief writeRamMovingSpeed       Write angular velocity of the specified servo. Maximum value (1023) corresponds with
     *                                  114 RPM . When set to 0, the veocity if the largest possible for the supplied voltage.
     * @param servo_id                  Servo id.
     * @param new_moving_speed          Moving speed value as specified above.
     * @return                          BulletStatus type.
     */
    BulletStatus writeRamMovingSpeed(const uint8_t servo_id, const uint16_t new_moving_speed);

    /**
     * @brief writeRamTorqueLimit       Write the maximum output torque of specified servo. If it's set
     *                                  to 0 then servo is in free running mode (this value is copied
     *                                  from EEprom space when the power is applied to the servo.
     *                                  Possible values (0-1023)
     * @param servo_id                  Servo id.
     * @param new_torque_limit          new torque limit parameter.
     * @return                          BulletStatus type.
     */
    BulletStatus writeRamTorqueLimit(const uint8_t servo_id, const uint16_t new_torque_limit);

    /**
     * @brief writeRamRegisteredInstruction     Write if any instruction is assigned by REG_WRITE command (read more
     *                                          in dynamixel documentation). Don't know if it has any sense, but dynamixel
     *                                          gives opprtunity to write it.
     * @param servo_id                          Servo id.
     * @param new_registered_instruction        New register instruction value.
     * @return                                  BulletStatus type.
     */
    BulletStatus writeRamRegisteredInstruction(const uint8_t servo_id, const uint8_t new_registered_instruction);

    /**
     * @brief writeRamLock      Write lock value. If it is set to 1 only methods from Torque Enable to Torque Limit in
     *                          RAM area is writtable.
     * @param servo_id          Servo id.
     * @param new_lock          New lock parameter value.
     * @return                  BulletStatus type.
     */
    BulletStatus writeRamLock(const uint8_t servo_id, const uint8_t new_lock);

    /**
     * @brief writeRamPunch     Write the minimum current supplied to the motor during operation. The initial value is
     *                          32 and the maximum value is 1023.
     * @param servo_id          Servo id.
     * @param new_punch         New punch parameter value.
     * @return                  BulletStatus type.
     */
    BulletStatus writeRamPunch(const uint8_t servo_id, const uint16_t new_punch);

    //========================================================
    //====================== ALL READ ========================
    //========================================================

    /**
     * @brief readAllPositionSpeedTorque        Read position, speed and torque of all servos.
     * @param position                          Positions vector.
     * @param speed                             Speed vector.
     * @param torque                            Torque vector.
     * @return                                  BulletStatus type.
     */
    BulletStatus readAllPositionSpeedTorque(uint16_t *position, uint16_t *speed, uint16_t *torque);

    /**
     * @brief writeAllPosition                  Write positions of all servos.
     * @param position                          Positions vector.
     * @return                                  BulletStatus type.
     */
    BulletStatus writeAllPosition(uint16_t *position);
};


#endif // BULLET_ROBOT_H
