#include <stdlib.h>
#include "bullet_robot.h"

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */


int main(int argc,char** argv)
{
    BulletRobot bullet_robot;
    bullet_robot.changeBulletSTMMode(BULLET_MODE_PREOPERATIONAL);

    std::cout << BOLDBLUE << "Welcome to configure servo node! Please specify servo id to configure:\t" << RESET;

    int help;
    std::cin  >> help;
    uint8_t servo_number = help;

    int param=1;
    while (param)
    {
        std::cout << BOLDRED << "1.  " << BOLDBLUE <<     "Servo ID                     " << BOLDRED << (int)servo_number << RESET << std::endl;

        uint16_t model_number;
        if (bullet_robot.readEEpromModelNumber(servo_number, model_number) == BULLET_SUCCESS)
            std::cout << BOLDRED << "2.  " << BOLDBLUE << "Model number                 " << BOLDRED << (int)model_number << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t firmware_version;
        if (bullet_robot.readEEpromFirmwareVersion(servo_number, firmware_version) == BULLET_SUCCESS)
            std::cout << BOLDRED << "3.  " << BOLDBLUE << "Firmware version             " << BOLDRED << (int)firmware_version << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t return_delay_time;
        if (bullet_robot.readEEpromReturnDelayTime(servo_number, return_delay_time) == BULLET_SUCCESS)
            std::cout << BOLDRED << "4.  " << BOLDBLUE << "Return Delay Time            " << BOLDRED << (int)return_delay_time << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint16_t cw_angle_limit;
        if (bullet_robot.readEEpromCWAngleLimit(servo_number, cw_angle_limit) == BULLET_SUCCESS)
            std::cout << BOLDRED << "5.  " << BOLDBLUE << "CW Angle Limit               " << BOLDRED << (int)cw_angle_limit << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint16_t ccw_angle_limit;
        if (bullet_robot.readEEpromCCWAngleLimit(servo_number, ccw_angle_limit) == BULLET_SUCCESS)
            std::cout << BOLDRED << "6.  " << BOLDBLUE << "CCW Angle Limit              " << BOLDRED << (int)ccw_angle_limit << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t highest_limit_temperature;
        if (bullet_robot.readEEpromHighestLimitTemperature(servo_number, highest_limit_temperature) == BULLET_SUCCESS)
            std::cout << BOLDRED << "7.  " << BOLDBLUE << "Highest limit temperature    " << BOLDRED << (int)highest_limit_temperature << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t lowest_limit_voltage;
        if (bullet_robot.readEEpromLowestLimitVoltage(servo_number, lowest_limit_voltage) == BULLET_SUCCESS)
            std::cout << BOLDRED << "8.  " << BOLDBLUE << "Lowest limit voltage         " << BOLDRED << (int)lowest_limit_voltage << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t highest_limit_voltage;
        if (bullet_robot.readEEpromHighestLimitVoltage(servo_number, highest_limit_voltage) == BULLET_SUCCESS)
            std::cout << BOLDRED << "9.  " << BOLDBLUE << "Highest limit voltage        " << BOLDRED << (int)highest_limit_voltage << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint16_t max_torque;
        if (bullet_robot.readEEpromMaxTorque(servo_number, max_torque) == BULLET_SUCCESS)
            std::cout << BOLDRED << "10. " << BOLDBLUE << "Max toruqe                   " << BOLDRED << (int)max_torque << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t status_return_level;
        if (bullet_robot.readEEpromStatusReturnLevel(servo_number, status_return_level) == BULLET_SUCCESS)
            std::cout << BOLDRED << "11. " << BOLDBLUE << "Status return level          " << BOLDRED << (int)status_return_level << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t alarm_led;
        if (bullet_robot.readEEpromAlarmLED(servo_number, alarm_led) == BULLET_SUCCESS)
            std::cout << BOLDRED << "12. " << BOLDBLUE << "Alarm LED                    " << BOLDRED << (int)alarm_led << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t alarm_shutdown;
        if (bullet_robot.readEEpromAlarmShutdown(servo_number, alarm_shutdown) == BULLET_SUCCESS)
            std::cout << BOLDRED << "13. " << BOLDBLUE << "Alarm shutdown               " << BOLDRED << (int)alarm_shutdown << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::cout << BOLDBLUE << "What parameter do you want to change? 0 ends program. " << RESET;

        std::cin  >> param;
        switch (param)
        {
            case 1:
            {
                std::cout << BOLDRED << "configure_servo do not let the user to change servo ID or servo baudrate. Please run configure_servo_id_br" << RESET << std::endl;
                break;
            }
            case 2:
            case 3:
            {
                std::cout << BOLDRED << "Servo do not allow to change this parameter. Sorry!" << RESET << std::endl;
                break;
            }
            case 4:
            {
                std::cout << BOLDRED << "It is the time it takes for the Status Packet to return from servo to stm_dynamixel board. The delay time is given";
                std::cout << "by 2uSec * this parameter. Enter new parameter (0-255) " << RESET;

                int value;
                std::cin  >> value;
                return_delay_time = value;
                if (bullet_robot.writeEEpromReturnDelayTime(servo_number, return_delay_time) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 5:
            {
                std::cout << BOLDRED << "This parameter limits servo lower position. Enter new parameter (0-1023) " << RESET;

                int value;
                std::cin  >> value;
                cw_angle_limit = value;
                if (bullet_robot.writeEEpromCWAngleLimit(servo_number, cw_angle_limit) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 6:
            {
                std::cout << BOLDRED << "This parameter limits servo upper position. Enter new parameter (0-1023) " << RESET;

                int value;
                std::cin  >> value;
                ccw_angle_limit = value;
                if (bullet_robot.writeEEpromCCWAngleLimit(servo_number, ccw_angle_limit) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 7:
            {
                std::cout << BOLDRED << "This parameter limits the upper operating temperature (Celsius degrees) of the servo. Enter new parameter (0-255) " << RESET;

                int value;
                std::cin  >> value;
                highest_limit_temperature = value;
                if (bullet_robot.writeEEpromHighestLimitTemperature(servo_number, highest_limit_temperature) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 8:
            {
                std::cout << BOLDRED << "This parameter limits the lower operating voltage of the servo. The value is 10 times the actual voltage. Enter new parameter (0-255) " << RESET;

                int value;
                std::cin  >> value;
                lowest_limit_voltage = value;
                if (bullet_robot.writeEEpromLowestLimitVoltage(servo_number, lowest_limit_voltage) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 9:
            {
                std::cout << BOLDRED << "This parameter limits the upper operating voltage of the servo. The value is 10 times the actual voltage. Enter new parameter (0-255) " << RESET;

                int value;
                std::cin  >> value;
                highest_limit_voltage = value;
                if (bullet_robot.writeEEpromHighestLimitVoltage(servo_number, highest_limit_voltage) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 10:
            {
                std::cout << BOLDRED << "This parameter limits max torque of the servo. If the value is set to 0 servo enters free running mode. Enter new parameter (0-1023) " << RESET;

                int value;
                std::cin  >> value;
                max_torque = value;
                if (bullet_robot.writeEEpromMaxTorque(servo_number, max_torque) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 11:
            {
                std::cout << BOLDRED << "This parameter determines whether the servo will return status packet to the stm_dynamixel board. Possible values:" << std::endl;
                std::cout << "0 - do not respond to any instructions" << std::endl;
                std::cout << "1 - respond only to READ_DATA instructions" << std::endl;
                std::cout << "0 - respond to all instructions" << std::endl;
                std::cout << "Enter new parameter (0-2) " << RESET;

                int value;
                std::cin  >> value;
                status_return_level = value;
                if (bullet_robot.writeEEpromStatusReturnLevel(servo_number, status_return_level) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 12:
            {
                std::cout << BOLDRED << "This parameter determines whether the led diode will blink if corresponding bit is set:" << std::endl;
                std::cout << "0 - input voltage error" << std::endl;
                std::cout << "1 - angle limit error" << std::endl;
                std::cout << "2 - overheating error" << std::endl;
                std::cout << "3 - range error" << std::endl;
                std::cout << "4 - checksum error" << std::endl;
                std::cout << "5 - overload error" << std::endl;
                std::cout << "6 - instruction error" << std::endl;
                std::cout << "7 - always 0" << std::endl;
                std::cout << "Enter new parameter (0-127) " << RESET;

                int value;
                std::cin  >> value;
                alarm_led = value;
                if (bullet_robot.writeEEpromAlarmLED(servo_number, alarm_led) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 13:
            {
                std::cout << BOLDRED << "If corresponding bit is set to 1 servo's torque will be disabled if an error occurs. Possible bit values:" << std::endl;
                std::cout << "0 - input voltage error" << std::endl;
                std::cout << "1 - angle limit error" << std::endl;
                std::cout << "2 - overheating error" << std::endl;
                std::cout << "3 - range error" << std::endl;
                std::cout << "4 - checksum error" << std::endl;
                std::cout << "5 - overload error" << std::endl;
                std::cout << "6 - instruction error" << std::endl;
                std::cout << "7 - always 0" << std::endl;
                std::cout << "Enter new parameter (0-127) " << RESET;

                int value;
                std::cin  >> value;
                alarm_shutdown = value;
                if (bullet_robot.writeEEpromAlarmShutdown(servo_number, alarm_shutdown) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
        }
    }

    return 0;
}

