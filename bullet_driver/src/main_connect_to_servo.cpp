#include <iostream>
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
    std::cout << BOLDBLUE << "Welcome to connect to servo node! Please specify servo id to connect with:\t" << RESET;

    int help;
    std::cin  >> help;
    uint8_t servo_number = help;

    BulletRobot bullet_robot;
    bullet_robot.changeBulletSTMMode(BULLET_MODE_PREOPERATIONAL);

    if (bullet_robot.pingServo(servo_number) == BULLET_SUCCESS)
        std::cout << BOLDRED << "CONNECTED!" << std::endl;
    else
    {
        std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    int param=1;
    while (param)
    {
        std::cout << BOLDRED << "1.  " << BOLDBLUE <<     "Servo ID                     " << BOLDRED << (int)servo_number << RESET << std::endl;

        uint8_t torque_enable;
        if (bullet_robot.readRamTorqueEnable(servo_number, torque_enable) == BULLET_SUCCESS)
            std::cout << BOLDRED << "2.  " << BOLDBLUE << "Torque enable                " << BOLDRED << (int)torque_enable << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t led;
        if (bullet_robot.readRamLED(servo_number, led) == BULLET_SUCCESS)
            std::cout << BOLDRED << "3.  " << BOLDBLUE << "LED                          " << BOLDRED << (int)led << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t cw_compliance_margin;
        if (bullet_robot.readRamCWComplianceMargin(servo_number, cw_compliance_margin) == BULLET_SUCCESS)
            std::cout << BOLDRED << "4.  " << BOLDBLUE << "Lower compliance margin      " << BOLDRED << (int)cw_compliance_margin << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t ccw_compliance_margin;
        if (bullet_robot.readRamCCWComplianceMargin(servo_number, ccw_compliance_margin) == BULLET_SUCCESS)
            std::cout << BOLDRED << "5.  " << BOLDBLUE << "Upper compliance margin      " << BOLDRED << (int)ccw_compliance_margin << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t cw_compliance_slope;
        if (bullet_robot.readRamCWComplianceSlope(servo_number, cw_compliance_slope) == BULLET_SUCCESS)
            std::cout << BOLDRED << "6.  " << BOLDBLUE << "Lower compliance slope       " << BOLDRED << (int)cw_compliance_slope << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t ccw_compliance_slope;
        if (bullet_robot.readRamCCWComplianceSlope(servo_number, ccw_compliance_slope) == BULLET_SUCCESS)
            std::cout << BOLDRED << "7.  " << BOLDBLUE << "Upper compliance slope       " << BOLDRED << (int)ccw_compliance_slope << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint16_t goal_position;
        if (bullet_robot.readRamGoalPosition(servo_number, goal_position) == BULLET_SUCCESS)
            std::cout << BOLDRED << "8.  " << BOLDBLUE << "Goal position                " << BOLDRED << (int)goal_position << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint16_t moving_speed;
        if (bullet_robot.readRamMovingSpeed(servo_number, moving_speed) == BULLET_SUCCESS)
            std::cout << BOLDRED << "9.  " << BOLDBLUE << "Moving speed                 " << BOLDRED << (int)moving_speed << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint16_t torque_limit;
        if (bullet_robot.readRamTorqueLimit(servo_number, torque_limit) == BULLET_SUCCESS)
            std::cout << BOLDRED << "10. " << BOLDBLUE << "Torque limit                 " << BOLDRED << (int)torque_limit << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint16_t present_position;
        if (bullet_robot.readRamPresentPosition(servo_number, present_position) == BULLET_SUCCESS)
            std::cout << BOLDRED << "11. " << BOLDBLUE << "Present position             " << BOLDRED << (int)present_position << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint16_t present_speed;
        if (bullet_robot.readRamPresentSpeed(servo_number, present_speed) == BULLET_SUCCESS)
            std::cout << BOLDRED << "12. " << BOLDBLUE << "Present speed                " << BOLDRED << (int)present_speed << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint16_t present_load;
        if (bullet_robot.readRamPresentLoad(servo_number, present_load) == BULLET_SUCCESS)
            std::cout << BOLDRED << "13. " << BOLDBLUE << "Present load                 " << BOLDRED << (int)present_load << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t present_voltage;
        if (bullet_robot.readRamPresentVoltage(servo_number, present_voltage) == BULLET_SUCCESS)
            std::cout << BOLDRED << "14. " << BOLDBLUE << "Present voltage              " << BOLDRED << (int)present_voltage << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t present_temperature;
        if (bullet_robot.readRamPresentTemperature(servo_number, present_temperature) == BULLET_SUCCESS)
            std::cout << BOLDRED << "15. " << BOLDBLUE << "Present temperature          " << BOLDRED << (int)present_temperature << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t registered_instruction;
        if (bullet_robot.readRamRegisteredInstruction(servo_number, registered_instruction) == BULLET_SUCCESS)
            std::cout << BOLDRED << "16. " << BOLDBLUE << "Registered instruction       " << BOLDRED << (int)registered_instruction << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t moving;
        if (bullet_robot.readRamMoving(servo_number, moving) == BULLET_SUCCESS)
            std::cout << BOLDRED << "17. " << BOLDBLUE << "Moving                       " << BOLDRED << (int)moving << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint8_t lock;
        if (bullet_robot.readRamLock(servo_number, lock) == BULLET_SUCCESS)
            std::cout << BOLDRED << "18. " << BOLDBLUE << "Lock                         " << BOLDRED << (int)lock << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        uint16_t punch;
        if (bullet_robot.readRamPunch(servo_number, punch) == BULLET_SUCCESS)
            std::cout << BOLDRED << "19. " << BOLDBLUE << "Punch                        " << BOLDRED << (int)punch << RESET << std::endl;
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
                std::cout << BOLDRED << "connect_to_servo do not let the user to change servo ID or servo baudrate. Please run configure_servo_id_br" << RESET << std::endl;
                break;
            }
            case 2:
            {
                std::cout << BOLDRED << "When the power is first turned on, the servo enters the Free Run mode. Setting this parameter to 1 enables the torque. ";
                std::cout << "Enter new parameter (0-1) " << RESET;

                int value;
                std::cin  >> value;
                torque_enable = value;
                if (bullet_robot.writeRamTorqueEnable(servo_number, torque_enable) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 3:
            {
                std::cout << BOLDRED << "Turn the servo's led on (1) or off (0). ";
                std::cout << "Enter new parameter (0-1) " << RESET;

                int value;
                std::cin  >> value;
                led = value;
                if (bullet_robot.writeRamLED(servo_number, led) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 4:
            {
                std::cout << BOLDRED << "The compliance of the stm_dynamixel actuator. Please read the documentation for more information. ";
                std::cout << "Enter new parameter (0-255) " << RESET;

                int value;
                std::cin  >> value;
                cw_compliance_margin = value;
                if (bullet_robot.writeRamCWComplianceMargin(servo_number, cw_compliance_margin) == BULLET_SUCCESS)
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
                std::cout << BOLDRED << "The compliance of the stm_dynamixel actuator. Please read the documentation for more information. ";
                std::cout << "Enter new parameter (0-255) " << RESET;

                int value;
                std::cin  >> value;
                ccw_compliance_margin = value;
                if (bullet_robot.writeRamCCWComplianceMargin(servo_number, ccw_compliance_margin) == BULLET_SUCCESS)
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
                std::cout << BOLDRED << "The compliance of the stm_dynamixel actuator. Please read the documentation for more information. ";
                std::cout << "Enter new parameter (0-255) " << RESET;

                int value;
                std::cin  >> value;
                cw_compliance_slope = value;
                if (bullet_robot.writeRamCWComplianceSlope(servo_number, cw_compliance_slope) == BULLET_SUCCESS)
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
                std::cout << BOLDRED << "The compliance of the stm_dynamixel actuator. Please read the documentation for more information. ";
                std::cout << "Enter new parameter (0-255) " << RESET;

                int value;
                std::cin  >> value;
                ccw_compliance_slope = value;
                if (bullet_robot.writeRamCCWComplianceSlope(servo_number, ccw_compliance_slope) == BULLET_SUCCESS)
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
                std::cout << BOLDRED << "Write angluar position of servo. ";
                std::cout << "Enter new parameter (0-1023) " << RESET;
                int value;
                std::cin  >> value;
                goal_position = value;
                if (bullet_robot.writeRamGoalPosition(servo_number, goal_position) == BULLET_SUCCESS)
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
                std::cout << BOLDRED << "This parameter tells about angular velocity of the specified servo. Maximum value (1023) corresponds with ";
                std::cout << "114 RPM . When set to 0, the veocity if the largest possible for the supplied voltage. ";
                std::cout << "Enter new parameter (0-1023) " << RESET;

                int value;
                std::cin  >> value;
                moving_speed = value;
                if (bullet_robot.writeRamMovingSpeed(servo_number, moving_speed) == BULLET_SUCCESS)
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
                std::cout << BOLDRED << "This parameter is setting the maximum output torque of specified servo. If it's set to 0 then servo is ";
                std::cout << "in free running mode (this value is copied from EEprom space when the power is applied to the servo. ";
                std::cout << "Enter new parameter (0-1023) " << RESET;

                int value;
                std::cin  >> value;
                torque_limit = value;
                if (bullet_robot.writeRamTorqueLimit(servo_number, torque_limit) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 11:
            case 12:
            case 13:
            case 14:
            case 15:
            case 17:
            {
                std::cout << BOLDRED << "Servo do not allow to change this parameter. Sorry!" << RESET << std::endl;
                break;
            }
            case 16:
            {
                std::cout << BOLDRED << "Write if any instruction is assigned by REG_WRITE command (read more ";
                std::cout << "in stm_dynamixel documentation). Don't know if it has any sense, but stm_dynamixel ";
                std::cout << "gives opprtunity to write it. Enter new parameter (0-1) " << RESET;

                int value;
                std::cin  >> value;
                registered_instruction = value;
                if (bullet_robot.writeRamRegisteredInstruction(servo_number, registered_instruction) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 18:
            {
                std::cout << BOLDRED << "Write lock value. If it is set to 1 only methods from Torque Enable to Torque Limit in RAM area is writtable. ";
                std::cout << "Enter new parameter (0-1) " << RESET;

                int value;
                std::cin  >> value;
                lock = value;
                if (bullet_robot.writeRamLock(servo_number, lock) == BULLET_SUCCESS)
                    std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
                else
                {
                    std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                break;
            }
            case 19:
            {
                std::cout << BOLDRED << "Write the minimum current supplied to the motor during operation. The initial value is 32 and the maximum value is 1023. ";
                std::cout << "Enter new parameter (0-1023) " << RESET;

                int value;
                std::cin  >> value;
                punch = value;
                if (bullet_robot.writeRamPunch(servo_number, punch) == BULLET_SUCCESS)
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
