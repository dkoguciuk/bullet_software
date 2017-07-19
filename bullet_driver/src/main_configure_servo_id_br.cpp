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
    std::cout << BOLDBLUE << "Welcome to configure servo id & broadcast node! Are you sure you want to send a message in broadcast mode - ";
    std::cout << "there should be only one servo connected to the STM board! (1/0)\t" << RESET;

    int sure;
    std::cin  >> sure;
    if (!sure) exit(-1);

    BulletRobot bullet_robot;
    bullet_robot.changeBulletSTMMode(BULLET_MODE_PREOPERATIONAL);

//    if (result == Bullet::STMDynamixel::success)
//        std::cout << BOLDRED << "CONNECTED!" << std::endl;
//    else
//    {
//        std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
//        std::exit(EXIT_FAILURE);
//    }

    std::cout << BOLDBLUE << "What do you want to change id (1) or broadcast (2) \t" << RESET;

    int what;
    std::cin  >> what;

    if (what==1)
    {
        std::cout << BOLDBLUE << "Enter new servo id (0-253)\t" << RESET;

        int help_id;
        std::cin  >> help_id;
        uint8_t id = help_id;

        if (bullet_robot.writeEEpromID(id) == BULLET_SUCCESS)
            std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }
    } else if (what==2)
    {
        std::cout << BOLDBLUE << "Enter new baudrate. The values are calculated with the following formula:\n";
        std::cout << "Speed(BPS) = 2000000 / (value + 1) \t" << RESET;

        int help_br;
        std::cin  >> help_br;
        uint8_t br = help_br;

        if (bullet_robot.writeEEpromBaudrate(br) != BULLET_SUCCESS)
            std::cout << BOLDRED << "SUCCESS!" << RESET << std::endl;
        else
        {
            std::cout << BOLDRED << "COMMUNICATION ERROR!" << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    return 0;
}
