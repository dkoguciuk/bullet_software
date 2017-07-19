#ifndef BULLET_SPI_H
#define BULLET_SPI_H

#include "bullet_datatypes.h"

#include <iostream>                 // std::cout
#include <string>                   // std::string
#include <vector>                   // std::vector
#include <cstdio>                   // perror
#include <cstdlib>                  // exit
#include <stdint.h>                 // ints with 8 & 16 bits
#include <unistd.h>                 // close
#include <fcntl.h>                  // open
#include <linux/spi/spidev.h>       // spi_ioc_transfer
#include <sys/ioctl.h>              // ioctl
#include <errno.h>                  // errors

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"                   /* Reset */
#define BLACK   "\033[30m"                  /* Black */
#define RED     "\033[31m"                  /* Red */
#define GREEN   "\033[32m"                  /* Green */
#define YELLOW  "\033[33m"                  /* Yellow */
#define BLUE    "\033[34m"                  /* Blue */
#define MAGENTA "\033[35m"                  /* Magenta */
#define CYAN    "\033[36m"                  /* Cyan */
#define WHITE   "\033[37m"                  /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

using namespace std;

class BulletRobot;

class BulletSPI
{

friend class BulletRobot;

private:

    /**
     * @brief spi_baud                  Baudrate of spi communication.
     */
    int spi_baud;

    /**
     * @brief spi_file_name             File name of spi device.
     */
    string spi_file_name;

    /**
     * @brief spi_file_desc             File descriptor of the spi device.
     */
    int spi_file_desc;

    /**
     * @brief spi_transfer              SPI transfer via ioctl mechanism.
     */
    struct spi_ioc_transfer spi_transfer[2] = { {}, {} };

protected:

    /**
     * @brief message_out               Main output data container.
     */
    vector<uint8_t> message_out;

    /**
     * @brief message_in                Main input data container.
     */
    vector<uint8_t> message_in;

public:

    /**
     * @brief BulletSPI                   Default constructor.
     * @param spi_dev_desc
     */
    BulletSPI(string _spi_file_name, int _spi_baud);

    /**
     *  @brief ~BulletSPI                 Default destructor.
     */
    ~BulletSPI();

    /**
     * @brief print_message_in          Debug method for displaying the contents.
     */
    void print_message_in();

    /**
     * @brief print_message_out         Debug method for displaying the contents.
     */
    void print_message_out();

    /**
     * @brief operator <<               Overloaded operator.
     * @param data                      Data to add to the message.
     * @return                          Class object.
     */
    BulletSPI& operator<<(uint8_t data);

    /**
     * @brief operator <<               Overloaded operator.
     * @param data                      Data to add to the message.
     * @return                          Class object.
     */
    BulletSPI& operator<<(uint16_t data);

    /**
     * @brief calc_msg_out              Calculate output CRC value.
     */
    void calc_msg_out();

    /**
     * @brief calc_crc_in               Check if crc of the input massage is valid.
     * @return                          True if crc of the input massage is valid.
     */
    bool calc_msg_in();


    /**
     * @brief send                      Send message via SPI.
     * @param spi_dev_desc              SPI device descriptor.
     */
    BulletStatus transfer(int return_bytes);

    /**
     * @brief clear                     Method for clear input and output buffors.
     */
    void clear();
};

#endif //BULLET_SPI_H
