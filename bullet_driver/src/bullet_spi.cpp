#include "bullet_spi.h"

BulletSPI::BulletSPI(string _spi_file_name, int _spi_baud)
{
    // Copy file name
    spi_file_name = _spi_file_name;
    spi_baud = _spi_baud;

    // Config
    __u8  mode, lsb, bits;
    __u32 speed=spi_baud;

    // Open file
    if ((spi_file_desc = open(spi_file_name.c_str(),O_RDWR)) < 0)
    {
        std::cout << BOLDRED << "BulletSPI: " << BOLDBLUE << "Cannot open the SPI bus!" << RESET << std::endl;
        exit(-1);
    }

//        // CONFIG
//        // possible modes: mode |= SPI_LOOP; mode |= SPI_CPHA; mode |= SPI_CPOL; mode |= SPI_LSB_FIRST; mode |= SPI_CS_HIGH; mode |= SPI_3WIRE; mode |= SPI_NO_CS; mode |= SPI_READY;
//        // multiple possibilities using |
//        if (ioctl(spi_file_desc, SPI_IOC_WR_MODE, &mode)<0)
//        {
//            perror("can't set spi mode");
//            exit(-1);
//        }
//        if (ioctl(spi_file_desc, SPI_IOC_RD_MODE, &mode) < 0)
//        {
//            perror("SPI rd_mode");
//            exit(-1);
//        }
//        if (ioctl(spi_file_desc, SPI_IOC_RD_LSB_FIRST, &lsb) < 0)
//        {
//            perror("SPI rd_lsb_fist");
//            exit(-1);
//        }
//        if (ioctl(spi_file_desc, SPI_IOC_WR_BITS_PER_WORD, 8)<0)
//        {
//            perror("can't set bits per word");
//            exit(-1);
//        }
//        if (ioctl(spi_file_desc, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0)
//        {
//            perror("SPI bits_per_word");
//            exit(-1);
//        }
//        if (ioctl(file, SPI_IOC_WR_MAX_SPEED_HZ, &speed)<0)
//        {
//            perror("can't set max speed hz");
//            exit(-1);
//        }
//        if (ioctl(spi_file_desc, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
//        {
//            perror("SPI max_speed_hz");
//            exit(-1);
//        }
//        printf("%s: spi mode %d, %d bits %sper word, %d Hz max\n",spi_file_name.c_str(), mode, bits, lsb ? "(lsb first) " : "", speed);

    // SPI STRUCT CONFIG
    spi_transfer[0].delay_usecs = 10000;        // delay in us
    spi_transfer[0].speed_hz = spi_baud;    // speed in Hz
    spi_transfer[0].bits_per_word = 8;     // bites per word 8

    spi_transfer[1].delay_usecs = 10000;
    spi_transfer[1].speed_hz = spi_baud;
    spi_transfer[1].bits_per_word = 8;

    // INFO
    std::cout << BOLDRED << "BulletSPI: " << BOLDGREEN << "Connected to the BulletSPI!" << RESET << std::endl;
}

BulletSPI::~BulletSPI()
{
    close(spi_file_desc);
}

void BulletSPI::print_message_in()
{
    std::cout << BOLDRED << "BulletSPI: " << BOLDBLUE << "message in:" << RESET << std::endl;
    for (int i=0; i<message_in.size(); ++i) cout << (int)message_in.at(i) << endl;
}

void BulletSPI::print_message_out()
{
    std::cout << BOLDRED << "BulletSPI: " << BOLDBLUE << "message out:" << RESET << std::endl;
    for (int i=0; i<message_out.size(); ++i) cout << (int)message_out.at(i) << endl;
}

BulletSPI& BulletSPI::operator<<(uint8_t data)
{
    message_out.push_back(data);
    return *this;
}

BulletSPI& BulletSPI::operator<<(uint16_t data)
{
    message_out.push_back((uint8_t)data);
    message_out.push_back((uint8_t)(data>>8));
    return *this;
}

void BulletSPI::calc_msg_out()
{
    uint8_t sum=0;
    for(int i=0; i<message_out.size(); ++i) sum+=message_out.at(i);
    message_out.push_back(255-sum);
    message_out.insert(message_out.begin(), message_out.size());
}

bool BulletSPI::calc_msg_in()
{
    uint8_t sum=0;
    for(int i=0; i<message_in.size()-1; ++i) sum+=message_in.at(i);

    if (message_in.back() != (255-sum)) return false;
    return true;
}

BulletStatus BulletSPI::transfer(int return_bytes)
{
    // Set data
    spi_transfer[0].tx_buf = (unsigned long) message_out.data();
    spi_transfer[0].len = message_out.size();

    message_in.resize(return_bytes);
    spi_transfer[1].rx_buf = (unsigned long) message_in.data();
    spi_transfer[1].len = message_in.size();

    // Transfer
    int status = ioctl(spi_file_desc, SPI_IOC_MESSAGE(2), spi_transfer);
    if (status < 0)
    {
        std::cout << BOLDRED << "BulletSPI: " << BOLDBLUE << "ioctl error!" << RESET << std::endl;
        return BULLET_ERROR_IOCTL;
    }

    // Calc CRC
    if (!calc_msg_in())
    {
        std::cout << BOLDRED << "BulletSPI: " << BOLDBLUE << "crc of the input massage is invalid!" << RESET << std::endl;
        return BULLET_ERROR_SPI_RCR_IN;
    }

    // Return
    return (BulletStatus)message_in.front();
}

void BulletSPI::clear()
{
    message_in.clear();
    message_out.clear();
}
