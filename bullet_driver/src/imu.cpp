#include "imu.h"
#include "Fifo.c"

IMU::IMU(QObject *parent) : QObject(parent)
{
    //Ustawiamy interwal dla timera odbioru
    dataTimeInterval=50;

    //Tworzymy timer trybu chopped
    choppedTimer=new QTimer(this);

    //Ustawiamy interwal timera trybu chopped
    choppedTimer->setInterval(dataTimeInterval);

    //Laczymy sygnal przepelnienia ze slotem obslugi
    QObject::connect(choppedTimer,SIGNAL(timeout()),this,SLOT(choppedTimerTimeout()));

    //Tworzymy timer
    receiverTimer=new QTimer(this);

    //Czas oczekiwania na odpowiedz IMU
    receiverTimer->setInterval(50);

    //Ustawiamy jako pojedynczy strzal
    receiverTimer->setSingleShot(true);

    //Laczymy sygnal przepelnienia ze slotem obslugi
    QObject::connect(receiverTimer,SIGNAL(timeout()),this,SLOT(receiverTimerTimeout()));

}
IMU::~IMU()
{
    //Zamykamy port
    close();

    //Usuwamy timery
    delete choppedTimer;
    delete receiverTimer;
}

bool IMU::open()
{
    //Otwieramy port
    descriptor=serialOpen("/dev/ttyAMA0",57600);
    continous = 0;
    //Jezeli blad
    if (descriptor==-1)
    {
        //Zamykamy port
        serialClose(descriptor);

        //Zerujemy zmienna urzadzenia
        descriptor=0;

        //Zwracamy niepowodzenie
        return false;
    }

    //Ustawiamy dane binarne
    serialPutchar(descriptor,'#');
    serialPutchar(descriptor,'o');
    serialPutchar(descriptor,'b');

    //Zwracamy sukces
    return true;
}
bool IMU::close()
{
    //Jezeli port jest otwarty to zamknij
    if (descriptor>0) serialClose(descriptor);

    //Zwroc sukces
    return true;
}

bool IMU::setDataTimeInterval(int _interval)
{
    if (_interval>50)
    {
        dataTimeInterval=_interval;
        choppedTimer->setInterval(_interval);
        return true;
    }
    return false;
}
int IMU::getDataTimeInterval()
{
    return dataTimeInterval;
}

float IMU::getPitchAngle()
{
    return eulerAngles[1];
}
float IMU::getRollAngle()
{
    return eulerAngles[2];
}
float IMU::getYawAngle()
{
    return eulerAngles[0];
}

bool IMU::continousMeasurement(bool flag)
{
    //Sprawdzamy, czy port jest otwarty
    if (descriptor>0)
    {
        if(flag)
        {
            //Wysylamy komende wlaczenia pomiaru ciaglego
            serialPutchar(descriptor,'#');
            serialPutchar(descriptor,'o');
            serialPutchar(descriptor,'1');

            //Startujemy timer odbioru zmieniajac jego interwal
            receiverTimer->setInterval(20);
            receiverTimer->start();

            //Wlaczamy flage pomiaru ciaglego
            continous = true;

        }
        else
        {
            //Wysylamy komende wylaczenia pomiaru ciaglego
            serialPutchar(descriptor,'#');
            serialPutchar(descriptor,'o');
            serialPutchar(descriptor,'0');

            //Wylaczamy flage pomiaru ciaglego
            continous = false;

            //Wylaczamy timer
            receiverTimer->stop();

            //Wracamy do poprzedniego interwalu
            receiverTimer->setInterval(50);
        }
        return true;
    }
    return false;
}
bool IMU::choppedMeasurement(bool flag)
{
    //Sprawdzamy, czy port jest otwarty
    if (descriptor>0)
    {
        if (flag) choppedTimer->start();
        else choppedTimer->stop();
        return true;
    }
    return false;
}
bool IMU::singleMeasurement()
{
    //Sprawdzamy, czy port jest otwarty
    if (descriptor>0)
    {
        //Wysylamy komende jednego pomiarusudo
        serialPutchar(descriptor,'#');
        serialPutchar(descriptor,'f');

        //Startujemy timer odbioru
        receiverTimer->start();

        //Zwracamy powodzenie wyslania
        return true;
    }

    //Jezeli port nie jest otwarty to zwracamy blad
    return false;
}

void IMU::choppedTimerTimeout()
{
    singleMeasurement();
}
void IMU::receiverTimerTimeout()
{
    //Zmienna statyczna okreslajaca gotowosc ramki do odbioru w pomiarze ciaglym
    static bool ready=false;

    //Zmienna statyczna okreslajaca ile jest danych w kolejce FIFO
    static int w_FIFO = 0;

    //Zmienna statyczna okreslajaca ilosc odebranych danych
    static int counter=0;

    //Tworzymy kolejke fifo
    static FIFO_type continous_FIFO={{},0,0};

    //Tablica pomocnicza
    int newEulerBinary[3] = {0,0,0};

    //Jesli pomiar ciagly
    if(continous)
    {
        //Dopoki jest co odbierac
        while(serialDataAvail(descriptor))
        {
            //Pobierz dane do kolejki i zwieksz licznik danych kolejki
            unsigned int data = serialGetchar(descriptor);
            FIFO_Put(&continous_FIFO,data);
            w_FIFO++; 
        }

        //Jesli bramka jest gotowa do obioru == FIFO ma powyzej 12 danych
        while(w_FIFO>=12)
        {
            for(int i = 0; i<12;i++)
            {
                //Pobierz dana z kolejki
                unsigned int data = FIFO_Pop(&continous_FIFO);

                //Zapisz do odpowiedniego miejsca tabeli
                newEulerBinary[i/4] |= (data << 8*(i%4));

                //Ustaw flage gotowosci do obliczenia danych
                ready=true;
            }
            //Zmniejsz licznik danych w kolejce
            w_FIFO-=12;
        }
    }
    //Jesli nie pomiar ciagly
    else
    {
        //Petla po wszystkich dostepnych danych
        while (serialDataAvail(descriptor))
        {
            //Pobieranie kolejnej odebranej danej
            unsigned int data = serialGetchar(descriptor);

            //Wyswietlanie w postaci inta
            //printf("%c  ",data);

            //Przypisywanie w odpowiednie miejsce do tablicy zmiennych
            newEulerBinary[counter/4] |= (data << 8*(counter%4));

            //Inkrementowanie licznika;
            counter++;
        }
    }

    //Jezeli odebrano jakakolwiek wiadomosc lub w pomiarze ciaglym gotowy do wypisu
    if (counter!=0 || ready)
    {
        //Przepisywanie do tablicy wlasciwej
        for (int i=0;i<3;i++) eulerBinary[i]=newEulerBinary[i];

        //Wyswietlenie obliczonych danych
        printf("Euler binary = %u %u %u\n\r",eulerBinary[0],eulerBinary[1],eulerBinary[2]);

        //Obliczanie wartosci katow (IEEE754)
        for (int i=0;i<3;i++)
        {
            //Obliczanie znaku
            int sign= (eulerBinary[i] >> 31);

            //Obliczanie exponenta
            signed char exponent=((eulerBinary[i] >> 23)-127);

            //Obliczanie mantysy
            float mantisa = 1 + ((float)(eulerBinary[i] & 0x7FFFFF))/(1<<23);

            //Przypisywanie wartosci katow
            eulerAngles[i] = pow(-1,sign) * mantisa * pow (2,exponent);
        }

        //Wyswietlenie obliczonych danych
        //printf("Euler angles = %f %f %f\n\r",eulerAngles[0],eulerAngles[1],eulerAngles[2]);

        //Wykonano obliczenia, ponownie ustawic flage wypisu pomiaru ciaglego na wylaczona
        ready = false;
    }

    //Zerowanie licznika
    counter=0;

    //Jesli pomiar ciagly, wlacz ponownie odbior danych
    if (continous)
        receiverTimer->start();
}
