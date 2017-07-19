/**
 *  @file
 *  @author     Daniel Koguciuk <daniel.koguciuk@gmail.com>
 *              Dariusz Naploszek <d4rio3d@gmail.com>
 *  @date       22.08.2013
 *  @version    1.0
 */

#ifndef IMU_H
#define IMU_H

#include <QObject>
#include <QTimer>
#include <math.h>
#include "stdio.h"

#include "wiringPi.h"
#include "wiringSerial.h"
#include "Fifo.h"


/**
 *  @brief  Klasa obslugujaca odbierane od IMU dane.
 *
 *          Aby odczytywac dane od IMU najpierw nalezy otworzyc port szeregowy i sluzy do tego metoda open().
 *          Dane od IMU mozna odczytywac na dwa sposoby: jednorazowo lub ciagle. Zasada jest taka, ze przy
 *          odczycie jednorazowym wysylana jest komenda jednorazowego odczytu do urzadznia i uruchamiany jest
 *          timer, ktory odmierza 50 ms. Po tym czasie dane sa gotowe do odczytu i sa odbierane i przetwarzane
 *          z postaci binarnej do zmiennoprzecinkowej.Sposob ciagly rozni sie od jednorazowego tym, ze cala
 *          procedura, ktora jest zawarta w skanie pojedynczym jest powtarzana co jakis czas zgodnie z wartoscia
 *          zmiennej dataTimeInterval.
 */

class IMU : public QObject
{
    Q_OBJECT

public:
    /**
     *  @brief  Konstruktor klasy IMU.
     *  @param  parent  Wskaznik na rodzica typu QObject.
     */
    IMU(QObject *parent=0);

    /**
     *  @brief  Destruktor klasy IMU.
     */
    ~IMU();

    /**
     *  @brief  Metoda sluzaca do otwierania portu szeregowego.
     *  @return Wartosc logiczna oznaczajaca, czy operacja sie udala, czy nie.
     */
    bool open();

    /**
     *  @brief  Metoda zamykajaca port szeregowy.
     *  @return Wartosc logiczna oznaczajaca, czy operacja sie udala, czy nie.
     */
    bool close();

    /**
     *  @brief  Metoda sluzaca do ustawienia interwalu miedzy pomiarami od IMU (w milisekundach)
     *  @param  _interval   Zmienna typu calokowitego. Musi byc wieksza niz 50.
     *  @return Wartosc logiczna oznaczajaca, czy operacja sie udala, czy nie.
     */
    bool setDataTimeInterval(int _interval);

    /**
     *  @brief  Metoda sluzaca do pobrania odstepu miedzy pomiarami od IMU.
     *  @return Wartosc calkowita oznaczajaca interwal w milisekundach.
     */
    int getDataTimeInterval();

    /**
     *  @brief  Metoda sluzaca do aktywowania/dezaktywowania ciaglego skanu.
     *  @param  flag    Wartosc logiczna oznaczajaca, czy ciagly skan ma byc wlaczony, czy nie.
     *  @return Wartosc logiczna oznaczajaca, czy operacja sie udala, czy nie.
     */
    bool continousMeasurement(bool flag);

    /**
     *  @brief  Metoda sluzaca do aktywowania/dezaktywowania pseudo-ciaglego skanu.
     *  @param  flag    Wartosc logiczna oznaczajaca, czy ciagly skan ma byc wlaczony, czy nie.
     *  @return Wartosc logiczna oznaczajaca, czy operacja sie udala, czy nie.
     */
    bool choppedMeasurement(bool flag);

    /**
     *  @brief  Metoda sluzaca do jednorazowego pobrania wartosci katow Eulera od IMU.
     *  @return Wartosc logiczna oznaczajaca, czy operacja sie udala, czy nie.
     */
    bool singleMeasurement();

    /**
     *  @brief  Pobranie wartosci kata Eulera Pitch.
     *  @return Wartosc kata w postaci float (w stopniach).
     */
    float getPitchAngle();

    /**
     *  @brief  Pobranie wartosci kata Eulera Roll.
     *  @return Wartosc kata w postaci float (w stopniach).
     */
    float getRollAngle();

    /**
     *  @brief  Pobranie wartosci kata Eulera Yaw.
     *  @return Wartosc kata w postaci float (w stopniach).
     */
    float getYawAngle();

private slots:
    /**
     *  @brief  Slot, ktory obsluguje sygnal przepelnienia timera trybu chopped.
     */
    void choppedTimerTimeout();

    /**
     *  @brief  Slot, ktory obsluguje sygnal przepelnienia timera oczekujacego na odbior danych od IMU.
     */
    void receiverTimerTimeout();


private:
    /**
     *  @brief  Zmienna okreslajaca deskryptor urzadzenia podpietego do portu szeregowego.
     */
    int descriptor;

    /**
     *  @brief  Flaga okreslajaca rozpoczecie odbioru ciaglego.
     */
    bool continous;

    /**
     *  @brief  Timer czekajacy na odbior danych od IMU.
     */
    QTimer *receiverTimer;

    /**
     *  @brief  Timer pobierania danych od IMU trybu chopped.
     */
    QTimer *choppedTimer;

    /**
     *  @brief  Tablica 3 zmiennych odebranych od IMU w postaci binarnej float.
     */
    int eulerBinary[3];

    /**
     *  @brief  Tablica 3 katow Eulera.
     */
    float eulerAngles[3];

    /**
     *  @brief  Zmienna okreslajaca dlugosc interwalu skanowania ciaglego.
     */
    int dataTimeInterval;
};

#endif // IMU_H
