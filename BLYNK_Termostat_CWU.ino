 /*************************************************************/
#define TERMOSTAT_ver "Termoregulator CWU v1.0" 
/*************************************************************/
// TERMOREGULATOR NA PLATFORMIE https://blynk.io/
// Funkcjonalności:
// - prezentacja parametrów na wyświetlaczu LCD (bieżąca i zadana temp., czas, IP, stan grzałki, wyłączenie termoregulacji, połączenie z BLYNC)
// - termoregulacja z wykorzystaniem czujnika DS18B20
// - ustawieni zadanej temperatury z aplikacji BLYCK z telefonu komórkowego
// - funkcja wyłączenia termoregulacji
// - sterowanie urządzeniem poprzez COM, serwer Blync, enkoder
// - automatyczne włączanie termoregulacji o zadanej godzinie w cyklu tygodniowym
// - prezentacja działania urządzenia poprzez COM baud 115200
// - konfigurowanie wszystkich parametrów urządzenia przez COM

#include <BlynkSimpleEsp8266.h>    // Blynk Device - biblioteka dla ESP8266
#include <ESP8266WiFi.h>           // Wifi - biblioteka dla ESP8266   
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
//#include <RtcDS3231.h>
#include "uRTCLib.h"
uRTCLib rtc(0x68);

#include <EEPROM.h>

#define BLYNK_TEMPLATE_ID "TMPLQn1rj3zE"                    // Blynk Device - konfiguracja urządzenia
#define BLYNK_TEMPLATE_NAME "Termostat CWU"                 // Blynk Device - konfiguracja nazwy urządzenia
#define BLYNK_AUTH_TOKEN "gYoXzJS2nsqyvb97yTylaZ8TRbpJPXDR" // Blynk Device - konfiguracja TOKENU
#define BLYNK_PRINT Serial                                  //* Comment this out to disable prints and save space */

#define SCREEN_WIDTH 128          // OLED szerokość wyświetacza (pixel)
#define SCREEN_HEIGHT 64          // OLED wysokość wyświetacza (pixel)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define Grzalka_Przekaznik 12


#define Enkoder_SW 13                 //Enkoder - switch podłączony do GPI13=D7
#define Enkoder_wyj_A 2               //Enkoder - wyjście A podłączonedo GPI03=RX GPI10=S3 GPI02=D4
#define Enkoder_wyj_B 16              //Enkoder - wyjście B podłączonedo GPI16=D0
int licznik=0;

#define EE_Adress_Blynk_flaga 9    //EEPROM - Flaga zezwalająca na łączenie z Blynk = 1
#define EE_Adress_T_Bojler 10      //EEPROM - Temperatura zadana w bojlerze (Integer)
#define EE_Adress_LogoPS 14        //EEPROM - Flaga zezwolenia na logo PS
#define EE_Adress_histereza 15     //EEPROM - definiowanie adresów dla zmiennych
#define EE_Adress_WiFi_pass 16     //EEPROM - hasło do WiFi  31 znaków
#define EE_Adress_WiFi_SSID 48     //EEPROM - Nazwa Access Point dla WiFi 31 znaków x030
#define EE_Adress_BL_ID 80         //EEPROM -BLYNK_TEMPLATE_ID 15 znaków x050
#define EE_Adress_BL_NA 96         //EEPROM -BLYNK_TEMPLATE_ID 31 znaków x060
#define EE_Adress_BL_TO 128         //EEPROM -BLYNK_TEMPLATE_ID 32 znaków x080
#define EE_Adress_Prog_czas 160     //EEPROM -dane dla programatora czasowego od adresu x0A0

const byte oneWireBus=0;             //1Wire - określenie portu magistrali (D8)
OneWire oneWire(oneWireBus);         //1Wire - definicja portu magistrali dla biblioteki
DallasTemperature sensors(&oneWire); //1Wire - przekazanie bibliotee obsługi termometrów info o 1wire
DeviceAddress insideThermometer;     //1Wire - tablica do przechowywania adresów czujników

char ssid[20] = "AP21";            // WiFi - nazwa SSID
char pass[30] = "12341234";        // Wifi - hasło

String inputRS232_String = "";    // RS232 - zmienna tekstowaString to hold incoming data
bool Rs232_string_OK = false;         // RS232 - kiedy String z RS232 jest kompletny
bool Rs232_flaga_Setup = true;        // RS232 - flaga ustawiana po wejściu do setup (RS232 nie wysyła danych co 1s)
bool RS232_flaga_Termostat = false;   // Zezwala na monitorowanie funkcji termostatu na terminalu RS232
byte RS232_flaga_logoPS = 0;      // Zezwala na wyświetlenie LOGO PS na ekranie OLED


int BojlerTmpAukualna=0;        // Te parametry powinny być odczytane z pamięci
int BojlerTmpZadana=550;
byte BojlerTmpHistereza=5;     //Histereza +/- 0,5C czyli 1C
int BojlerTmpMax=0;            //Podaje maxylamną temperaturę bojlera

bool Termostat_on_off=1;       //Funkcja termostatu jest domyślie w włączona
bool BojlerGrzalkaStan=0;      //Grzałka domyślnie O1=ON 0=OFF (inwersję można zmienić w funkcji obsługi ON/OFF Grzałki

bool Enkoder_SW_stan=true;
byte Enkoder_SW_stan_procedura=0;  //Wskazuje czy została wykonana procedura mo naciśnięciu przycisku
bool Enkoder_wyj_A_stan;

bool RS232_flaga_program_czasowy=false; //ON/OFF funkcji podglądu PROGRAMATORA CZASOWEGO
bool PC_flaga_sekundy=true;             //Wykonanie sprawdzenia ustawień programatora czasowego raz na minutę, tylko gdy sek=0;

float timeout = (millis() / 1000);      //Zmienna do wyjścia z procedury łączenia z BLYNC
//DS3231  rtc(SDA, SCL);
//DS3231 clock;
//RTCDateTime dt;

//RtcDS3231<TwoWire> Rtc(Wire);


byte PrzeCzas_ON_h = 0;    //Przełącznik czasowy Godzina ON (dziesiętnie)
byte PrzeCzas_ON_i = 0;    //Przełącznik czasowy minuta ON (dziesiętnie)
byte PrzeCzas_OFF_h = 0;    //Przełącznik czasowy Godzina OFF (dziesiętnie)
byte PrzeCzas_OFF_i = 0;    //Przełącznik czasowy minuta OFF(dziesiętnie)


// 'PolitechnikaSlaska128x44', 128x44px
const unsigned char Logo_PolitechnikaSlaska128x44 [] PROGMEM = {
  0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x1f, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x04, 0x1f, 0x04, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x1c, 0xff, 0xe7, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x7f, 0xff, 0xff, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x7f, 0xff, 0xff, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x7f, 0xfb, 0xff, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x7f, 0x00, 0x1f, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xfc, 0x0c, 0x07, 0xe0, 0x03, 0x1e, 0x00, 0x11, 0x00, 0x00, 0x04, 0x00, 0x04, 0x40, 0x00, 
  0x0d, 0xf0, 0x3f, 0x01, 0xf6, 0x03, 0x1f, 0x80, 0x1b, 0x00, 0x00, 0x06, 0x00, 0x06, 0xc0, 0x00, 
  0x0f, 0xe3, 0x1f, 0x18, 0xfe, 0x03, 0x1f, 0x80, 0x18, 0x30, 0x00, 0x06, 0x00, 0x00, 0xc0, 0x00, 
  0x1f, 0xc7, 0x8f, 0x3c, 0x7f, 0x03, 0x19, 0xc6, 0x18, 0x78, 0x60, 0xe6, 0xc4, 0xc4, 0xc0, 0x70, 
  0x1f, 0xcf, 0x8e, 0x2e, 0x3f, 0x03, 0x19, 0xdf, 0x9b, 0x7d, 0xf1, 0xf7, 0xe7, 0xe6, 0xcc, 0xf8, 
  0x3f, 0x8c, 0x8e, 0x26, 0x3f, 0x83, 0x18, 0xdf, 0x9b, 0x7d, 0xfb, 0xf7, 0xf7, 0xf6, 0xcc, 0xfc, 
  0x1f, 0x0f, 0x8e, 0x3e, 0x1f, 0x03, 0x19, 0xf9, 0xdb, 0x33, 0x1b, 0x06, 0x76, 0x76, 0xdc, 0x1c, 
  0x0f, 0x0f, 0x9e, 0x3e, 0x1e, 0x03, 0x1f, 0xb0, 0xdb, 0x33, 0xfb, 0x06, 0x76, 0x76, 0xf8, 0xfc, 
  0x1e, 0x1f, 0xdf, 0x7f, 0x0f, 0x03, 0x1f, 0xb0, 0xdb, 0x33, 0xfb, 0x06, 0x76, 0x76, 0xf9, 0xfc, 
  0x1e, 0x1f, 0xff, 0xff, 0x0f, 0x03, 0x18, 0x31, 0xdb, 0x33, 0x03, 0x06, 0x76, 0x76, 0xf9, 0x9c, 
  0x1e, 0x3f, 0xff, 0xdf, 0x8f, 0x03, 0x18, 0x39, 0x9b, 0x33, 0x83, 0x86, 0x76, 0x76, 0xdd, 0xdc, 
  0x7c, 0x7f, 0xdf, 0x7f, 0xc7, 0xc3, 0x18, 0x1f, 0x9b, 0x3d, 0xfb, 0xf6, 0x76, 0x76, 0xcf, 0xfc, 
  0xfc, 0x7f, 0xff, 0xff, 0x47, 0xe3, 0x18, 0x0f, 0x1b, 0x38, 0xf9, 0xf6, 0x36, 0x26, 0xc6, 0xec, 
  0xfc, 0x3f, 0xff, 0xff, 0x87, 0xe3, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xfc, 0x3f, 0xff, 0xff, 0x87, 0xe3, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xfc, 0x7f, 0xff, 0xff, 0xc7, 0xe3, 0x0f, 0xb0, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x7c, 0x7f, 0xdf, 0x7f, 0xc7, 0xc3, 0x1f, 0xb0, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x1e, 0x1e, 0xff, 0xef, 0x0f, 0x03, 0x18, 0x30, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x1e, 0x1f, 0xff, 0xff, 0x0f, 0x03, 0x38, 0x33, 0xe7, 0xd9, 0x9f, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x1e, 0x31, 0xff, 0xf1, 0x8f, 0x03, 0x18, 0x33, 0xe7, 0xd9, 0x9f, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x1f, 0x25, 0xdf, 0x74, 0x9e, 0x03, 0x1f, 0x30, 0x76, 0x19, 0x81, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x3f, 0x0f, 0x1f, 0x1e, 0x1f, 0x03, 0x0f, 0xb1, 0xf7, 0x1f, 0x87, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x3f, 0x8f, 0xff, 0xfe, 0x3f, 0x83, 0x03, 0xb3, 0xf7, 0xdf, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x1f, 0xc4, 0xff, 0xe4, 0x7f, 0x03, 0x01, 0xf7, 0xf3, 0xdf, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x1f, 0xc0, 0x3f, 0x80, 0x7f, 0x03, 0x01, 0xf6, 0x30, 0xff, 0xb9, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x0f, 0xe0, 0x0e, 0x00, 0xfe, 0x03, 0x01, 0xb7, 0x70, 0xfb, 0xbb, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x0d, 0xf0, 0x04, 0x01, 0xf6, 0x03, 0x1f, 0xb7, 0xf7, 0xd9, 0xdf, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xfc, 0x04, 0x07, 0xe0, 0x03, 0x1f, 0x33, 0xb7, 0x98, 0xdd, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x7f, 0x00, 0x1f, 0xc0, 0x03, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x7f, 0xfb, 0xff, 0xc0, 0x03, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x7f, 0xff, 0xff, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x7f, 0xff, 0xff, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x1c, 0xff, 0xe7, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x04, 0x3f, 0x04, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x1f, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
/*
// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 720)
const int epd_bitmap_allArray_LEN = 1;
const unsigned char* epd_bitmap_allArray[1] = {
  epd_bitmap_PolitechnikaSlaska128x44
};
*/

// 'Politechnika Śląska', 43x44px
const unsigned char Logo_Politechnika_Slaska [] PROGMEM = {
  0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x04, 0x1f, 0x04, 
  0x00, 0x00, 0x00, 0x1c, 0xff, 0xe7, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x7f, 
  0xff, 0xff, 0xc0, 0x00, 0x00, 0x7f, 0xfb, 0xff, 0xc0, 0x00, 0x00, 0x7f, 0x00, 0x1f, 0xc0, 0x00, 
  0x00, 0xfc, 0x0c, 0x07, 0xe0, 0x00, 0x0d, 0xf0, 0x3f, 0x01, 0xf6, 0x00, 0x0f, 0xe3, 0x1f, 0x18, 
  0xfe, 0x00, 0x1f, 0xc7, 0x8f, 0x3c, 0x7f, 0x00, 0x1f, 0xcf, 0x8e, 0x2e, 0x3f, 0x00, 0x3f, 0x8c, 
  0x8e, 0x26, 0x3f, 0x80, 0x1f, 0x0f, 0x8e, 0x3e, 0x1f, 0x00, 0x0f, 0x0f, 0x9e, 0x3e, 0x1e, 0x00, 
  0x1e, 0x1f, 0xdf, 0x7f, 0x0f, 0x00, 0x1e, 0x1f, 0xff, 0xff, 0x0f, 0x00, 0x1e, 0x3f, 0xff, 0xdf, 
  0x8f, 0x00, 0x7c, 0x7f, 0xdf, 0x7f, 0xc7, 0xc0, 0xfc, 0x7f, 0xff, 0xff, 0x47, 0xe0, 0xfc, 0x3f, 
  0xff, 0xff, 0x87, 0xe0, 0xfc, 0x3f, 0xff, 0xff, 0x87, 0xe0, 0xfc, 0x7f, 0xff, 0xff, 0xc7, 0xe0, 
  0x7c, 0x7f, 0xdf, 0x7f, 0xc7, 0xc0, 0x1e, 0x1e, 0xff, 0xef, 0x0f, 0x00, 0x1e, 0x1f, 0xff, 0xff, 
  0x0f, 0x00, 0x1e, 0x31, 0xff, 0xf1, 0x8f, 0x00, 0x1f, 0x25, 0xdf, 0x74, 0x9e, 0x00, 0x3f, 0x0f, 
  0x1f, 0x1e, 0x1f, 0x00, 0x3f, 0x8f, 0xff, 0xfe, 0x3f, 0x80, 0x1f, 0xc4, 0xff, 0xe4, 0x7f, 0x00, 
  0x1f, 0xc0, 0x3f, 0x80, 0x7f, 0x00, 0x0f, 0xe0, 0x0e, 0x00, 0xfe, 0x00, 0x0d, 0xf0, 0x04, 0x01, 
  0xf6, 0x00, 0x00, 0xfc, 0x04, 0x07, 0xe0, 0x00, 0x00, 0x7f, 0x00, 0x1f, 0xc0, 0x00, 0x00, 0x7f, 
  0xfb, 0xff, 0xc0, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xc0, 0x00, 
  0x00, 0x1c, 0xff, 0xe7, 0x00, 0x00, 0x00, 0x04, 0x3f, 0x04, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 288)
//const int epd_bitmap_allArray_LEN = 1;
//const unsigned char* epd_bitmap_allArray[1] = {
//  epd_bitmap_Politechnika__l_ska
//};
//-------------------------------------------

BlynkTimer timer;

//-------------------------------------------------------------------------------------------------------------
//  BLYNK - obsługa serwera
//-------------------------------------------------------------------------------------------------------------

//----- BLYNK V0 -------------------- Funkcja wywoływana za każdym razem kiedy zmieni się wartość Virtual Pin 0
BLYNK_WRITE(V0)
{
  int value = param.asInt();                      // Set incoming value from pin V0 to a variable
  //Serial.print("TERMOSTAT: ");
  //Serial.println(value);
 //digitalWrite(Grzalka_Przekaznik, value);
  
  Termostat_on_off= value; 
  if(!Termostat_on_off) {
    BojlerGrzalkaStan=0;//Wyłącz grzałkę jeśl wyłączono termoregulację
    Grzalka_ON_OFF();
    //Serial.print("VO wylącz termoregulację V0:"); Serial.println(Termostat_on_off);
  }
  
  // Update state
  //Blynk.virtualWrite(V3, BojlerGrzalkaStan);    //Stan ON/OFF grzałki wysłany na Blynk
  //delay(100);
  //Blynk.virtualWrite(V1, (BojlerTmpAukualna) / 10 );    //Wysłanie zmiennych na Blynk
}
//----- BLYNK V2 -------------------- Funkcja wywoływana za każdym razem kiedy zmieni się war
BLYNK_WRITE(V2)
{
  int value = param.asInt();                      // Set incoming value from pin V0 to a variable
    BojlerTmpZadana = value * 10;  
    Serial.print("  TERMOSTAT: T_zadana: ");
    Serial.println(float(BojlerTmpZadana)/10, 1); 
    EEPROM.put(EE_Adress_T_Bojler, BojlerTmpZadana);  EEPROM.commit(); 
}
//----- BLYNK V4 -------------------- Funkcja wywoływana za każdym razem kiedy zmieni się wartość Virtual Pin 4
BLYNK_WRITE(V4)                         //PC_1 ON/OFF ustawienia programatora czasowego
{
  TimeInputParam t(param);
  String PC_string_ustawianie="1 ON ";  
/*  if (t.hasStartTime())
  {
    Serial.println(String("Start: ") +
                   t.getStartHour() + ":" +
                   t.getStartMinute() + ":" +
                   t.getStartSecond());
  }
  if (t.hasStopTime())
  {
    Serial.println(String("Stop: ") +
                   t.getStopHour() + ":" +
                   t.getStopMinute() + ":" +
                   t.getStopSecond());
  }  
  for (int i = 1; i <= 7; i++) {
    if (t.isWeekdaySelected(i)) {
      Serial.println(String("Day ") + i + " is selected");
    }
  }
*/
 // Serial.println();
  if (t.getStartHour() < 10 ) PC_string_ustawianie = PC_string_ustawianie + "0"; PC_string_ustawianie = PC_string_ustawianie + String(t.getStartHour()) + ":";
  if (t.getStartMinute() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStartMinute()) + " ";
  if (t.getStopHour() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStopHour()) + ":";
  if (t.getStopMinute() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStopMinute()) + " "; 
  if (t.isWeekdaySelected(1)) PC_string_ustawianie += "P"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(2)) PC_string_ustawianie += "W"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(3)) PC_string_ustawianie += "S"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(4)) PC_string_ustawianie += "C"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(5)) PC_string_ustawianie += "K"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(6)) PC_string_ustawianie += "S"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(7)) PC_string_ustawianie += "N"; else PC_string_ustawianie += "_";
  Serial.println("  V4:" + PC_string_ustawianie);
// Serial.println(String("Time zone: ") + t.getTZ());
// Serial.println(String("Time zone offset: ") + t.getTZ_Offset());
  RS232_str_programator_czasowy(PC_string_ustawianie); 
}
//----- BLYNK V5 -------------------- Funkcja wywoływana za każdym razem kiedy zmieni się wartość Virtual Pin
BLYNK_WRITE(V5)                        //PC_2 ON/OFF ustawienia programatora czasowego
{
  TimeInputParam t(param);
  String PC_string_ustawianie="2 ON ";
  if (t.getStartHour() < 10 ) PC_string_ustawianie = PC_string_ustawianie + "0"; PC_string_ustawianie = PC_string_ustawianie + String(t.getStartHour()) + ":";
  if (t.getStartMinute() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStartMinute()) + " ";
  if (t.getStopHour() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStopHour()) + ":";
  if (t.getStopMinute() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStopMinute()) + " "; 
  if (t.isWeekdaySelected(1)) PC_string_ustawianie += "P"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(2)) PC_string_ustawianie += "W"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(3)) PC_string_ustawianie += "S"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(4)) PC_string_ustawianie += "C"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(5)) PC_string_ustawianie += "K"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(6)) PC_string_ustawianie += "S"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(7)) PC_string_ustawianie += "N"; else PC_string_ustawianie += "_";
  //Serial.println("  V5:" + PC_string_ustawianie);
  RS232_str_programator_czasowy(PC_string_ustawianie);
}
//----- BLYNK V6 -------------------- Funkcja wywoływana za każdym razem kiedy zmieni się wartość Virtual Pin
BLYNK_WRITE(V6)                        //PC_3 ON/OFF ustawienia programatora czasowego
{
  TimeInputParam t(param);
  String PC_string_ustawianie="3 ON ";
  if (t.getStartHour() < 10 ) PC_string_ustawianie = PC_string_ustawianie + "0"; PC_string_ustawianie = PC_string_ustawianie + String(t.getStartHour()) + ":";
  if (t.getStartMinute() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStartMinute()) + " ";
  if (t.getStopHour() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStopHour()) + ":";
  if (t.getStopMinute() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStopMinute()) + " "; 
  if (t.isWeekdaySelected(1)) PC_string_ustawianie += "P"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(2)) PC_string_ustawianie += "W"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(3)) PC_string_ustawianie += "S"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(4)) PC_string_ustawianie += "C"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(5)) PC_string_ustawianie += "K"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(6)) PC_string_ustawianie += "S"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(7)) PC_string_ustawianie += "N"; else PC_string_ustawianie += "_";
  //Serial.println("  V6:" + PC_string_ustawianie);
  RS232_str_programator_czasowy(PC_string_ustawianie);
}
//----- BLYNK V7 -------------------- Funkcja wywoływana za każdym razem kiedy zmieni się wartość Virtual Pin 0
BLYNK_WRITE(V7)                        //PC_4 ON/OFF ustawienia programatora czasowego
{
  TimeInputParam t(param);
  String PC_string_ustawianie="4 ON ";
  if (t.getStartHour() < 10 ) PC_string_ustawianie = PC_string_ustawianie + "0"; PC_string_ustawianie = PC_string_ustawianie + String(t.getStartHour()) + ":";
  if (t.getStartMinute() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStartMinute()) + " ";
  if (t.getStopHour() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStopHour()) + ":";
  if (t.getStopMinute() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStopMinute()) + " "; 
  if (t.isWeekdaySelected(1)) PC_string_ustawianie += "P"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(2)) PC_string_ustawianie += "W"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(3)) PC_string_ustawianie += "S"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(4)) PC_string_ustawianie += "C"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(5)) PC_string_ustawianie += "K"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(6)) PC_string_ustawianie += "S"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(7)) PC_string_ustawianie += "N"; else PC_string_ustawianie += "_";
  //Serial.println("  V7:" + PC_string_ustawianie);
  RS232_str_programator_czasowy(PC_string_ustawianie);
}
//----- BLYNK V8 -------------------- Funkcja wywoływana za każdym razem kiedy zmieni się wartość Virtual Pin 0
BLYNK_WRITE(V8)                        //PC_5 ON/OFF ustawienia programatora czasowego
{
  TimeInputParam t(param);
  String PC_string_ustawianie="5 ON ";
   if (t.getStartHour() < 10 ) PC_string_ustawianie = PC_string_ustawianie + "0"; PC_string_ustawianie = PC_string_ustawianie + String(t.getStartHour()) + ":";
  if (t.getStartMinute() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStartMinute()) + " ";
  if (t.getStopHour() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStopHour()) + ":";
  if (t.getStopMinute() < 10 ) PC_string_ustawianie += "0"; PC_string_ustawianie += String(t.getStopMinute()) + " "; 
  if (t.isWeekdaySelected(1)) PC_string_ustawianie += "P"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(2)) PC_string_ustawianie += "W"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(3)) PC_string_ustawianie += "S"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(4)) PC_string_ustawianie += "C"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(5)) PC_string_ustawianie += "K"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(6)) PC_string_ustawianie += "S"; else PC_string_ustawianie += "_";
  if (t.isWeekdaySelected(7)) PC_string_ustawianie += "N"; else PC_string_ustawianie += "_";
  //Serial.println("  V8:" + PC_string_ustawianie);
  RS232_str_programator_czasowy(PC_string_ustawianie);
}
//----- BLYNK V9 -------------------- Funkcja wywoływana za każdym razem kiedy zmieni się wartość Virtual Pin 0
BLYNK_WRITE(V9)                        //PC_1 ON/OFF włącza wyłącza ustawienie nr PC_1 przełącznik
{
 int value = param.asInt();                         //Odczyt stanu przełącznika w aplikacji BLYNC - ON/ONN ustawienia PC_1 programatora czasowego
 int EE_Adress_Programator = EE_Adress_Prog_czas;   //Ustawienie początkowego adresu danych w EEPROM DLA pROGRAMATORA CZASOWEGO
 EEPROM.write(EE_Adress_Programator, value);EEPROM.commit();
 RS232_str_programator_czasowy("");                 //Ustawienia PROGRAMATORA CZASOWEGO na ekran
} 
//----- BLYNK CONNECTED -------------- Urządzenie połączone z Blynk.Cloud
BLYNK_CONNECTED()
{
 // Blynk.syncVirtual(V0);
    
    // Change Web Link Button message to "Congratulations!"
  //Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  //Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  //Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}
//----- BLYNK UPTIME -------------- Wysyłanie danych z Arduino do Virtual Pin co sekundę
void Timer_1s()
{
 
  Blynk.virtualWrite(V0,Termostat_on_off);  //Serial.print("VO wyslane na Blynk:"); Serial.println(Termostat_on_off);
  Blynk.virtualWrite(V1,float(BojlerTmpAukualna) /10);    //Wysłanie zmiennych na Blynk
  Blynk.virtualWrite(V2, BojlerTmpZadana / 10);
  Blynk.virtualWrite(V3, BojlerGrzalkaStan); 

  int EE_Adress_Programator = EE_Adress_Prog_czas;   //Ustawienie początkowego adresu danych w EEPROM DLA pROGRAMATORA CZASOWEGO
  byte value = EEPROM.read(EE_Adress_Programator);   
  Blynk.virtualWrite(V9, value);  
    
//        if (i==0) {  //Czy programator czasowy jest włączony
//        Serial.print("  "); Serial.print(Prog_czasowy_zestaw);
//        if (value == 0) Serial.print(" OFF"); else Serial.print("  ON");  
//      }
  

  // Blynk.virtualWrite(V4, millis() / 1000);

   //Serial.print("Requesting temperatures...");
   sensors.requestTemperatures(); // Send the command to get temperatures

   Temp_odczyt_pomiaru(insideThermometer); // Use a simple function to print out the data
 
   Termostat();
 
   //Serial.println(rtc.getTimeStr());
  
   OLED_oblsuga_wyswietlacza();  //OLED - aktualne dane na wyświetlacz

 //----- ENCDER -------------------------------------------------------
 if (Enkoder_SW_stan==0)  //wciśnięto SW na Enkoderze
{
    if (Enkoder_SW_stan_procedura == 0)
    {
      Termostat_on_off = !Termostat_on_off; Enkoder_SW_stan_procedura = 2;
    }        
      if (digitalRead(Enkoder_SW)) Enkoder_SW_stan = 1;   //Spr. stan przycisku, jeśli puszczony to zmień stan zmiennej na 1 (eliminacja drgań i długiegi trzymania             
} else {   // przycisk już jest puszczony więc poczekaj chwilę na ponowne obsługę pomimo przyciśnięcia i wywołania przerwania (dłużej się czaka)   
      
    Enkoder_SW_stan = 1; 
    if (Enkoder_SW_stan_procedura != 0) --Enkoder_SW_stan_procedura;     
}


// Serial.println(licznik);
/*
  RtcDateTime now = Rtc.GetDateTime();
    //if (!wasError("loop GetDateTime"))
    //{
        printDateTime(now);
        Serial.print(" ->");Serial.print(Rtc.Hour());Serial.print(":");Serial.print(Wire.Minute());
        Serial.println();
    //}
*/

/*  rtc.refresh();
  Serial.print("RTC DateTime: ");
  Serial.print(rtc.year());
  Serial.print('/');
  Serial.print(rtc.month());
  Serial.print('/');
  Serial.print(rtc.day());

  Serial.print(' ');

  Serial.print(rtc.hour());
  Serial.print(':');
  Serial.print(rtc.minute());
  Serial.print(':');
  Serial.print(rtc.second());

  Serial.print(" DOW: ");
  Serial.print(rtc.dayOfWeek());

  Serial.print(" - Temp: ");
  Serial.print(rtc.temp()  / 100);

  Serial.println();
*/
  
Programator_czasowy();
 
}
//-------------------------------------------------------------------------------------------------------------
//--- PRZERWANIA ----------------------------------------------------------------------------------------------
 void zliczISR() {    
    Enkoder_SW_stan = 0; licznik++;
}
 void EncoderISR() {    
   if (digitalRead(Enkoder_wyj_B))
   {
    licznik--;
    if (BojlerTmpZadana >200) BojlerTmpZadana -= 10;
   }else {
    licznik++; if (BojlerTmpZadana <700) BojlerTmpZadana += 10;
   }
   EEPROM.put(EE_Adress_T_Bojler, BojlerTmpZadana);  EEPROM.commit(); 
}
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//----- SETUP -------------------------------------------------------   
void setup()
{
  pinMode(Grzalka_Przekaznik, OUTPUT);                   //Przekaźnik - aktywacja wyjścia
  Grzalka_ON_OFF();                                      //domyślnie wyłącznony 
  
  pinMode(Enkoder_SW, INPUT_PULLUP);          //Enkoder_WS  - piny procesora ustawione jako wejścia
  pinMode(Enkoder_wyj_A, INPUT);       //Enkoder_wyj_A
  pinMode(Enkoder_wyj_B, INPUT);       //Enkoder_wyj_B

  EEPROM.begin(256);

  // rtc.set(0, 11, 10, 5, 2, 6, 23);
  //  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year
    
  Serial.begin(115200);           //RS232 ustawienie prędkości transmisji
  //inputRS232_String.reserve(20);  //10 znaków zarezerwowane na zmienną z Rs232
                                  //   WiFi.begin("Miszorro_Operatorro", "Mikol143"); // zainicjowanie wifi modułu 
  delay(100); 
   
   //--- EEPROM odzyt ustawień jaśli są ---------------------------------------------------
  byte BojlerTmp_h; BojlerTmp_h = EEPROM.read(EE_Adress_histereza);
  if (BojlerTmp_h > 0 && BojlerTmp_h < 100) BojlerTmpHistereza = BojlerTmp_h; //Czy histereza jest różna od zera i nie większa od 100 
     
  RS232_flaga_logoPS = EEPROM.read(EE_Adress_LogoPS);   //Odczyt Flagi z EEPROM 
   
  int BojlerTmpZadana_TMP; EEPROM.get(EE_Adress_T_Bojler, BojlerTmpZadana_TMP); //Odczyt Tmp. Zadanej dla bojlera z EEPROM
  if (BojlerTmpZadana_TMP <= 800 && BojlerTmpZadana_TMP >=200) BojlerTmpZadana = BojlerTmpZadana_TMP;       //czy wygląda na zapisaną w EEPROM

  String WiFi_pass = EE_Odczyt_String(EE_Adress_WiFi_pass);     //Odczyt hasła zapisanego w EEPROM
   if (WiFi_pass.length()<=32 && WiFi_pass.length()>1)
   {
    for(byte i = 0; i < WiFi_pass.length() ; i++)
        {
          pass[i] = char(WiFi_pass[i]); if ((i+1) == WiFi_pass.length()) pass[i+1] = 0;      //wstaw zero na końcu ciągu char     
        }
   }

   String WiFi_SSID = EE_Odczyt_String(EE_Adress_WiFi_SSID);     //Odczyt SSID zapisanego w EEPROM
   if (WiFi_SSID.length()<=32 && WiFi_SSID.length()>1 ){
     for(byte i = 0; i < WiFi_SSID.length() ; i++)
     {
       ssid[i] = char(WiFi_SSID[i]); if ((i+1) == WiFi_SSID.length()) ssid[i+1] = 0;      //wstaw zero na końcu ciągu char     
     }
   }  
  
  
    Serial.println("");Serial.println("");
    Serial.print("(c) "); Serial.print(TERMOSTAT_ver); Serial.println(" - regulator temperatury wody w bojlerze");    
    Serial.println(""); Serial.print("  Inicjalizacja SSD1306....."); 
      if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
       Serial.println(F("SSD1306 brak wyświetlacza!"));
       for(;;); // Don't proceed, loop forever
      } else Serial.println("OK");
  display.clearDisplay(); display.display();
  //----- LOGO NA LCD ------------------------------------------------
  Serial.print("  Logo PS na OLED....");
  if (RS232_flaga_logoPS !=0 ) {                                //Czy wyświetlić logo na LCD?
    Serial.println("OK");
     display.clearDisplay(); 
     display.drawBitmap(0, 8, Logo_Politechnika_Slaska, 43, 44, WHITE); // Draw the bitmap: drawBitmap(x position, y position, bitmap data, bitmap width, bitmap height, color)
     display.display();  // Update the display
  
     display.startscrollright(0x00, 0x0F);
     delay(1000);
       display.stopscroll();
       display.startscrollleft(0x00, 0x0F);
         delay(1000);
         display.stopscroll();
         //delay(1000);
 
     display.clearDisplay();
     display.drawBitmap(0, 8, Logo_PolitechnikaSlaska128x44, 128, 44, WHITE);
     display.display(); //delay(3000);
   } else Serial.println("NO");
  
  delay(2000);
  
  //rtc.begin();

  //----- 1Wire DS18B20 ------------------------------------------------------- 
  Serial.print("  Szukam czujnikow temperatury....");
    sensors.begin();
      Serial.print("Znalezione...: ");
      Serial.print(sensors.getDeviceCount(), DEC);
      Serial.println(" czujnikow.");

  // report parasite power requirements
  Serial.print("     Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

// Method 1:
  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then 
  // use those addresses and manually assign them (see above) once you know 
  // the devices on your bus (and assuming they don't change).
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("     Unable to find address for Device 0"); 
// method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices, 
  // or you have already retrieved all of them. It might be a good idea to 
  // check the CRC to make sure you didn't get garbage. The order is 
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to insideThermometer
  //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");

  // show the addresses we found on the bus
  Serial.print("     Czujnik 0 Adres: ");
    printAddress(insideThermometer);
    Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);
 
  Serial.print("     Czujnik 0 Rozdzielczosc: ");
    Serial.print(sensors.getResolution(insideThermometer), DEC); 
    Serial.println();


  Serial.println("  Timer 1s ...ON");
  timer.setInterval(200L, Timer_1s);     //Deklaracaj FUNKCJI wywoływanej co 1s 


   Blynk.config(BLYNK_AUTH_TOKEN, "blynk-cloud.com", 8442); Blynk.config(BLYNK_AUTH_TOKEN);  //Kon
   WiFi.begin(ssid, pass);
//  if (EEPROM.read(EE_Adress_Blynk_flaga) == 1)  //Czy uruchamiać Blynk 
//   {
   
  //DODać odczyt danych konfiguracyjnych
    
//   Serial.print("  BLYNC uruchamianie...");     //TAK
       
 //   Serial.println("entering Blynk.config()");
   
 // Serial.println("entering Blynk.connect()");
//  while (Blynk.connect() == false) {
//    Serial.println("in while: Blink.connect() == false");
//    if (((millis() / 1000) - timeout) > 5) {
//      break;
//    }
//  }

  //Serial.println("setup complete");
         
   Serial.println("procedua zakończona.");
  // You can also specify server:
  // Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud", 80);
  // Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, IPAddress(192,168,1,100), 8080);
  //}
  
  attachInterrupt(digitalPinToInterrupt(Enkoder_SW), zliczISR, FALLING);
  delay(50);
  attachInterrupt(digitalPinToInterrupt(Enkoder_wyj_A), EncoderISR, FALLING);  

 //rtc.set(0, 11, 10, 5, 2, 6, 23);

}
//-------------------------------------------------------------------------------------------------------------
// function to print the temperature for a device
void Temp_odczyt_pomiaru(DeviceAddress deviceAddress)
{
    // DS_temperatura = sensors.getTempC(deviceAddress);                  //Temperatura Float
  BojlerTmpAukualna = 10 * sensors.getTempC(deviceAddress) + 0,04;    //22,44 na 224
 
    if(BojlerTmpAukualna == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: brak czujnika temperatury!");
    return;
    BojlerTmpAukualna = -1270;                                 //Brak czujnika temperatury
    }
 
    if (BojlerTmpAukualna > BojlerTmpMax) BojlerTmpMax = BojlerTmpAukualna;  //Zapis do zmiennej maxymalnej temperatury w bojlerze

 // Serial.print("Temp C: ");
 // if (BojlerTmpAukualna==-1270) Serial.print("--.-");
 // else Serial.print(float(BojlerTmpAukualna) /10 ,1);
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  } 
}

//-------------------------------------------------------------------------------------------------------------
//----- PĘTLA GŁÓWNA PROGRAMU ----------------------------------------
//-------------------------------------------------------------------------------------------------------------
void loop()
{
  timer.run();
  // Blynk.run(); 

  if (EEPROM.read(EE_Adress_Blynk_flaga) == 1)  //Czy uruchamiać Blynk 
  {
     //  WiFi.begin(ssid, pass);
      if (WiFi.status() == WL_CONNECTED) {      //Czyjest połaczenie WiFi
         if (Blynk.connected() == true) {
          Blynk.run();
         }
         else {
         Blynk.connect();

  while (Blynk.connect() == false) {
    Serial.println("in while: Blink.connect() == false");
    if (((millis() / 1000) - timeout) > 8) {
      break;
    }
  }        
         Blynk.run();
    }
  }
  }
  
  RS232_obsluga();   
 
/*  if (Enkoder_SW_poprzedni_stan != digitalRead(Enkoder_SW)) {
     Serial.println("WS wciśnięty");
  if(!digitalRead(Enkoder_SW)){    //Wciśniety SWITCH =1
    Serial.println("SWITCH!!!!!!");
    Termostat_on_off = !Termostat_on_off;
    Enkoder_SW_poprzedni_stan = digitalRead(Enkoder_SW);
  } 
  } 
    Enkoder_SW_poprzedni_stan = digitalRead(Enkoder_SW);
 */
 // ------ RS232 ------------------------- odbieranie zmiennej String
 while (Serial.available()) {   //
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    //inputRS232_String += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
      if (inChar == '\n') Rs232_string_OK = true;     //Odebranie ciąu znaków z RS232 bez znaku powrotu linii
      else inputRS232_String += inChar;
  }
   //Serial.print("Termostat uruchomiony!");  
}
//-------------------------------------------------------------------------
// OLED - obsługa wyświetlacza
//-------------------------------------------------------------------------
void OLED_oblsuga_wyswietlacza()     // OLED - Ekran główny
{  
  display.clearDisplay();   //Wyczyszczenie pamięci ekranu pod nowe dane
  display.setTextColor(SSD1306_WHITE);
 
  display.setTextSize(1);                  // Linia 1 -----------------------
  display.setCursor(0, 0);                 // Ustawienie kursora 
  //display.print(F("12:00   G:"));                        
  if (rtc.hour()<10) display.print(F(" ")); display.print(rtc.hour()); display.print(F(":")); 
  if (rtc.minute()<10) display.print(F("0")); display.print(rtc.minute()); display.print(F("   G:")); 
  if (BojlerGrzalkaStan) display.print(" ON   ");
  else display.print("OFF   ");

  if (Blynk.connected() == true) display.print("BLY"); else display.print("---");
  
  //display.drawLine(0, 10, 1, display.height()-1, SSD1306_WHITE);  // Podkreślenie
  display.drawLine(0, 9, display.width()-1, 9, SSD1306_WHITE);
  
  display.setTextSize(2);                  // Linia 2 (EKRAN GŁÓWNY) ---------
  display.setCursor(4, 15);
  display.print(F("T"));
  
  display.setTextSize(1);
  display.setCursor(14, 22);
  display.println(F("SET"));

  display.setTextSize(2);
  display.setCursor(6, 35);
  if ((!Termostat_on_off)||(BojlerTmpAukualna == -1270)) display.println("OFF"); else display.println(BojlerTmpZadana/10);
  //display.cp437(true);
  //display.print((char)223);
  
  display.setTextSize(3);
  display.setCursor(46, 19);
  if (BojlerTmpAukualna==-1270) display.println("--.-"); else display.println(float(BojlerTmpAukualna) /10 ,1); 
  //display.println(tempC);
  //display.print((char)223);  //stopień Celcjusza
   
  display.drawLine(0, 54, display.width()-1, 54, SSD1306_WHITE);  // Podkreślenie
  display.setTextSize(1);
  display.setCursor(0, 57); 
  display.print("IP:"); // Linia 3 -----------------------    
  
  display.println(WiFi.localIP());
  
  display.display();      // Show initial text 
}
//-------------------------------------------------------------------------
// TERMOSTAT - realizacja funkcji
//-------------------------------------------------------------------------
void Termostat()          // TERMOSTAT procedura ON/OFF grzałki
{  
  if (Termostat_on_off) {     //Termoregulacja przy włączonej fladze zezwalająej oraz poprawnym odczycie temperatury

      if  (BojlerTmpAukualna == -1270) {
           BojlerGrzalkaStan = 0; 
      } else {
          if (BojlerTmpAukualna <= (BojlerTmpZadana - BojlerTmpHistereza)) BojlerGrzalkaStan = 1;   // ON grzałkę w procedurze Grzalka_ON_OFF
          if (BojlerTmpAukualna >= (BojlerTmpZadana + BojlerTmpHistereza)) BojlerGrzalkaStan = 0;   // OFFN grzałkę w procedurze Grzalka_ON_OFF
      }
  } else BojlerGrzalkaStan = 0;
    
  //if  (BojlerTmpAukualna == -1270) BojlerGrzalkaStan = 0;      //Brak termometru, wyłą grzałkę   
  //Serial.print("Gezałka "); Serial.println(BojlerGrzalkaStan);
  
  Grzalka_ON_OFF();

  if (RS232_flaga_Termostat) { 
    Serial.print("  TERMOSTAT: "); 
    if (Termostat_on_off) {
      Serial.print("ON"); Serial.print(" T:"); if (BojlerTmpAukualna != -1270) Serial.print(float(BojlerTmpAukualna) /10 ,1); else Serial.print("--.-");
      Serial.print(" Zadana:"); Serial.print(float(BojlerTmpZadana) /10, 1);
      Serial.print(" Zadana-H:"); Serial.print(float(BojlerTmpZadana - BojlerTmpHistereza) /10, 1);
      Serial.print(" Zadana+H:"); Serial.print(float(BojlerTmpZadana + BojlerTmpHistereza) /10, 1); 
      Serial.print(" G:"); if (BojlerGrzalkaStan) Serial.println("ON"); else Serial.println("OFF");
    
      } else Serial.println("OFF");     
  }    
}
void Grzalka_ON_OFF()     // GRZAŁKA - ON/OFFy
{ 
  if (BojlerGrzalkaStan ==1 ) digitalWrite(Grzalka_Przekaznik, 1);     // ON/OFF grzałki w zależności od flagi BojlerGrzalkaStan
  else digitalWrite(Grzalka_Przekaznik, 0);
}

//-------------------------------------------------------------------------
// RS232 - obsługa transmisji szeregowej
//-------------------------------------------------------------------------
void RS232_obsluga() {
   if (Rs232_string_OK) { 
     
     int i;
     boolean inputRS232_komenda_dana = false;         //0-składanie komendy,1-składanie danej
     int znakDEC;
     String inputRS232_komenda = "";String inputRS232_dana = "";
      
    //Serial.print("To jest zmienna STR:"); Serial.print(inputRS232_String);Serial.print("znaków:");Serial.println(sizeof(inputRS232_String));
     
     for(i = 0; i < inputRS232_String.length() ; i++)
       {            
           znakDEC = int(inputRS232_String[i]);                    //Rozdzielenie komendy i danej poprzez szukanie spacji
           // Serial.print(i, DEC); Serial.print(" -> "); 
           if (int(inputRS232_String[i]) == 32 && inputRS232_komenda_dana==false) {  //Szukanie tylko pierwszej spacji w tekście
           inputRS232_komenda_dana = true; i++;
           }
            
           if (!inputRS232_komenda_dana) inputRS232_komenda += inputRS232_String[i]; else inputRS232_dana += inputRS232_String[i];
           
        //Serial.print(znakDEC, DEC); Serial.print(" = "); Serial.write(inputRS232_String[i]); Serial.println();
       //Serial.print("RS_komenda:");Serial.println(inputRS232_komenda);
       //Serial.print("   RS_dana:");Serial.println(inputRS232_dana);
       }         
       if (inputRS232_String == "?") RS232_str_pomoc();
       if (inputRS232_String == "T") RS232_str_termo();
       if (inputRS232_String == "") RS232_str_ustaw();       
       
       if (inputRS232_komenda == "pass") RS232_str_pass(inputRS232_dana);  
       if (inputRS232_komenda == "H") RS232_str_histereza(inputRS232_dana);  

       if (inputRS232_komenda == "EE") RS232_str_EEPROM_zawartosc(inputRS232_dana);
       if (inputRS232_komenda == "L")  RS232_str_LogoPS(inputRS232_dana);
       if (inputRS232_komenda == "B")  RS232_str_BojlerZadana(inputRS232_dana);
       if (inputRS232_komenda == "S")  RS232_str_SSID(inputRS232_dana);
       if (inputRS232_komenda == "ID")  RS232_str_ID(inputRS232_dana);  
       if (inputRS232_komenda == "NA")  RS232_str_NA(inputRS232_dana);  
       if (inputRS232_komenda == "TO")  RS232_str_TO(inputRS232_dana);  
       if (inputRS232_komenda == "BF")  RS232_str_BF(inputRS232_dana);  
       if (inputRS232_komenda == "PC")  RS232_str_programator_czasowy(inputRS232_dana);
       if (inputRS232_komenda == "PC1") ProgCzas_ON_OFF(inputRS232_dana);
       if (inputRS232_komenda == "PV")  RS232_str_program_czasowy();
       if (inputRS232_komenda == "ZE")  RS232_RTC(inputRS232_dana);
       
     inputRS232_String = "";
     Rs232_string_OK = false;
   }
}
//-----------------------------------------------------------
//----- RS232 - POMOC wyświetlenie komend urzadzenia
void RS232_str_pomoc() {
  Serial.println();

    Serial.print("(c) "); Serial.print(TERMOSTAT_ver); Serial.println(" - regulator temperatury wody w bojlerze");
  Serial.println();
  Serial.println("  <ENTER> - podaje ustawienia parametrów modułu");
  Serial.println("  T - (ON/OFF) działanie funkcji termostatu na terminalu");

  Serial.println("  H - histereza, ustawianie H <wartość> np H 1.0");
 
  Serial.println("  B - Temp. bojlera, ustawianie B <wartość> np B 55");
  Serial.println("  S - Nazwa WiFi SSID, ustawianie A <wartość> np S BlynkWiFi");
  Serial.println("  L - ON/OFF logo PS, ustawianie L <wartość> np L 0");
  Serial.println("  ID - Blynk BLYNK_TEMPLATE_ID, ID <wartość> np ID TMPLQn1rj3zE");
  Serial.println("  NA - Blynk BLYNK_TEMPLATE_NAME, NA <wartość> np NA Termostat CWU");
  Serial.println("  TO - Blynk BLYNK_AUTH_TOKEN, TO <wartość> np TO  gYoXzJS2nsqyvb97yTylaZ8TRbpJPXDR");
  Serial.println("  BF - Blynk zezwolenie na uruchonienie, BF <wartość> np BF 1 lub 0");
  Serial.println("  PC - PROGRAMATOR CZASOWY, ustawianie PC x ON GG:MM GG:MM PWSCPSN");
  Serial.println("  PV - PROGRAMATOR CZASOWY, podgląd działania funkcji");
  Serial.println("  ZE - ZEGAR ustawienie: ZE xx:xx");
  
  Serial.println();
}
//-----------------------------------------------------------
//----- RS232 - parametry modułu na RS232 -------------------
void RS232_str_ustaw() {
  Serial.println();
  Serial.print("(c) "); Serial.print(TERMOSTAT_ver); Serial.println(" - regulator temperatury wody w bojlerze");
  Serial.println();
  Serial.print("   BLYNK_TEMPLATE_ID:   "); Serial.println(BLYNK_TEMPLATE_ID);
  Serial.print("   BLYNK_TEMPLATE_NAME: "); Serial.println(BLYNK_TEMPLATE_NAME);
  Serial.print("   BLYNK_AUTH_TOKEN:    "); Serial.println(BLYNK_AUTH_TOKEN);
  
  Serial.print("   WiFi SSID:  "); Serial.println(ssid);
  Serial.print("   WiFi pass:  "); Serial.println("nie podam :)");
  Serial.print("   WiFi IP:    "); Serial.println(WiFi.localIP());
  
  Serial.print("   DS18B20 szukam..."); Serial.print(" znalezione: "); Serial.println(sensors.getDeviceCount(), DEC);
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("  DS18B20 Unable to find address for Device 0"); 
  Serial.print("   DS18B20 adress:          "); printAddress(insideThermometer); Serial.println();
  Serial.print("   DS18B20 tryb pasożyt:    "); if (sensors.isParasitePowerMode()) Serial.println("ON"); else Serial.println("OFF");
  Serial.print("   DS18B20 rozdzielczość:   "); Serial.println(sensors.getResolution(insideThermometer), DEC);
  sensors.requestTemperatures(); Temp_odczyt_pomiaru(insideThermometer); 
  Serial.print("   DS18B20 temperatura:     "); Serial.println(float(BojlerTmpAukualna) /10 ,1);
  
  Serial.print("   TERMOSTAT:         "); if (Termostat_on_off) Serial.println("ON"); else Serial.println("OFF");
  Serial.print("   T ZADANA:          "); Serial.println(float(BojlerTmpZadana) /10 ,1);
  Serial.print("   HISTEREZA:         "); Serial.println(float(BojlerTmpHistereza) /10 ,1);
  Serial.print("   GRZAŁKA:           "); if (BojlerGrzalkaStan) Serial.println("ON"); else Serial.println("OFF");
  Serial.print("   BOJLER Tmax:       "); Serial.println(float(BojlerTmpMax) /10 ,1);
 
  Serial.println();  
}

//-----------------------------------------------------------
//--- RS232 - ON/OFF podglądu funkcji PROGRAMATOR CZASOWY ---
void RS232_str_program_czasowy() {
  RS232_flaga_program_czasowy = !RS232_flaga_program_czasowy;
  Serial.print("  PROGRAMATOR CZASOWY: podgląd działania funkcji: ");
  if (RS232_flaga_program_czasowy) Serial.println("ON");
  else Serial.println("OFF");
}

//-----------------------------------------------------------
//----- RS232 - ON/OFF podglądu funkcji TERMOSTAT -----------
void RS232_str_termo() {
  RS232_flaga_Termostat = !RS232_flaga_Termostat;
  Serial.print("  TERMOSTAT: podgląd działania funkcji termostatu: ");
  if (RS232_flaga_Termostat) Serial.println("ON");
  else Serial.println("OFF");
}

//-----------------------------------------------------------
//----- RS232 - Uruchamia lub blokuje BLYNK -----------------
void RS232_str_BF(String Blynk_BF) {
   if (Blynk_BF != ""){              //Czy podane zostało nowe hasło
    
   if (Blynk_BF == "1" || Blynk_BF == "0")  //TAK
      {
        char EE_Zapis_Dana = Blynk_BF[0];
        EE_Zapis_Dana -= 48;
        EEPROM.write(EE_Adress_Blynk_flaga, EE_Zapis_Dana);       //TAK - Zapis flagi do EEPROM 
        EEPROM.commit();
        Serial.print("  BLYNK zezwolenie na uruchomienie: ustawione ");
      } else Serial.println("  BLYNK zezwolenie na uruchomienie: błąd formatu danych, podaj BF x gdzie x jest 0 lub 1");
          
    } else Serial.print("  BLYNK zezwolenie na uruchomienie: ");                    //NIE

   byte Blynk_BF_byte = EEPROM.read(EE_Adress_Blynk_flaga);     //Odczyt flagi zezwalającej na BLYNK z EEPROM
   if (Blynk_BF_byte ==1) { Serial.println("ON");
     } else Serial.println("OFF");
   
}
//-----------------------------------------------------------
//----- RS232 - zmienia lub podaje BBLYNK_TEMPLATE_ID x060---
void RS232_str_ID(String Blynk_ID) {
   if (Blynk_ID != ""){              //Czy podane zostało nowe hasło

      EE_Zapis_String(EE_Adress_BL_ID, Blynk_ID);       //TAK - Zapis hasła do EEPROM
      Serial.print("  BLYNK_TEMPLATE_ID: ustawione ");
          
    } else Serial.print("  BLYNK_TEMPLATE_ID: ");                    //NIE
         
   Blynk_ID = EE_Odczyt_String(EE_Adress_BL_ID);     //Odczythasła zapisanego w EEPROM
   
   Serial.println(Blynk_ID);
}
//-----------------------------------------------------------
//----- RS232 - zmienia lub podaje BLYNK_TEMPLATE_NAME x070---
void RS232_str_NA(String Blynk_NA) {
   if (Blynk_NA != ""){              //Czy podane zostało nowe hasło

      EE_Zapis_String(EE_Adress_BL_NA, Blynk_NA);       //TAK - Zapis hasła do EEPROM
      Serial.print("  BLYNK_TEMPLATE_NAME: ustawione ");
          
    } else Serial.print("  BLYNK_TEMPLATE_NAME: ");                    //NIE
         
   Blynk_NA = EE_Odczyt_String(EE_Adress_BL_NA);     //Odczythasła zapisanego w EEPROM
   
   Serial.println(Blynk_NA);
}
//-----------------------------------------------------------
//----- RS232 - zmienia lub podaje BLYNK_AUTH_TOKEN x090---
void RS232_str_TO(String Blynk_TO) {
   if (Blynk_TO != ""){              //Czy podane zostało nowe hasło

      EE_Zapis_String(EE_Adress_BL_TO, Blynk_TO);       //TAK - Zapis hasła do EEPROM
      Serial.print("  BLYNK_AUTH_TOKEN: ustawione ");
          
    } else Serial.print("  BLYNK_AUTH_TOKEN: ");                    //NIE
         
   Blynk_TO = EE_Odczyt_String(EE_Adress_BL_TO);     //Odczythasła zapisanego w EEPROM
   
   Serial.println(Blynk_TO);
}
//-----------------------------------------------------------
//----- RS232 - zmienia lub podaje WiFi SSID ------------
void RS232_str_SSID(String WiFi_SSID) {
   if (WiFi_SSID != ""){              //Czy podane zostało nowe hasło

      EE_Zapis_String(EE_Adress_WiFi_SSID, WiFi_SSID);       //TAK - Zapis hasła do EEPROM
      Serial.print("  WiFi SSID: ustawione ");
          
    } else Serial.print("  WiFi SSID: ");                    //NIE
         
   WiFi_SSID = EE_Odczyt_String(EE_Adress_WiFi_SSID);     //Odczythasła zapisanego w EEPROM
   for(byte i = 0; i < WiFi_SSID.length() ; i++)
        {
          ssid[i] = char(WiFi_SSID[i]);  
          if ((i+1) == WiFi_SSID.length()) ssid[i+1] = 0;      //wstaw zero na końcu ciągu char     
        }
   Serial.println(ssid);
}
//-----------------------------------------------------------
//----- RS232 - zmienia lub podaje WiFi password ------------
void RS232_str_pass(String WiFi_pass) {
   if (WiFi_pass != ""){              //Czy podane zostało nowe hasło

      EE_Zapis_String(EE_Adress_WiFi_pass, WiFi_pass);       //TAK - Zapis hasła do EEPROM
      Serial.print("  WiFi pass: ustawione ");
          
    } else Serial.print("  WiFi pass: ");                    //NIE
         
   WiFi_pass = EE_Odczyt_String(EE_Adress_WiFi_pass);     //Odczythasła zapisanego w EEPROM
   for(byte i = 0; i < WiFi_pass.length() ; i++)
        {
          pass[i] = char(WiFi_pass[i]);  
          if ((i+1) == WiFi_pass.length()) pass[i+1] = 0;      //wstaw zero na końcu ciągu char     
        }
   Serial.println(pass);
}
//-----------------------------------------------------------
//----- RS232 - Histereza -----------------------------------
void RS232_str_histereza(String histereza) {
    if (histereza != ""){                           //Czy podana została nowa histereza?

      if (histereza.length() == 3) {
       
         if (byte(histereza[0]) >= 48 && byte(histereza[0]) <= 57 ){      //Czy cyfra stopni to cyfra od 0 do 9
           BojlerTmpHistereza = ((byte(histereza[0]))-48)*10;   
            //Serial.print("  Histereza STR:"); Serial.println(histereza);
            //Serial.print("  histereza[0]:"); Serial.print(histereza[0]);Serial.print("  BojlerTmpHistereza]:"); Serial.println(BojlerTmpHistereza);
         } else Serial.println("  Histereza: błąd formatu danych, podaj H x.x");
         
         if (byte(histereza[0]) >= 48 && byte(histereza[0]) <= 57 ){      //Czy cyfra dziątych stopnia to cyfra od 0 do 9
           BojlerTmpHistereza += byte(histereza[2])-48;           
            //Serial.print("  histereza[2]:"); Serial.print(histereza[2]);Serial.print("  BojlerTmpHistereza]:"); Serial.println(BojlerTmpHistereza);
            Serial.print("  Histereza: ustawiona "); Serial.print(float(BojlerTmpHistereza) / 10, 1);Serial.println("'C");
            EEPROM.put(EE_Adress_histereza, BojlerTmpHistereza);                   //Zapis zmiennej histereza pod adres EE_Adress_histereza
            EEPROM.commit();
         } else Serial.println("  Histereza: błąd formatu danych, podaj H x.x");
       
      } else Serial.println("  Histereza: błąd formatu danych, podaj H x.x");   //zły format danych histerezy
   
    } else {
      //Serial.println();

    byte BojlerTmp_h; BojlerTmp_h = EEPROM.read(EE_Adress_histereza);  //Odczyt Histerezy z EEPROM do zmiennej tymczasowej w celu sprawdzenia czy nie jest 0
    if (BojlerTmp_h > 0 && BojlerTmp_h < 100) BojlerTmpHistereza = BojlerTmp_h; //Czy histereza jest różna od zera i nie większa od 100
    
      Serial.print("  Histereza: "); Serial.print(float(BojlerTmpHistereza) / 10, 1);Serial.println("'C");
    }
} 
//-----------------------------------------------------------
//----- RS232 - Logo PS -------------------------------------
void RS232_str_LogoPS(String LogoPS_on_off)
{
   if (LogoPS_on_off != "")       //Czy podana została wartość flagi
   {
      if (LogoPS_on_off == "1" || LogoPS_on_off == "0")  //TAK
      {
        char EE_Zapis_Dana = LogoPS_on_off[0];
        EE_Zapis_Dana -= 48;
        EEPROM.write(EE_Adress_LogoPS, EE_Zapis_Dana);       //TAK - Zapis flagi do EEPROM 
        EEPROM.commit();
      } else Serial.println("  LogoPS: błąd formatu danych, podaj L x gdzie x jest 0 lub 1");
  
   }
     RS232_flaga_logoPS = EEPROM.read(EE_Adress_LogoPS);   //Odczyt Flagi z EEPROM
     if (RS232_flaga_logoPS ==0) 
     {
      Serial.println("  LogoPS: OFF");
     } else Serial.println("  LogoPS: ON");
}

//-----------------------------------------------------------
//----- RS232 - Ustawianie temperatury w bojlerze -----------
void RS232_str_BojlerZadana(String BojlerZadana)
{
  int TBojleraZadana;
  if (BojlerZadana != "")       //Czy podana została wartość temperatury w bojlerze (String)
  {                             //TAK
    if (BojlerZadana.length() == 2)  //Czy są to dwa znaki
    {                                //TAK
        if (byte(BojlerZadana[0]) >= 48 && byte(BojlerZadana[0]) <= 57 ){      //Czy cyfra stopni to cyfra od 0 do 9
          
           TBojleraZadana = ((byte(BojlerZadana[0]))-48)*100;   
            //Serial.print("  Histereza STR:"); Serial.println(histereza);
            //Serial.print("  histereza[0]:"); Serial.print(histereza[0]);Serial.print("  BojlerTmpHistereza]:"); Serial.println(BojlerTmpHistereza);
         } else Serial.println("  Tmp. Bojlera: błąd formatu danych, podaj B xx  np: B 55");

        if (byte(BojlerZadana[1]) >= 48 && byte(BojlerZadana[1]) <= 57 ){      //Czy cyfra stopni to cyfra od 0 do 9
          
           TBojleraZadana += ((byte(BojlerZadana[1]))-48)*10;   
            //Serial.print("  Histereza STR:"); Serial.println(histereza);
            //Serial.print("  histereza[0]:"); Serial.print(histereza[0]);Serial.print("  BojlerTmpHistereza]:"); Serial.println(BojlerTmpHistereza);
           EEPROM.put(EE_Adress_T_Bojler, TBojleraZadana);  EEPROM.commit();
           Serial.print("  Temp. zadana: ustawiona ");
         } else Serial.println("  Tmp. Bojlera: błąd formatu danych, podaj B xx  np: B 55");
    
    }else Serial.println("  Tmp. Bojlera: błąd formatu danych, podaj B xx  np: B 55");
     
  } else Serial.print("  Temp. zadana: ");

    int BojlerTmpZadana_TMP;
    EEPROM.get(EE_Adress_T_Bojler, BojlerTmpZadana_TMP);

    if (BojlerTmpZadana_TMP < 700) BojlerTmpZadana = BojlerTmpZadana_TMP;
   
    Serial.println(float(BojlerTmpZadana) /10 ,0); 
}

//-----------------------------------------------------------
//----- RS232 - EEPROM odczyt zawartości --------------------
void RS232_str_EEPROM_zawartosc(String EE_komenda) {
  if (EE_komenda != "")       //Czy podana została jakaś komenda
  {
    if (EE_komenda = "CLR")   //TAK CLR wyczyszczenie pamięci EEPROM
    {
       for (int i=0; i<=255; i++) EEPROM.write(i, 255);
       EEPROM.commit();
       Serial.println("  EEPROM wyczyszczona!");
    }
  }
     
  String value_Str = "";
  for (int EE_Adress=0; EE_Adress<256; EE_Adress++)    //Odczyt 256 bajtów
  { 
    if (EE_Adress<16) Serial.print("0"); if (EE_Adress<256) Serial.print("0");  //Dodatkowe zera na RS232 dla Adresu w EEPROM (wizualne wyrównanie
    Serial.print(EE_Adress, HEX);Serial.print("  "); //Adres EEPROM na RS232s
    value_Str = "";
     for (byte kolumna=1; kolumna<=16; kolumna++)
     {
       
       byte value = EEPROM.read(EE_Adress);         //Odczyt bajtu z EEPROM
       
       if (value <16) Serial.print("0");            //Drukuj HEX i jeżeli jest to jeden znak (wartość poniżej16) to dodatkowo dodaj zero
       Serial.print(value, HEX);Serial.print(" ");
       
       if (value<32 || value == 255) { value_Str += ".";   //Składanie linijki znaków drukowanej po wartościach HEX
       } else value_Str += char(value);
       EE_Adress++;
     }
     EE_Adress--;
     Serial.print("  "); Serial.println(value_Str);
  } 
}

//-----------------------------------------------------------
//----- RS232 Programator Czasowy    ------------------------
void RS232_str_programator_czasowy(String Ustawienie_programatora){

 if (Ustawienie_programatora != ""){  //Sprawdż czy są dane do ustawienia programatora czasowego

  byte Numer_ustawnienia= char(Ustawienie_programatora[0]-48);    //Pobranie numeru 1 do 5 ustawienia czasowego
  if (Numer_ustawnienia >= 1 && Numer_ustawnienia<=5)
  {

    int EE_Adress_Programator = EE_Adress_Prog_czas-16;   //Ustawienie początkowego adresu danych w EEPROM
    EE_Adress_Programator += 16 * Numer_ustawnienia;

    String PC_komenda_OFF =""; char PC_komenda_dana;
    PC_komenda_dana = Ustawienie_programatora[2];PC_komenda_OFF = PC_komenda_dana;// Sklejanie komendy OFF
    PC_komenda_dana = Ustawienie_programatora[3];PC_komenda_OFF += PC_komenda_dana;
    PC_komenda_dana = Ustawienie_programatora[4];PC_komenda_OFF += PC_komenda_dana;
    if (PC_komenda_OFF=="OFF") {
       EEPROM.write(EE_Adress_Programator, 0); EEPROM.commit();  //Wyłączenie ustawienia
    } 
       
                                   
    PC_komenda_dana = Ustawienie_programatora[2];PC_komenda_OFF = PC_komenda_dana;  // Czy jest komenda ON Sklejanie komendy 
    PC_komenda_dana = Ustawienie_programatora[3];PC_komenda_OFF += PC_komenda_dana;
    if (PC_komenda_OFF=="ON") {
       EEPROM.write(EE_Adress_Programator, 1); EEPROM.commit();  //Włączenie ustawienia
       PC_komenda_dana = char(Ustawienie_programatora[5])-48; PC_komenda_dana *=10; //Godzina ON
       PC_komenda_dana += char(Ustawienie_programatora[6])-48;
       EEPROM.write(EE_Adress_Programator+1, PC_komenda_dana); EEPROM.commit();     
       PC_komenda_dana = char(Ustawienie_programatora[8])-48; PC_komenda_dana *=10; //Minuta ON
       PC_komenda_dana += char(Ustawienie_programatora[9])-48;
       EEPROM.write(EE_Adress_Programator+2, PC_komenda_dana); EEPROM.commit();     
       PC_komenda_dana = char(Ustawienie_programatora[11])-48; PC_komenda_dana *=10; //Godzina OFF
       PC_komenda_dana += char(Ustawienie_programatora[12])-48;
       EEPROM.write(EE_Adress_Programator+3, PC_komenda_dana); EEPROM.commit();    
       PC_komenda_dana = char(Ustawienie_programatora[14])-48; PC_komenda_dana *=10;  //Minuta OFF
       PC_komenda_dana += char(Ustawienie_programatora[15])-48;
       EEPROM.write(EE_Adress_Programator+4, PC_komenda_dana); EEPROM.commit();  

       if (Ustawienie_programatora[17] == 'P') EEPROM.write(EE_Adress_Programator+5, 1); else  EEPROM.write(EE_Adress_Programator+5, 0);
       if (Ustawienie_programatora[18] == 'W') EEPROM.write(EE_Adress_Programator+6, 1); else  EEPROM.write(EE_Adress_Programator+6, 0);
       if (Ustawienie_programatora[19] == 'S') EEPROM.write(EE_Adress_Programator+7, 1); else  EEPROM.write(EE_Adress_Programator+7, 0);
       if (Ustawienie_programatora[20] == 'C') EEPROM.write(EE_Adress_Programator+8, 1); else  EEPROM.write(EE_Adress_Programator+8, 0);
       if (Ustawienie_programatora[21] == 'K') EEPROM.write(EE_Adress_Programator+9, 1); else  EEPROM.write(EE_Adress_Programator+9, 0);
       if (Ustawienie_programatora[22] == 'S') EEPROM.write(EE_Adress_Programator+10, 1); else  EEPROM.write(EE_Adress_Programator+10, 0);
       if (Ustawienie_programatora[23] == 'N') EEPROM.write(EE_Adress_Programator+11, 1); else  EEPROM.write(EE_Adress_Programator+11, 0);
       EEPROM.commit();
    } 
   
    //Serial.print("  Numer_ustawnienia:");Serial.println(Numer_ustawnienia);
    //Serial.print("  EE_Adress_Programator:");Serial.println(EE_Adress_Programator, HEX);

    
  } else Serial.println("  Bład formatu danych");
  
     }
  
    Serial.println("  USTAWIENIA PROGRAMATORA CZASOWEGO:"); //-----------------------------------------------------------------------------
    int EE_Adress_Programator = EE_Adress_Prog_czas;   //Ustawienie początkowego adresu danych w EEPROM

    for (byte Prog_czasowy_zestaw = 1; Prog_czasowy_zestaw <=5; Prog_czasowy_zestaw++)  //Odczyt dla 5 zastawów prog.czasowego
    {
       int EE_Adress_Programator = EE_Adress_Prog_czas-16;   //Ustawienie początkowego adresu danych w EEPROM
       EE_Adress_Programator += 16 * Prog_czasowy_zestaw;
          
    for (byte i=0; i<=15; i++)
    {
      byte value = EEPROM.read(EE_Adress_Programator);   //Wartości w pamięci EEPROM nie mogą być FF jeśli tak jaest to ustawia na 0
      if (value == 255){
        EEPROM.write(EE_Adress_Programator, 0); value=0; EEPROM.commit();
      }

      if (i==0) {  //Czy programator czasowy jest włączony
        Serial.print("  "); Serial.print(Prog_czasowy_zestaw);
        if (value == 0) Serial.print(" OFF"); else Serial.print("  ON");  
      }
      
      if (i==1){
        Serial.print("  START>"); if (value<10) Serial.print("0");
        Serial.print(value, DEC); Serial.print(":");        //Godzina ON
      }
      if (i==2) {
        if (value<10) Serial.print("0");
        Serial.print(value, DEC); Serial.print("  STOP>");  //Minuta ON
      }
      if (i==3) {
        if (value<10) Serial.print("0");
        Serial.print(value, DEC); Serial.print(":");        //Godzina OFF
      }
      if (i==4) {
        if (value<10) Serial.print("0");
        Serial.print(value, DEC); Serial.print("  ");       //Minuta OFF
      }
      
      if (i==5) {
        if (value>0) Serial.print("P"); else Serial.print("_");        //Analiza dni poniedziałek
      }
      if (i==6) {
        if (value>0) Serial.print("W"); else Serial.print("_");        //Analiza dni poniedziałek
      }
      if (i==7) {
        if (value>0) Serial.print("S"); else Serial.print("_");        //Analiza dni poniedziałek
      }
      if (i==8) {
        if (value>0) Serial.print("C"); else Serial.print("_");        //Analiza dni poniedziałek
      }
      if (i==9) {
        if (value>0) Serial.print("K"); else Serial.print("_");        //Analiza dni poniedziałek
      }
      if (i==10) {
        if (value>0) Serial.print("S"); else Serial.print("_");        //Analiza dni poniedziałek
      }
      if (i==11) {
        if (value>0) Serial.print("N"); else Serial.print("_");        //Analiza dni poniedziałek
      }
      EE_Adress_Programator++;
    }    
     Serial.println();
  }
}
//-----------------------------------------------------------
//----- RS232 ZEGAR USTAWIENIE       ------------------------
void RS232_RTC(String RTC_ustawienie){

  char RTC_godzina; char RTC_minuta;
  if (RTC_ustawienie != ""){  //Sprawdż czy są dane do ustawienia programatora czasowego

    RTC_godzina = char(RTC_ustawienie[0])-48; RTC_godzina *=10;      //Godzina - ZŁOŻENIE ZE sTRING GODZINY CHAR
    RTC_godzina += char(RTC_ustawienie[1])-48;
    RTC_minuta = char(RTC_ustawienie[3])-48; RTC_minuta *=10;        //Minuta - ZŁOŻENIE ZE sTRING Minuty CHAR
    RTC_minuta += char(RTC_ustawienie[4])-48;
   
    if ( RTC_godzina < 24 && RTC_minuta <60 ) {
      rtc.set(0, RTC_minuta, RTC_godzina, 5, 12, 9, 23); rtc.refresh();      
      Serial.print("  ZEGAR->"); 
      if (rtc.hour()<10) Serial.print("0"); Serial.print(rtc.hour()); Serial.print(":"); 
      if (rtc.minute()<10) Serial.print("0"); Serial.print(rtc.minute());Serial.println(" -> USTAWIONY"); 
    } else {
      Serial.println("  ZEGAR-> błąd formatu danych"); 
      if (rtc.hour()<10) Serial.print("0"); Serial.print(rtc.hour()); Serial.print(":"); 
      if (rtc.minute()<10) Serial.print("0"); Serial.print(rtc.minute());
    }
  }else {
     Serial.print("  ZEGAR->"); 
        
     if (rtc.hour()<10) Serial.print("0"); Serial.print(rtc.hour()); Serial.print(":"); 
     if (rtc.minute()<10) Serial.print("0"); Serial.print(rtc.minute());  
  }
}
//-----------------------------------------------------------
//----- PROGRAMATOR CZASOWY      ----------------------------
//-----------------------------------------------------------
void Programator_czasowy()
{
 rtc.refresh();
 byte PC_sekundy = rtc.second();
 String PC_ustawienie_str;

 if (PC_sekundy==0 && PC_flaga_sekundy==1){             //Wykonanie sprawdzenia ustawień programatora tylko gdy sekundy=0, wykonaj sprawdzenie tylko raz dla sekundy=1
     
     if (RS232_flaga_program_czasowy) Serial.println("  Programator_czasowy:"); 
       for (byte PC_ustawienie=1; PC_ustawienie<=5; PC_ustawienie++){         
         PC_ustawienie_str = String(PC_ustawienie);
         // Serial.print(PC_ustawienie);  Serial.print(" STR:");  Serial.print(PC_ustawienie_str);
         ProgCzas_ON_OFF(PC_ustawienie_str);
         }
     PC_flaga_sekundy=0;
 }
 if (PC_sekundy!=0) PC_flaga_sekundy = 1;    //Zezwolenie na ponowne wykonanie funkcji gdy sek<>0
}
//-----------
void ProgCzas_ON_OFF(String PC_nr_ustawienia)
{
  byte PC_nr_ustawienia_DEC = char(PC_nr_ustawienia[0])-48;
  PC_nr_ustawienia_DEC--;
  
    // Serial.print("  Programator czasowy - ustawienie nr: "); Serial.println(PC_nr_ustawienia);
    // Serial.print("  PC_nr_ustawienia_DEC:"); Serial.println(PC_nr_ustawienia_DEC);
  
  int EE_Adress_Programator = EE_Adress_Prog_czas+(PC_nr_ustawienia_DEC*16);   //Ustawienie początkowego adresu danych w EEPROM

   // Serial.print("  EE_Adress_Prog_czas:"); Serial.println(EE_Adress_Programator);

   
    if (RS232_flaga_program_czasowy){
      Serial.print("  "); Serial.print(PC_nr_ustawienia);
      Serial.print("  CZAS->"); 
      if (rtc.hour()<10) Serial.print("0"); Serial.print(rtc.hour()); Serial.print(":"); 
      if (rtc.minute()<10) Serial.print("0"); Serial.print(rtc.minute());  
    }

  
  if (EEPROM.read(EE_Adress_Programator)>0)        //Czy ustawienie jest aktywne
  {
    EE_Adress_Programator++;
    byte PC_godz_ON = EEPROM.read(EE_Adress_Programator); EE_Adress_Programator++;
    byte PC_min_ON = EEPROM.read(EE_Adress_Programator); EE_Adress_Programator++;
    byte PC_godz_OFF = EEPROM.read(EE_Adress_Programator); EE_Adress_Programator++;
    byte PC_min_OFF = EEPROM.read(EE_Adress_Programator); EE_Adress_Programator++;


    //rtc.refresh();
    int PC_zegar = (rtc.hour()*256)+rtc.minute();   //Aktualny czas do zmiennej Int
    int PC_czas_ON = PC_godz_ON * 256 + PC_min_ON; 
    int PC_czas_OFF = PC_godz_OFF * 256 + PC_min_OFF; 
    
     
   // if (PC_zegar>=PC_czas_ON && PC_zegar <= PC_czas_OFF) Serial.println(" -> TERMOREGULACJA ON"); else Serial.println(" -> TERMOREGULACJA OFF"); //PRZEDZIAŁ CZASOWY TYLKO PO URUCHOMIENIU URZĄDZENIA

   if (RS232_flaga_program_czasowy){
    Serial.print("  ON->"); if (PC_godz_ON < 10) Serial.print("0"); Serial.print(PC_godz_ON); Serial.print(":"); if (PC_min_ON < 10) Serial.print("0");Serial.print(PC_min_ON);
    Serial.print("  OFF->"); if (PC_godz_OFF < 10) Serial.print("0"); Serial.print(PC_godz_OFF); Serial.print(":"); if (PC_min_OFF < 10) Serial.print("0"); Serial.print(PC_min_OFF);
   }
    
    if (PC_zegar == PC_czas_ON)  {
      Termostat_on_off= 1;  //Wyłłacz funkcję termostatu
      //BojlerGrzalkaStan=0;//Wyłącz grzałkę jeśl wyłączono termoregulację
      //Grzalka_ON_OFF();
      if (RS232_flaga_program_czasowy) Serial.print(" -> TERMOREGULACJA ON");
      }
    
    
    if (PC_zegar == PC_czas_OFF)  {
      Termostat_on_off= 0;  //Wyłłacz funkcję termostatu
      BojlerGrzalkaStan=0;//Wyłącz grzałkę jeśl wyłączono termoregulację
      Grzalka_ON_OFF();
      if (RS232_flaga_program_czasowy) Serial.print(" -> TERMOREGULACJA OFF");
      } 
      if (RS232_flaga_program_czasowy) Serial.println();
   } else if (RS232_flaga_program_czasowy) Serial.println("  --- OFF ---");   
}

//-----------------------------------------------------------
//----- EEPROM zapis STRING do EEPROM    --------------------
void EE_Zapis_String(int EE_Adress, String EE_String)
{
  for (byte i = 0; i < EE_String.length() ; i++)    //opbranie długości string  
  {
  byte EE_Zapis_Dana = EE_String[i];               //Zapis pojedyńczych bajtów
  EEPROM.write(EE_Adress, EE_Zapis_Dana);
  EE_Adress++;
  }
  char EE_Zapis_Dana=0; EEPROM.write(EE_Adress, EE_Zapis_Dana);  //Dopisuje do EEPROM 0 jako koniec String
  EEPROM.commit();
}
//-----------------------------------------------------------
//----- EEPROM odczyt STRING z EEPROM    --------------------
String EE_Odczyt_String(int EE_Adress)
{
 String EE_Odczytany_Str = ""; char EE_Dana;     //Deklaracja zmiennej String dla funkcji
 do {
   EE_Dana = EEPROM.read(EE_Adress);      //Odczyt bajtu z EEPROM
   EE_Odczytany_Str += EE_Dana; 
    // Serial.print("EE_Adress:"); Serial.print(EE_Adress, HEX); Serial.print(" EE_Dana:"); Serial.print(EE_Dana); Serial.print(" EE_Odczytany_Str:"); Serial.println(EE_Odczytany_Str);
   EE_Adress++;
 }while (EE_Dana != 0);        //Odczytuj String dopuki nie trafisz na 0
 
 return EE_Odczytany_Str;
}
