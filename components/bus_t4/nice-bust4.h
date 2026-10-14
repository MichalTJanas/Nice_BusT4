/*
  Nice BusT4
  Wymiana danych przez UART z prędkością 19200 8n1
  Przed pakietem z danymi wysyłany jest break o długości 519us (10 bitów)
  Zawartość pakietu, którą udało się rozszyfrować, opisano w strukturze packet_cmd_body_t

 

  Dla Oview do adresu zawsze dodawane jest 80.
  Adres sterownika bramy bez zmian.


Podłączenie

BusT4                       ESP8266

Ścianka urządzenia        Rx Tx GND
9  7  5  3  1  
10 8  6  4  2
miejsce na kabel
            1 ---------- Rx
            2 ---------- GND
            4 ---------- Tx
            5 ---------- +24V




Z instrukcji nice_dmbm_integration_protocol.pdf

• ADR: to adres sieci NICE, w której znajdują się urządzenia, którymi chcesz sterować. Może to być wartość od 1 do 63 (od 1 do 3F).
Ta wartość musi być w HEX. Jeśli adresatem jest moduł integracyjny na szynie DIN, wartość ta wynosi 0 (adr = 0), jeśli adresatem
jest inteligentny silnik, wartość ta wynosi 1 (adr = 1).
• EPT: to adres silnika Nice należącego do sieci ADR. Może to być wartość od 1 do 127. Ta wartość musi być w HEX.
• CMD: to komenda, którą chcesz wysłać do adresata (ADR, EPT).
• PRF: komenda ustawienia profilu.
• FNC: to funkcja, którą chcesz wysłać do adresata (ADR, EPT).
• EVT: to zdarzenie, które jest wysyłane do adresata (ADR, EPT).



*/


#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"           // do dodawania Action
#include "esphome/components/cover/cover.h"
#include <HardwareSerial.h>
#include "esphome/core/helpers.h"              // parsujemy ciągi wbudowanymi narzędziami
#include <queue>                               // do obsługi kolejki



namespace esphome {
namespace bus_t4 {

/* dla krótkiego odwoływania się do członków klasy */
using namespace esphome::cover;
//using esp8266::timeoutTemplate::oneShotMs;


static const int _UART_NO=UART0; /* numer uart */
static const int TX_P = 1;         /* pin Tx */
static const uint32_t BAUD_BREAK = 9200; /* baudrate dla długiego impulsu przed pakietem */
static const uint32_t BAUD_WORK = 19200; /* roboczy baudrate */
static const uint8_t START_CODE = 0x55; /*bajt startowy pakietu */

static const float CLOSED_POSITION_THRESHOLD = 0.007;  // Wartość położenia napędu w procentach, poniżej której brama jest uznawana za całkowicie zamkniętą
static const uint32_t POSITION_UPDATE_INTERVAL = 500;  // Interwał aktualizacji bieżącego położenia napędu, ms

/* ustawienia sieciowe esp
  Seria może przyjmować wartości od 0 do 63, domyślnie 0
  Adres OVIEW zaczyna się od 8

  Przy łączeniu w sieć kilku napędów z OXI należy dla różnych napędów ustawić różne serie.
  W takim przypadku OXI musi mieć taką samą serię jak napęd, którym steruje.
*/






/* Typ komunikatu pakietów
  na razie interesują nas tylko CMD i INF
  pozostałych nie badałem dokładnie i numerów nie sprawdzałem
  6. bajt pakietów CMD i INF
*/
enum mes_type : uint8_t {
  CMD = 0x01,  /* numer sprawdzony, wysyłanie komend do automatyki */
//  LSC = 0x02,  /* obsługa list scenariuszy */
//  LST = 0x03,  /* obsługa list automatyk */
//  POS = 0x04,  /* odczyt i zmiana pozycji automatyki */
//  GRP = 0x05,  /* wysyłanie komend do grupy automatyk z podaniem maski bitowej silnika */
//  SCN = 0x06,  /* obsługa scenariuszy */
//  GRC = 0x07,  /* wysyłanie komend do grupy automatyk utworzonych w Nice Screen Configuration Tool */
  INF = 0x08,  /* zwraca lub ustawia informacje o urządzeniu */
//  LGR = 0x09,  /* obsługa list grup */
//  CGR = 0x0A,  /* obsługa kategorii grup utworzonych w Nice Screen Configuration Tool */
};




/* 
menu komendy w hierarchii oview
9. bajt pakietów CMD
*/
enum cmd_mnu  : uint8_t {
  CONTROL = 0x01,
};


/* używane w odpowiedziach STA*/
enum sub_run_cmd2 : uint8_t {
  STA_OPENING = 0x02,
  STA_CLOSING = 0x03,
       OPENED = 0x04,
       CLOSED = 0x05,
      ENDTIME = 0x06,  // manewr zakończony po przekroczeniu czasu
      STOPPED = 0x08,
  PART_OPENED = 0x10,  // częściowe otwarcie
};

/* Błędy */
enum errors_byte  : uint8_t {
  NOERR = 0x00, // Brak błędów
  FD = 0xFD,    // Brak komendy dla tego urządzenia
  };

// Typy silników
enum motor_type  : uint8_t {
  SLIDING = 0x01, 
  SECTIONAL = 0x02,
  SWING = 0x03,
  BARRIER = 0x04,
  UPANDOVER = 0x05, // up-and-over brama uchylna
  };

//  dziewiąty bajt
enum whose_pkt  : uint8_t {
  FOR_ALL = 0x00,  /* pakiet dla/od wszystkich */
  FOR_CU = 0x04,  /* pakiet dla/od centrali sterującej */
  FOR_OXI = 0x0A,  /* pakiet dla/od odbiornika OXI */
  };
	
// dziesiąty bajt GET/SET pakietów EVT, dla pakietów CMD występowała tylko wartość RUN
enum command_pkt  : uint8_t {
  TYPE_M         = 0x00,   /* Zapytanie o typ napędu */
  INF_STATUS     = 0x01, //	Stan bramy (Otwarta/Zamknięta/Zatrzymana)	
  WHO	         = 0x04,  /* Kto jest w sieci? */
  MAC            = 0x07,    // mac address.
  MAN            = 0x08,   // manufacturer.
  PRD            = 0x09,   // product.
  INF_SUPPORT    = 0x10, //  Dostępne komendy INF
  HWR            = 0x0a,   // hardware version.
  FRM            = 0x0b,   // firmware version.
  DSC            = 0x0c,   // description.
  CUR_POS        = 0x11,  // bieżące umowne położenie automatyki, DPRO924 czeka potem na ustawienie położeń
  MAX_OPN        = 0x12,   // Maksymalne możliwe otwarcie wg enkodera.
  POS_MAX        = 0x18,   // Maksymalne położenie (otwarcia) wg enkodera
  POS_MIN        = 0x19,   // Minimalne położenie (zamknięcia) wg enkodera	
  INF_P_OPN1     = 0x21, //	Częściowe otwarcie1 
  INF_P_OPN2     = 0x22, //	Częściowe otwarcie2
  INF_P_OPN3     = 0x23, //	Częściowe otwarcie3
  INF_SLOW_OPN   = 0x24, // Zwolnienie przy otwieraniu
  INF_SLOW_CLS   = 0x25, // Zwolnienie przy zamykaniu	
  OPN_OFFSET     = 0x28, /* Opóźnienie otwierania  open offset */
  CLS_OFFSET     = 0x29, /* Opóźnienie zamykania  close offset */
  OPN_DIS        = 0x2a, /* Parametry podstawowe - Odciążenie przy otwarciu Open discharge */
  CLS_DIS        = 0x2b, /* Parametry podstawowe - Odciążenie przy zamknięciu Close discharge */
  REV_TIME       = 0x31, /* Parametry podstawowe - Czas trwania rewersu (Brief inversion value) */
  OPN_PWR        = 0x4A,    /* Parametry podstawowe - Regulacja siły - Siła otwierania */	  	  	  	  	  
  CLS_PWR        = 0x4B,    /* Parametry podstawowe - Regulacja siły - Siła zamykania */	  	  	  	  	  	  
  SPEED_OPN      = 0x42,    /* Parametry podstawowe - Ustawienie prędkości - Prędkość otwierania */	  	  	  	  	  	  	  
  SPEED_CLS      = 0x43,    /* Parametry podstawowe - Ustawienie prędkości - Prędkość zamykania */	  
  SPEED_SLW_OPN  = 0x45,    /* Parametry podstawowe - Ustawienie prędkości - Prędkość zwolnionego otwierania */	
  SPEED_SLW_CLS  = 0x46,    /* Parametry podstawowe - Ustawienie prędkości - Prędkość zwolnionego zamykania */	
  OUT1           = 0x51,  /* Konfiguracja wyjść */	  
  OUT2           = 0x52,  /* Konfiguracja wyjść */	  	  
  LOCK_TIME      = 0x5A,  /* Konfiguracja wyjść - Czas pracy zamka */
  S_CUP_TIME     = 0x5C,  /* Konfiguracja wyjść - Czas pracy przyssawki Suction Cup Time*/	  
  LAMP_TIME      = 0x5B,  /* Konfiguracja wyjść - Czas pracy lampy oświetleniowej courtesy light Time*/
  COMM_SBS       = 0x61,  /* Konfiguracja komend - Krok po kroku */	  
  COMM_POPN      = 0x62,  /* Konfiguracja komend - Otwórz częściowo */	  	  
  COMM_OPN       = 0x63,  /* Konfiguracja komend - Otwórz */	  	  	  
  COMM_CLS       = 0x64,  /* Konfiguracja komend - Zamknij */	  
  COMM_STP       = 0x65,  /* Konfiguracja komend - STOP */		  
  COMM_PHOTO     = 0x68,  /* Konfiguracja komend - Foto */		  
  COMM_PHOTO2    = 0x69,  /* Konfiguracja komend - Foto2 */
  COMM_PHOTO3    = 0x6A,  /* Konfiguracja komend - Foto3 */
  COMM_OPN_STP   = 0x6B,  /* Konfiguracja komend - Stop przy otwieraniu */	  
  COMM_CLS_STP   = 0x6C,  /* Konfiguracja komend - Stop przy zamykaniu */	 
  IN1            = 0x71,  /* Konfiguracja wejść */
  IN2            = 0x72,  /* Konfiguracja wejść */
  IN3            = 0x73,  /* Konfiguracja wejść */
  IN4            = 0x74,  /* Konfiguracja wejść */
  COMM_LET_OPN   = 0x78,  /* Konfiguracja komend - Przeszkoda przy otwieraniu */	  	  	  
  COMM_LET_CLS   = 0x79,  /* Konfiguracja komend - Przeszkoda przy zamykaniu */	  	  	  	  

  AUTOCLS        = 0x80,    /* Parametry podstawowe - Automatyczne zamykanie */
  P_TIME         = 0x81,    /* Parametry podstawowe - Czas pauzy */
  PH_CLS_ON      = 0x84,    /* Parametry podstawowe - Zamknij po Foto - Aktywne */	  
  PH_CLS_VAR     = 0x86,    /* Parametry podstawowe - Zamknij po Foto - Tryb */	  	  
  PH_CLS_TIME    = 0x85,    /* Parametry podstawowe - Zamknij po Foto - Czas oczekiwania */	  	  	  
  ALW_CLS_ON     = 0x88,    /* Parametry podstawowe - Zawsze zamykaj - Aktywne */	  	  
  ALW_CLS_VAR    = 0x8A,    /* Parametry podstawowe - Zawsze zamykaj - Tryb */	  
  ALW_CLS_TIME   = 0x89,    /* Parametry podstawowe - Zawsze zamykaj - Czas oczekiwania */	  	  	  
  STAND_BY_ACT   = 0x8c,    /* Parametry podstawowe - Tryb czuwania - Aktywne  ON / OFF */
  WAIT_TIME      = 0x8d,    /* Parametry podstawowe - Tryb czuwania - Czas oczekiwania */
  STAND_BY_MODE  = 0x8e,    /* Parametry podstawowe - Tryb czuwania - Tryb -  safety = 0x00, bluebus=0x01, all=0x02*/
  START_ON       = 0x90,    /* Parametry podstawowe - Ustawienie rozruchu - Aktywne */		  	  
  START_TIME     = 0x91,    /* Parametry podstawowe - Ustawienie rozruchu - Czas rozruchu */		  	  	  
  SLOW_ON        = 0xA2,    /* Parametry podstawowe - Zwolnienie */	
  DIS_VAL        = 0xA4,    /* Położenie - Wartość niedozwolona disable value */

  BLINK_ON       = 0x94,    /* Parametry podstawowe - Wstępne miganie - Aktywne */		  	  	  	  
  BLINK_OPN_TIME = 0x95,    /* Parametry podstawowe - Wstępne miganie - Czas przy otwieraniu */		  	  	  	  	  
  BLINK_CLS_TIME = 0x99,    /* Parametry podstawowe - Wstępne miganie - Czas przy zamykaniu */
  OP_BLOCK       = 0x9a,    /* Parametry podstawowe - Blokada silnika (Operator block)*/
  KEY_LOCK       = 0x9c,    /* Parametry podstawowe - Blokada przycisków */
  T_VAL          = 0xB1,    /*Alarm threshold value Próg do serwisu w liczbie manewrów*/
  P_COUNT        = 0xB2,    /* Partial count Licznik częściowy*/
  C_MAIN         = 0xB4,    /* Cancel maintenance Anulowanie serwisu */
  DIAG_BB        = 0xD0,     /*   DIAGNOSTICS of bluebus devices */  
  INF_IO         = 0xD1,    /*	stan wejść-wyjść	*/
  DIAG_PAR       = 0xD2,    /*  DIAGNOSTICS of other parameters   */
  
  
  
  


  

  CUR_MAN = 0x02,  // Bieżący manewr
  SUBMNU  = 0x04,  // Podmenu
  STA = 0xC0,   // status w ruchu
  MAIN_SET = 0x80,   // Parametry podstawowe
  RUN = 0x82,   // Komenda do wykonania	  

  };	

	
/* run cmd 11. bajt pakietów EVT */
enum run_cmd  : uint8_t {
  SET = 0xA9,  /* żądanie zmiany parametrów */
  GET = 0x99,   /* żądanie odczytu parametrów */
  GET_SUPP_CMD = 0x89, /* pobierz obsługiwane komendy */
  };


/* Komenda, która ma zostać wykonana.   
11. bajt pakietu CMD
Używana w zapytaniach i odpowiedziach */
enum control_cmd : uint8_t { 
  SBS = 0x01,    /* Step by Step */
  STOP = 0x02,   /* Stop */
  OPEN = 0x03,   /* Open */
  CLOSE = 0x04,  /* Close */
  P_OPN1 = 0x05, /* Partial opening 1 - częściowe otwarcie, tryb furtki */
  P_OPN2 = 0x06, /* Partial opening 2 */
  P_OPN3 = 0x07, /* Partial opening 3 */
  RSP = 0x19, /* odpowiedź interfejsu potwierdzająca otrzymanie komendy  */
  EVT = 0x29, /* odpowiedź interfejsu przesyłająca żądane informacje */
 
  P_OPN4 = 0x0b, /* Partial opening 4 - Zbiorczo */
  P_OPN5 = 0x0c, /* Partial opening 5 - Priorytet krok po kroku */
  P_OPN6 = 0x0d, /* Partial opening 6 - Otwórz i zablokuj */
  UNLK_OPN = 0x19, /* Odblokuj i otwórz */
  CLS_LOCK = 0x0E, /* Zamknij i zablokuj */
  UNLCK_CLS = 0x1A, /*  Odblokuj i zamknij */
  LOCK = 0x0F, /* Zablokuj*/
  UNLOCK = 0x10, /* Odblokuj */
  LIGHT_TIMER = 0x11, /* Timer oświetlenia */
  LIGHT_SW = 0x12, /* Oświetlenie wł./wył. */
  HOST_SBS = 0x13, /* Master SBS */
  HOST_OPN = 0x14, /* Master otwórz */
  HOST_CLS = 0x15, /* Master zamknij */
  SLAVE_SBS = 0x16, /*  Slave SBS */
  SLAVE_OPN = 0x17, /* Slave otwórz */
  SLAVE_CLS = 0x18, /* Slave zamknij */
  AUTO_ON = 0x1B, /* Automatyczne otwieranie aktywne */
  AUTO_OFF = 0x1C, /* Automatyczne otwieranie nieaktywne	  */
  
};
	
	
	
	
	
	
/* Informacje ułatwiające zrozumienie budowy pakietów w protokole */
// treść pakietu zapytania CMD
// pakiety z rozmiarem treści 0x0c=12 bajtów 
	/*
struct packet_cmd_body_t {
  uint8_t byte_55;              // Nagłówek, zawsze 0x55
  uint8_t pct_size1;                // rozmiar treści pakietu (bez nagłówka i CRC. Łączna liczba bajtów minus trzy), dla komend = 0x0c
  uint8_t for_series;           // seria adresata pakietu ff = wszyscy
  uint8_t for_address;          // adres adresata pakietu ff = wszyscy
  uint8_t from_series;           // seria nadawcy pakietu
  uint8_t from_address;          // adres nadawcy pakietu
  uint8_t mes_type;           // typ komunikatu, 1 = CMD, 8 = INF
  uint8_t mes_size;              // liczba dalszych bajtów pomniejszona o dwa bajty CRC na końcu, dla komend = 5
  uint8_t crc1;                // CRC1, XOR sześciu poprzednich bajtów
  uint8_t cmd_mnu;                // Menu komendy. cmd_mnu = 1 dla komend sterujących
  uint8_t setup_submnu;            // Podmenu, w połączeniu z grupą komendy określa typ wysyłanego komunikatu
  uint8_t control_cmd;            // Komenda, która ma zostać wykonana
  uint8_t offset;            //  Przesunięcie dla odpowiedzi. Wpływa na zapytania typu lista obsługiwanych komend
  uint8_t crc2;            // crc2, XOR czterech poprzednich bajtów
  uint8_t pct_size2;            // rozmiar treści pakietu (bez nagłówka i CRC. Łączna liczba bajtów minus trzy), dla komend = 0x0c

};





// treść pakietu odpowiedzi RSP
// pakiety z rozmiarem treści 0x0e=14 bajtów 
struct packet_rsp_body_t {
  uint8_t byte_55;              // Nagłówek, zawsze 0x55
  uint8_t pct_size1;                // rozmiar treści pakietu (bez nagłówka i CRC. Łączna liczba bajtów minus trzy), >= 0x0e
  uint8_t to_series;           // seria adresata pakietu ff = wszyscy
  uint8_t to_address;          // adres adresata pakietu ff = wszyscy
  uint8_t from_series;           // seria nadawcy pakietu
  uint8_t from_address;          // adres nadawcy pakietu
  uint8_t mes_type;           // typ komunikatu, dla tych pakietów zawsze  8 = INF
  uint8_t mes_size;              // liczba dalszych bajtów pomniejszona o dwa bajty CRC na końcu, dla komend = 5
  uint8_t crc1;                // CRC1, XOR sześciu poprzednich bajtów
  uint8_t cmd_mnu;                // Menu komendy. cmd_mnu = 1 dla komend sterujących
  uint8_t sub_inf_cmd;            // Z którego podmenu otrzymano komendę. Wartość mniejsza o 0x80 niż pierwotne podmenu
  uint8_t sub_run_cmd;            // Jaką komendę otrzymano. Wartość większa o 0x80 niż otrzymana komenda
  uint8_t hb_data;             // dane, starszy bit
  uint8_t lb_data;            // dane, młodszy bit
  uint8_t err;               // Błędy
  uint8_t crc2;            // crc2, XOR czterech poprzednich bajtów
  uint8_t pct_size2;            // rozmiar treści pakietu (bez nagłówka i CRC. Łączna liczba bajtów minus trzy), >= 0x0e

};
	
 // treść pakietu odpowiedzi z danymi EVT
 
 struct packet_evt_body_t {
  uint8_t byte_55;              // Nagłówek, zawsze 0x55
  uint8_t pct_size1;                // rozmiar treści pakietu (bez nagłówka i CRC. Łączna liczba bajtów minus trzy), >= 0x0e
  uint8_t to_series;           // seria adresata pakietu ff = wszyscy
  uint8_t to_address;          // adres adresata pakietu ff = wszyscy
  uint8_t from_series;           // seria nadawcy pakietu
  uint8_t from_address;          // adres nadawcy pakietu
  uint8_t mes_type;           // typ komunikatu, dla tych pakietów zawsze  8 = INF
  uint8_t mes_size;              // liczba dalszych bajtów pomniejszona o dwa bajty CRC na końcu, dla komend = 5
  uint8_t crc1;                // CRC1, XOR sześciu poprzednich bajtów
  uint8_t whose;                // Czyj pakiet. Warianty: 00 - ogólny, 04 - sterownika napędu, 0A - odbiornika OXI
  uint8_t setup_submnu;            // Z którego podmenu otrzymano komendę. Wartość równa pierwotnemu podmenu
  uint8_t sub_run_cmd;            // Na jaką komendę odpowiadamy. Wartość mniejsza o 0x80 niż wcześniej wysłana komenda
  uint8_t next_data;            // Następny blok danych
  uint8_t err;               // Błędy
  uint8_t data_blk;            // Blok danych, może zajmować kilka bajtów
  uint8_t crc2;            // crc2, XOR wszystkich poprzednich bajtów do dziewiątego (Czyj pakiet)
  uint8_t pct_size2;            // rozmiar treści pakietu (bez nagłówka i CRC. Łączna liczba bajtów minus trzy), >= 0x0e

};
 
 
*/

enum position_hook_type : uint8_t {
     IGNORE = 0x00,
    STOP_UP = 0x01,
  STOP_DOWN = 0x02
 };

// tworzę klasę, dziedziczę członków klas Component i Cover
class NiceBusT4 : public Component, public Cover {
  public:
	
    // ustawienia napędu
    bool autocls_flag; // Automatyczne zamykanie - L1
    bool photocls_flag; // Zamknij po foto - L2
    bool alwayscls_flag; // Zawsze zamykaj - L3
    bool init_ok = false; //  wykrywanie napędu przy włączeniu
    bool is_walky = false; // dla walky inna jest komenda zapytania o położenie
    bool is_robus = false; // dla robus nie trzeba okresowo odpytywać o pozycję
    bool is_ro = false; // dla ro600 inny jest pakiet ze statusem pozycji i statusem ruchu
		
    void setup() override;
    void loop() override;
    void dump_config() override; // do wypisania w logu informacji o sprzęcie

    void send_raw_cmd(std::string data);
    void send_cmd(uint8_t data) {this->tx_buffer_.push(gen_control_cmd(data));}	
    void send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command,  std::string next_data, bool data_on, std::string data_command); // długa komenda
    void set_mcu(std::string command, std::string data_command); // komenda dla sterownika silnika
		

    void set_class_gate(uint8_t class_gate) { class_gate_ = class_gate; }
    
 /*   void set_update_interval(uint32_t update_interval) {  // interwał pobierania statusu napędu
      this->update_interval_ = update_interval;
    }*/

    cover::CoverTraits get_traits() override;

  protected:
    void control(const cover::CoverCall &call) override;
    void send_command_(const uint8_t *data, uint8_t len);
    void request_position(void);  // Zapytanie o umowne bieżące położenie napędu
    void update_position(uint16_t newpos);  // Aktualizacja bieżącego położenia napędu

    uint32_t last_position_time{0};  // Czas ostatniej aktualizacji bieżącego położenia
    uint32_t update_interval_{500};
    uint32_t last_update_{0};
    uint32_t last_uart_byte_{0};

    CoverOperation last_published_op;  // Ostatnio opublikowany stan i położenie
    float last_published_pos{-1};

    void publish_state_if_changed(void);

    uint8_t position_hook_type{IGNORE};  // Flaga i pozycja ustawienia zadanego położenia napędu
    uint16_t position_hook_value;

    uint8_t class_gate_ = 0x55; // 0x01 sliding, 0x02 sectional, 0x03 swing, 0x04 barrier, 0x05 up-and-over
//    uint8_t last_init_command_;
	
    bool init_cu_flag = false;	
    bool init_oxi_flag = false;	

	
    // zmienne dla uart
    uint8_t _uart_nr;
    uart_t* _uart = nullptr;
    uint16_t _max_opn = 0;  // maksymalna pozycja enkodera lub timera
    uint16_t _pos_opn = 2048;  // pozycja otwarcia enkodera lub timera, nie dla wszystkich napędów.
    uint16_t _pos_cls = 0;  // pozycja zamknięcia enkodera lub timera, nie dla wszystkich napędów
    uint16_t _pos_usl = 0;  // umowna bieżąca pozycja enkodera lub timera, nie dla wszystkich napędów	
    // ustawienia nagłówka tworzonego pakietu
    uint8_t addr_from[2] = {0x00, 0x66}; //nadawca pakietu, adres bramki bust4
    uint8_t addr_to[2]; // = 0x00ff;	 // adresat pakietu, adres sterownika napędu, którym sterujemy
    uint8_t addr_oxi[2]; // = 0x000a;	 // adres odbiornika

    std::vector<uint8_t> raw_cmd_prepare (std::string data);             // przygotowanie danych wprowadzonych przez użytkownika do wysłania	
	
    // generowanie komend inf
    std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len);	 // wszystkie pola
    std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd) {return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, 0x00, {0x00}, 0 );} // dla komend bez danych
    std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, std::vector<uint8_t> data){
	    return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, next_data, data, data.size());} // dla komend z danymi
    std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data){
	    return gen_inf_cmd(to_addr1, to_addr2, whose, inf_cmd, run_cmd, next_data, {0x00}, 0);} // dla komend z adresem i bez danych 	
    	    
    // generowanie komend cmd
    std::vector<uint8_t> gen_control_cmd(const uint8_t control_cmd);	    	
	
    void init_device (const uint8_t addr1, const uint8_t addr2, const uint8_t device );
    void send_array_cmd (std::vector<uint8_t> data);	
    void send_array_cmd (const uint8_t *data, size_t len);


    void parse_status_packet (const std::vector<uint8_t> &data); // parsujemy pakiet statusu
    
    void handle_char_(uint8_t c);                                         // obsługa odebranego bajtu
    void handle_datapoint_(const uint8_t *buffer, size_t len);          // obsługa odebranych danych
    bool validate_message_();                                         // funkcja weryfikacji odebranego komunikatu

    std::vector<uint8_t> rx_message_;                          // tu bajt po bajcie gromadzony jest odebrany komunikat
    std::queue<std::vector<uint8_t>> tx_buffer_;             // kolejka komend do wysłania	
    bool ready_to_tx_{true};	                           // flaga gotowości do wysyłania komend
	
    std::vector<uint8_t> manufacturer_ = {0x55, 0x55};  // przy inicjalizacji nieznany producent
    std::vector<uint8_t> product_;
    std::vector<uint8_t> hardware_;
    std::vector<uint8_t> firmware_;
    std::vector<uint8_t> description_;	
    std::vector<uint8_t> oxi_product;
    std::vector<uint8_t> oxi_hardware;
    std::vector<uint8_t> oxi_firmware;
    std::vector<uint8_t> oxi_description;	

}; //klasa

} // namespace bus_t4
} // namespace esphome
