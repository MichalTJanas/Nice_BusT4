/*
Wysyłanie UART Break na Wemos D1 mini
Arduino Core 3.0.2

Analizator stanów logicznych pomógł dopracować kształt sygnału
*/

unsigned long timing; 

#define _UART_NO UART0
#define TX_P 1
#define baud_work 19200
#define baud_break 9200
#define LIN_BREAK_BAUDRATE(BAUD) ((BAUD * 9) / 13)

// zmienne dla uart
    int _uart_nr;
    uart_t* _uart = nullptr;
    
    

    uint8_t master_tx_buf[] = {0x55, 0x00, 0x33};
    uint8_t break_tx_buf[] = {0x00};
    //uint8_t break_tx_buf[] = {0x00, 0x66, 0x77, 0x00};
    //master_tx_buf[0] = 0x55; // sync byte
    //master_tx_buf[1] = 0x22;

    //uint8_t dummy = 0;


void send_break() {
  
/* Serial on/off Break
 // Działa źle.
 // 1. W momencie włączenia uart widoczny jest krótki, zbędny impuls
 // 2. Ostatni znak pakietu o niskim baudrate nie zdąży zostać wysłany przed przełączeniem i trafia na linię z wysokim baudrate

  //Serial.end(); // HardwareSerial::begin consist end();
  //Serial.flush();
  Serial.begin(110);  // Starting low baudrate uart
  Serial.write(0x00); // send low baud packet
  Serial.write(0x66);
  
  Serial.end();       // close low baud uart
 
  //Serial.flush();
  Serial.begin(19200); // Starting high baudrate uart
  Serial.write(0x00);  // send high baud packet
  Serial.write(0x55);
*/


//uart.h

/* UART on/off Break
//Działa źle.
// 1. W momencie włączenia uart widoczny jest krótki, zbędny impuls
// 2. Ostatni znak pakietu o niskim baudrate nie zdąży zostać wysłany przed przełączeniem i trafia na linię z wysokim baudrate


 uart_flush(_uart);
 uart_uninit(_uart); 
 _uart =  uart_init(_UART_NO, baud_break, SERIAL_8N1, UART_TX_ONLY, TX_P, 2, false);
 uart_set_baudrate(_uart, baud_break);
  //uart_write(_uart, (char *)&dummy, 1);
 uart_write(_uart, (char *)break_tx_buf, sizeof(break_tx_buf));
 uart_wait_tx_empty(_uart);
  //delayMicroseconds(100);
 uart_uninit(_uart); 
 _uart =  uart_init(_UART_NO, baud_work, SERIAL_8N1, SERIAL_FULL, TX_P, 2, false);
 uart_write(_uart, (char *)master_tx_buf, sizeof(master_tx_buf));
 uart_wait_tx_empty(_uart);
 */



// UART set_baudrate Break
// Działa dobrze z wemos d1 mini i Arduino Core 3.0.2

// Wysyłanie break + pakiet

 uart_flush(_uart);                                               // czyścimy uart
 uart_set_baudrate(_uart, baud_break);                            // obniżamy baudrate
 //uart_write(_uart, (char *)&dummy, 1);        
 uart_write(_uart, (char *)break_tx_buf, sizeof(break_tx_buf));   // wysyłamy zero z niską prędkością, długie zero
 uart_wait_tx_empty(_uart);                                       // czekamy, aż wysyłanie się zakończy. Tu w bibliotece uart.h (esp8266 core 3.0.2) jest błąd, oczekiwanie jest niewystarczające przy późniejszym uart_set_baudrate().
 delayMicroseconds(90);                                          // dodajemy oczekiwanie, inaczej prędkość przełączy się przed zakończeniem wysyłania. Z opóźnieniem na d1-mini uzyskałem idealny sygnał, break = 520us? wysyłałem 0x00
 uart_set_baudrate(_uart, baud_work);                             // przywracamy roboczy baudrate
 uart_write(_uart, (char *)master_tx_buf, sizeof(master_tx_buf)); // wysyłamy główny pakiet
 uart_wait_tx_empty(_uart);                                       // czekamy na zakończenie wysyłania

  
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
 _uart =  uart_init(_UART_NO, baud_work, SERIAL_8N1, SERIAL_FULL, TX_P, 256, false);
}


void loop() {


  if (millis() - timing > 2000) {
    timing = millis();
    bool on_led = digitalRead(LED_BUILTIN);
    digitalWrite(LED_BUILTIN, !on_led);
    send_break(); 
  }
}

