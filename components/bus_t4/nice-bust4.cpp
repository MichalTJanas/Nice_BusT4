#include "nice-bust4.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"  // do korzystania z funkcji pomocniczych do pracy z ciągami znaków






namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";

using namespace esphome::cover;




CoverTraits NiceBusT4::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  return traits;
}


/*
  zrzuty poleceń OVIEW

  SBS               55 0c 00 ff 00 66 01 05 9D 01 82 01 64 E6 0c
  STOP              55 0c 00 ff 00 66 01 05 9D 01 82 02 64 E5 0c
  OPEN              55 0c 00 ff 00 66 01 05 9D 01 82 03 00 80 0c
  CLOSE             55 0c 00 ff 00 66 01 05 9D 01 82 04 64 E3 0c
  PARENTAL OPEN 1   55 0c 00 ff 00 66 01 05 9D 01 82 05 64 E2 0c
  PARENTAL OPEN 2   55 0c 00 ff 00 66 01 05 9D 01 82 06 64 E1 0c



*/

void NiceBusT4::control(const CoverCall &call) {
  position_hook_type = IGNORE;
  if (call.get_stop()) {
    send_cmd(STOP);

  } else if (call.get_position().has_value()) {
    float newpos = *call.get_position();
    if (newpos != position) {
      if (newpos == COVER_OPEN) {
        if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);

      } else if (newpos == COVER_CLOSED) {
        if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);

      } else { // Dowolne położenie
        position_hook_value = (_pos_opn - _pos_cls) * newpos + _pos_cls;
        ESP_LOGI(TAG, "Żądane położenie napędu: %d", position_hook_value);
        if (position_hook_value > _pos_usl) {
          position_hook_type = STOP_UP;
          if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);
        } else {
          position_hook_type = STOP_DOWN;
          if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);
        }
      }
    }
  }
}

void NiceBusT4::setup() {


  _uart =  uart_init(_UART_NO, BAUD_WORK, SERIAL_8N1, SERIAL_FULL, TX_P, 256, false);
  // kto jest w sieci?
//  this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
  

}

void NiceBusT4::loop() {

    if ((millis() - this->last_update_) > 10000) {    // co 10 sekund
// jeśli napęd nie został wykryty za pierwszym razem, spróbujemy później
        std::vector<uint8_t> unknown = {0x55, 0x55};
        if (this->init_ok == false) {
          this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
          this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //zapytanie o produkt
        }
        
        else if (this->class_gate_ == 0x55) {
		init_device(this->addr_to[0], this->addr_to[1], 0x04);  
	//        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
        //        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //zapytanie o produkt
	}
        else if (this->manufacturer_ == unknown)  {
                init_device(this->addr_to[0], this->addr_to[1], 0x04);  
        //        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
        //        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //zapytanie o produkt
		
        }
        this->last_update_ = millis();
    }  // if  co minutę

	
  // zezwalamy na wysyłanie co 100 ms
    uint32_t now = millis();
  if (now - this->last_uart_byte_ > 100) {
    this->ready_to_tx_ = true;
    this->last_uart_byte_ = now;
  } 


  while (uart_rx_available(_uart) > 0) {
    uint8_t c = (uint8_t)uart_read_char(_uart);                // odczytujemy bajt
    this->handle_char_(c);                                     // przekazujemy bajt do przetworzenia
    this->last_uart_byte_ = now;
  } //while

  if (this->ready_to_tx_) {   // jeśli można wysyłać
    if (!this->tx_buffer_.empty()) {  // jeśli jest co wysłać
      this->send_array_cmd(this->tx_buffer_.front()); // wysyłamy pierwsze polecenie z kolejki
      this->tx_buffer_.pop();
      this->ready_to_tx_ = false;
    }
  }

  // Odpytywanie bieżącego położenia napędu
  if (!is_robus) {
  
  now = millis();
  if (init_ok && (current_operation != COVER_OPERATION_IDLE) && (now - last_position_time > POSITION_UPDATE_INTERVAL)) {
  	last_position_time = now;
    request_position();
  } 
  } // not robus

} //loop


void NiceBusT4::handle_char_(uint8_t c) {
  this->rx_message_.push_back(c);                      // dokładamy bajt na koniec odebranej wiadomości
  if (!this->validate_message_()) {                    // sprawdzamy powstałą wiadomość
    this->rx_message_.clear();                         // jeśli sprawdzenie nie przeszło, wiadomość zawiera śmieci i trzeba ją usunąć
  }
}


bool NiceBusT4::validate_message_() {                    // sprawdzenie powstałej wiadomości
  uint32_t at = this->rx_message_.size() - 1;       // numer ostatniego odebranego bajtu
  uint8_t *data = &this->rx_message_[0];               // wskaźnik na pierwszy bajt wiadomości
  uint8_t new_byte = data[at];                      // ostatni odebrany bajt

  // Byte 0: HEADER1 (zawsze 0x00)
  if (at == 0)
    return new_byte == 0x00;
  // Byte 1: HEADER2 (zawsze 0x55)
  if (at == 1)
    return new_byte == START_CODE;

  // Byte 2: packet_size - liczba kolejnych bajtów + 1
  // Sprawdzenie nie jest wykonywane

  if (at == 2)
    return true;
  uint8_t packet_size = data[2];
  uint8_t length = (packet_size + 3); // długość oczekiwanej wiadomości jest już znana


  // Byte 3: Seria (rząd) adresata pakietu
  // Sprawdzenie nie jest wykonywane
  //  uint8_t command = data[3];
  if (at == 3)
    return true;

  // Byte 4: Adres adresata pakietu
  // Byte 5: Seria (rząd) nadawcy pakietu
  // Byte 6: Adres nadawcy pakietu
  // Byte 7: Typ wiadomości CMD lub INF
  // Byte 8: Liczba kolejnych bajtów bez dwóch bajtów CRC na końcu.

  if (at <= 8)
    // Sprawdzenie nie jest wykonywane
    return true;

  uint8_t crc1 = (data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7] ^ data[8]);

  // Byte 9: crc1 = XOR (Byte 3 : Byte 8) XOR sześciu poprzednich bajtów
  if (at == 9)
    if (data[9] != crc1) {
      ESP_LOGW(TAG, "Odebrano wiadomość z błędną sumą kontrolną 1 %02X!=%02X", data[9], crc1);
      return false;
    }
  // Byte 10:
  // ...

  // czekamy, aż nadejdą wszystkie dane pakietu
  if (at  < length)
    return true;

  // obliczamy crc2
  uint8_t crc2 = data[10];
  for (uint8_t i = 11; i < length - 1; i++) {
    crc2 = (crc2 ^ data[i]);
  }

  if (data[length - 1] != crc2 ) {
    ESP_LOGW(TAG, "Odebrano wiadomość z błędną sumą kontrolną 2 %02X!=%02X", data[length - 1], crc2);
    return false;
  }

  // Byte Last: packet_size
  //  if (at  ==  length) {
  if (data[length] != packet_size ) {
    ESP_LOGW(TAG, "Odebrano wiadomość o błędnym rozmiarze %02X!=%02X", data[length], packet_size);
    return false;
  }

  // Jeśli tu dotarliśmy – poprawna wiadomość została odebrana i leży w buforze rx_message_

  // Usuwamy 0x00 z początku wiadomości
  rx_message_.erase(rx_message_.begin());

  // do wypisania pakietu w logu
  std::string pretty_cmd = format_hex_pretty(rx_message_);
  ESP_LOGI(TAG,  "Odebrano pakiet: %S ", pretty_cmd.c_str() );

  // tutaj przetwarzamy wiadomość
  parse_status_packet(rx_message_);



  // zwracamy false, aby wyzerować rx buffer
  return false;

}


// analizujemy odebrane pakiety
void NiceBusT4::parse_status_packet (const std::vector<uint8_t> &data) {
  if ((data[1] == 0x0d) && (data[13] == 0xFD)) { // błąd
    ESP_LOGE(TAG,  "Polecenie niedostępne dla tego urządzenia" );
  }

  if (((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) { // if evt
  //  ESP_LOGD(TAG, "Odebrano pakiet EVT z danymi. Ostatnia komórka %d ", data[12]);
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    std::string str(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    ESP_LOGI(TAG,  "Ciąg z danymi: %S ", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "Dane HEX %S ", pretty_data.c_str() );
    // odebraliśmy pakiet z danymi EVT, zaczynamy analizę

    if ((data[6] == INF) && (data[9] == FOR_CU)  && (data[11] == GET - 0x80) && (data[13] == NOERR)) { // interesują nas zakończone odpowiedzi na zapytania GET, które przyszły bez błędów od napędu
      ESP_LOGI(TAG,  "Odebrano odpowiedź na zapytanie %X ", data[10] );
      switch (data[10]) { // cmd_submnu
        case TYPE_M:
          //           ESP_LOGI(TAG,  "Typ napędu %X",  data[14]);
          switch (data[14]) { //14
            case SLIDING:
              this->class_gate_ = SLIDING;
              //        ESP_LOGD(TAG, "Typ bramy: przesuwna %#X ", data[14]);
              break;
            case SECTIONAL:
              this->class_gate_ = SECTIONAL;
              //        ESP_LOGD(TAG, "Typ bramy: segmentowa %#X ", data[14]);
              break;
            case SWING:
              this->class_gate_ = SWING;
              //        ESP_LOGD(TAG, "Typ bramy: skrzydłowa %#X ", data[14]);
              break;
            case BARRIER:
              this->class_gate_ = BARRIER;
              //        ESP_LOGD(TAG, "Typ bramy: szlaban %#X ", data[14]);
              break;
            case UPANDOVER:
              this->class_gate_ = UPANDOVER;
              //        ESP_LOGD(TAG, "Typ bramy: uchylna %#X ", data[14]);
              break;
          }  // switch 14
          break; //  TYPE_M
        case INF_IO: // odpowiedź na zapytanie o stan krańcówki bramy przesuwnej
          switch (data[16]) { //16
            case 0x00:
              ESP_LOGI(TAG, "  Krańcówka nie zadziałała ");
              break; // 0x00
            case 0x01:
              ESP_LOGI(TAG, "  Krańcówka zamknięcia ");
              this->position = COVER_CLOSED;
              break; //  0x01
            case 0x02:
              ESP_LOGI(TAG, "  Krańcówka otwarcia ");
              this->position = COVER_OPEN;
              break; // 0x02

          }  // switch 16
          this->publish_state_if_changed();  // publikujemy stan

          break; //  INF_IO


        //położenie maksymalnego otwarcia enkodera, otwarcia, zamknięcia

        case MAX_OPN:
          if (is_walky) {
            this->_max_opn = data[15];
            this->_pos_opn = data[15];
          }
          else {  
            this->_max_opn = (data[14] << 8) + data[15];
          }
          ESP_LOGI(TAG, "Maksymalne położenie enkodera: %d", this->_max_opn);
          break;

        case POS_MIN:
          this->_pos_cls = (data[14] << 8) + data[15];
          ESP_LOGI(TAG, "Położenie zamkniętej bramy: %d", this->_pos_cls);
          break;

        case POS_MAX:
          if (((data[14] << 8) + data[15])>0x00) { // jeśli odpowiedź napędu zawiera dane o położeniu otwarcia
          this->_pos_opn = (data[14] << 8) + data[15];}
          ESP_LOGI(TAG, "Położenie otwartej bramy: %d", this->_pos_opn);
          break;

        case CUR_POS:
          if (is_walky)
            update_position(data[15]);
          else
            update_position((data[14] << 8) + data[15]);
          break;

        case INF_STATUS:
          switch (data[14]) {
            case OPENED:
              ESP_LOGI(TAG, "  Brama otwarta");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_OPEN;
              break;
            case CLOSED:
              ESP_LOGI(TAG, "  Brama zamknięta");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_CLOSED;
              break;
            case 0x01:
              ESP_LOGI(TAG, "  Brama zatrzymana");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
            case 0x00:
              ESP_LOGI(TAG, "  Stan bramy nieznany");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
             case 0x0b:
              ESP_LOGI(TAG, "  Wyszukiwanie położeń zakończone");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
              case STA_OPENING:
              ESP_LOGI(TAG, "  Trwa otwieranie");
              this->current_operation = COVER_OPERATION_OPENING;
              break;
              case STA_CLOSING:
              ESP_LOGI(TAG, "  Trwa zamykanie");
              this->current_operation = COVER_OPERATION_CLOSING;
              break;
          }  // switch
          this->publish_state_if_changed();  // publikujemy stan
          break;

          //      default: // cmd_mnu
        case AUTOCLS:
          this->autocls_flag = data[14];
	  ESP_LOGCONFIG(TAG, "  Automatyczne zamykanie - L1: %S ", autocls_flag ? "Tak" : "Nie");	
          break;
          
        case PH_CLS_ON:
          this->photocls_flag = data[14];
          break;  
          
        case ALW_CLS_ON:
          this->alwayscls_flag = data[14];
          break;  
          
      } // switch cmd_submnu
    } // if zakończone odpowiedzi na zapytania GET, które przyszły bez błędów od napędu

     if ((data[6] == INF) &&  (data[11] == GET - 0x81) && (data[13] == NOERR)) { // interesują nas niezakończone odpowiedzi na zapytania GET, które przyszły bez błędów od wszystkich
	ESP_LOGI(TAG,  "Odebrano niezakończoną odpowiedź na zapytanie %X, kontynuacja z przesunięciem %X", data[10], data[12] );
	     // powtarzamy polecenie z nowym przesunięciem
	tx_buffer_.push(gen_inf_cmd(data[4], data[5], data[9], data[10], GET, data[12]));
     
     } // niezakończone odpowiedzi na zapytania GET, które przyszły bez błędów od napędu

	  
    
    if ((data[6] == INF) && (data[9] == FOR_CU)  && (data[11] == SET - 0x80) && (data[13] == NOERR)) { // interesują nas odpowiedzi na zapytania SET, które przyszły bez błędów od napędu    
      switch (data[10]) { // cmd_submnu
        case AUTOCLS:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, AUTOCLS, GET)); // Automatyczne zamykanie
          break;
          
        case PH_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, PH_CLS_ON, GET)); // Zamknij po fotokomórce
          break;  
          
        case ALW_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, ALW_CLS_ON, GET)); // Zawsze zamykaj
          break;  
      }// switch cmd_submnu
    }// if odpowiedzi na zapytania SET, które przyszły bez błędów od napędu

    if ((data[6] == INF) && (data[9] == FOR_ALL)  && ((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) { // interesują nas odpowiedzi FOR_ALL na zapytania GET, które przyszły bez błędów

      switch (data[10]) {
        case MAN:
          //       ESP_LOGCONFIG(TAG, "  Producent: %S ", str.c_str());
          this->manufacturer_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          break;
        case PRD:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // jeśli pakiet od odbiornika
//            ESP_LOGCONFIG(TAG, "  Odbiornik: %S ", str.c_str());
            this->oxi_product.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } // jeśli pakiet od odbiornika
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // jeśli pakiet od sterownika napędu
//            ESP_LOGCONFIG(TAG, "  Napęd: %S ", str.c_str());
            this->product_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
            std::vector<uint8_t> wla1 = {0x57,0x4C,0x41,0x31,0x00,0x06,0x57}; // do rozpoznania, że napęd to Walky
            std::vector<uint8_t> ROBUSHSR10 = {0x52,0x4F,0x42,0x55,0x53,0x48,0x53,0x52,0x31,0x30,0x00}; // do rozpoznania, że napęd to ROBUSHSR10
			std::vector<uint8_t> ROBUS = {0x52,0x4F,0x42,0x55,0x53,0x00}; // do rozpoznania, że napęd to ROBUS
            if (this->product_ == wla1) { 
              this->is_walky = true;
         //     ESP_LOGCONFIG(TAG, "  Napęd WALKY!: %S ", str.c_str());
                                        }
            if (this->product_ == ROBUSHSR10) { 
              this->is_robus = true;
          //    ESP_LOGCONFIG(TAG, "  Napęd ROBUS!: %S ", str.c_str());
                                        }		  
            if (this->product_ == ROBUS) { 
              this->is_robus = true;
          //    ESP_LOGCONFIG(TAG, "  Napęd ROBUS!: %S ", str.c_str());
                                        }		  

			  
          }
          break;
        case HWR:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // jeśli pakiet od odbiornika
            this->oxi_hardware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // jeśli pakiet od sterownika napędu          
          this->hardware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case FRM:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // jeśli pakiet od odbiornika
            this->oxi_firmware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // jeśli pakiet od sterownika napędu          
            this->firmware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case DSC:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // jeśli pakiet od odbiornika
            this->oxi_description.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // jeśli pakiet od sterownika napędu          
            this->description_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case WHO:
          if (data[12] == 0x01) {
            if (data[14] == 0x04) { // napęd
              this->addr_to[0] = data[4];
              this->addr_to[1] = data[5];
              this->init_ok = true;
     //         init_device(data[4], data[5], data[14]);
            }
            else if (data[14] == 0x0A) { // odbiornik
              this->addr_oxi[0] = data[4];
              this->addr_oxi[1] = data[5];
              init_device(data[4], data[5], data[14]);
            }
          }
          break;
      }  // switch

    }  // if  odpowiedzi FOR_ALL na zapytania GET, które przyszły bez błędów

    if ((data[9] == 0x0A) &&  (data[10] == 0x25) &&  (data[11] == 0x01) &&  (data[12] == 0x0A) &&  (data[13] == NOERR)) { //  pakiety od odbiornika z informacją o liście pilotów, które przyszły bez błędów
      ESP_LOGCONFIG(TAG, "Numer pilota: %X%X%X%X, polecenie: %X, przycisk: %X, tryb: %X, licznik naciśnięć: %d", vec_data[5], vec_data[4], vec_data[3], vec_data[2], vec_data[8] / 0x10, vec_data[5] / 0x10, vec_data[7] + 0x01, vec_data[6]);
    }  // if

    if ((data[9] == 0x0A) &&  (data[10] == 0x26) &&  (data[11] == 0x41) &&  (data[12] == 0x08) &&  (data[13] == NOERR)) { //  pakiety od odbiornika z informacją o odczytanym przycisku pilota
      ESP_LOGCONFIG(TAG, "Przycisk %X, numer pilota: %X%X%X%X", vec_data[0] / 0x10, vec_data[0] % 0x10, vec_data[1], vec_data[2], vec_data[3]);
    }  // if

  } //  if evt



  //else if ((data[14] == NOERR) && (data[1] > 0x0d)) {  // w przeciwnym razie pakiet Responce – potwierdzenie odebranego polecenia
  else if (data[1] > 0x0d) {  // w przeciwnym razie pakiet Responce – potwierdzenie odebranego polecenia
    ESP_LOGD(TAG, "Odebrano pakiet RSP");
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    std::string str(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    ESP_LOGI(TAG,  "Ciąg z danymi: %S ", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "Dane HEX %S ", pretty_data.c_str() );
    switch (data[9]) { // cmd_mnu
      case FOR_CU:
        ESP_LOGI(TAG, "Pakiet sterownika napędu");
        switch (data[10] + 0x80) { // sub_inf_cmd
          case RUN:
            ESP_LOGI(TAG, "Podmenu RUN");
			if (data[11] >= 0x80) {
			  switch (data[11] - 0x80) {  // sub_run_cmd1
			    case SBS:
			      ESP_LOGI(TAG, "Polecenie: Krok po kroku");
			      break;
			    case STOP:
			      ESP_LOGI(TAG, "Polecenie: STOP");
			      break;
			    case OPEN:
			      ESP_LOGI(TAG, "Polecenie: Otwórz");
			      this->current_operation = COVER_OPERATION_OPENING;
			      break;
			    case CLOSE:
			      ESP_LOGI(TAG, "Polecenie: Zamknij");
			      this->current_operation = COVER_OPERATION_CLOSING;
			      break;
			    case P_OPN1:
			      ESP_LOGI(TAG, "Polecenie: Częściowe otwarcie 1");
			      break;
			    case STOPPED:
			      ESP_LOGI(TAG, "Polecenie: Zatrzymano");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    case ENDTIME:
			      ESP_LOGI(TAG, "Operacja zakończona po przekroczeniu czasu");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    default:
			      ESP_LOGI(TAG, "Nieznane polecenie: %X", data[11]);
			  }  // switch sub_run_cmd1
			} else {
			  switch (data[11]) {  // sub_run_cmd2
			    case STA_OPENING:
			      ESP_LOGI(TAG, "Operacja: Otwiera się");
			      this->current_operation = COVER_OPERATION_OPENING;
			      break;
			    case STA_CLOSING:
			      ESP_LOGI(TAG, "Operacja: Zamyka się");
			      this->current_operation = COVER_OPERATION_CLOSING;
			      break;
			    case CLOSED:
			      ESP_LOGI(TAG, "Operacja: Zamknięte");
			      this->current_operation = COVER_OPERATION_IDLE;
			      this->position = COVER_CLOSED;
			      break;
			    case OPENED:
			      ESP_LOGI(TAG, "Operacja: Otwarte");
			      this->current_operation = COVER_OPERATION_IDLE;
			      this->position = COVER_OPEN;
			      // calibrate opened position if the motor does not report max supported position (Road 400)
                  if (this->_max_opn == 0) {
                    this->_max_opn = this->_pos_opn = this->_pos_usl;
                    ESP_LOGI(TAG, "Skalibrowano położenie otwarcia");
                  }
			      break;
			    case STOPPED:
			      ESP_LOGI(TAG, "Operacja: Zatrzymano");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    case PART_OPENED:
			      ESP_LOGI(TAG, "Operacja: Częściowo otwarte");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    default:
			      ESP_LOGI(TAG, "Nieznana operacja: %X", data[11]);
			  }  // switch sub_run_cmd2
			}
			this->publish_state_if_changed();  // publikujemy stan
            break; //RUN

          case STA:
            ESP_LOGI(TAG,  "Podmenu Stan w ruchu" );
            switch (data[11]) { // sub_run_cmd2
              case STA_OPENING:
              case 0x83: // Road 400
                ESP_LOGI(TAG, "Ruch: Otwiera się" );
                this->current_operation = COVER_OPERATION_OPENING;
                break;
              case STA_CLOSING:
              case 0x84: // Road 400
                ESP_LOGI(TAG,  "Ruch: Zamyka się" );
                this->current_operation = COVER_OPERATION_CLOSING;
                break;
              case CLOSED:
                ESP_LOGI(TAG,  "Ruch: Zamknięte" );
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_CLOSED;
                break;
              case OPENED:
                ESP_LOGI(TAG, "Ruch: Otwarte");
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_OPEN;
                break;
              case STOPPED:
                ESP_LOGI(TAG, "Ruch: Zatrzymano");
                this->current_operation = COVER_OPERATION_IDLE;
                request_position();
                break;
              default: // sub_run_cmd2
                ESP_LOGI(TAG,  "Ruch: %X", data[11] );

                
            } // switch sub_run_cmd2

            update_position((data[12] << 8) + data[13]);
            break; //STA

          default: // sub_inf_cmd
            ESP_LOGI(TAG,  "Podmenu %X", data[10] );
        }  // switch sub_inf_cmd

        break; // Pakiet sterownika napędu
      case CONTROL:
        ESP_LOGI(TAG,  "Pakiet CONTROL" );
        break; // CONTROL
      case FOR_ALL:
        ESP_LOGI(TAG,  "Pakiet dla wszystkich" );
        break; // FOR_ALL
      case 0x0A:
        ESP_LOGI(TAG,  "Pakiet odbiornika" );
        break; // pakiet odbiornika
      default: // cmd_mnu
        ESP_LOGI(TAG,  "Menu %X", data[9] );
    }  // switch  cmd_mnu


  } // else

 if ((data[6] == CMD) && (data[9] == FOR_CU)  && (data[10] == CUR_MAN) && (data[13] == NOERR)) { // interesują nas odpowiedzi FOR_CU na zapytania CMD, które przyszły bez błędów. Szukamy stanu dla RO600

  ///////////////////////////////////////////////////////////////////////////////////


  // RSP – odpowiedź (ReSPonce) na samo przyjęcie polecenia CMD, a nie na jego wykonanie. Informuje też o zakończeniu operacji.
  /* if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == CUR_MAN) && (data[12] == 0x19)) { // rozpoznajemy pakiet stanu po zawartości określonych bajtów
     //  ESP_LOGD(TAG, "Odebrano pakiet RSP. cmd = %#x", data[11]);
*/
     switch (data[11]) {
       case STA_OPENING:
         this->current_operation = COVER_OPERATION_OPENING;
         ESP_LOGD(TAG, "Stan: Otwiera się");
         break;
       case STA_CLOSING:
         this->current_operation = COVER_OPERATION_CLOSING;
         ESP_LOGD(TAG, "Stan: Zamyka się");
         break;
       case OPENED:
         this->position = COVER_OPEN;
         ESP_LOGD(TAG, "Stan: Otwarte");
         this->current_operation = COVER_OPERATION_IDLE;
         break;


       case CLOSED:
         this->position = COVER_CLOSED;
         ESP_LOGD(TAG, "Stan: Zamknięte");
         this->current_operation = COVER_OPERATION_IDLE;
         break;
       case STOPPED:
         this->current_operation = COVER_OPERATION_IDLE;
         ESP_LOGD(TAG, "Stan: Zatrzymano");
         break;

     }  // switch

     this->publish_state();  // publikujemy stan

    } //if
 
  /*
    // stan po osiągnięciu krańcówek
    if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == CUR_MAN) &&  (data[12] == 0x00)) { // rozpoznajemy pakiet stanu po zawartości określonych bajtów
      ESP_LOGD(TAG, "Odebrano pakiet krańcówek. Stan = %#x", data[11]);
      switch (data[11]) {
        case OPENED:
          this->position = COVER_OPEN;
          ESP_LOGD(TAG, "Stan: Otwarte");
          this->current_operation = COVER_OPERATION_IDLE;
          break;
        case CLOSED:
          this->position = COVER_CLOSED;
          ESP_LOGD(TAG, "Stan: Zamknięte");
          this->current_operation = COVER_OPERATION_IDLE;
          break;
        case OPENING:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Stan: Otwiera się");
          break;
        case CLOSING:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Stan: Zamyka się");
          break;
      } //switch
      this->publish_state();  // publikujemy stan
    } //if
  */
  // STA = 0x40,   // stan w ruchu
  /*
    if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == STA) ) { // rozpoznajemy pakiet stanu po zawartości określonych bajtów
      uint16_t ipos = (data[12] << 8) + data[13];
      ESP_LOGD(TAG, "Bieżący manewr: %#X Pozycja: %#X %#X, ipos = %#x,", data[11], data[12], data[13], ipos);
      this->position = ipos / 2100.0f; // przekazujemy pozycję do komponentu

      switch (data[11]) {
        case OPENING:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Stan: Otwiera się");
          break;

        case OPENING2:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Stan: Otwiera się");
          break;

        case CLOSING:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Stan: Zamyka się");
          break;
        case CLOSING2:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Stan: Zamyka się");
          break;
        case OPENED:
          this->position = COVER_OPEN;
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Stan: Otwarte");
          //      this->current_operation = COVER_OPERATION_OPENING;
          //    ESP_LOGD(TAG, "Stan: Otwiera się");
          break;
        case CLOSED:
          this->position = COVER_CLOSED;
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Stan: Zamknięte");
          //      this->current_operation = COVER_OPERATION_CLOSING;
          //ESP_LOGD(TAG, "Stan: Zamyka się");
          break;
        case STOPPED:
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Stan: Zatrzymano");
          break;

      }  // switch

      this->publish_state();  // publikujemy stan

    } //if
  */


  ////////////////////////////////////////////////////////////////////////////////////////
} // function







void NiceBusT4::dump_config() {    //  dodajemy do logu informacje o podłączonym sterowniku
  ESP_LOGCONFIG(TAG, "  Napęd Bus T4");
  /*ESP_LOGCONFIG(TAG, "  Adres: 0x%02X%02X", *this->header_[1], *this->header_[2]);*/
  switch (this->class_gate_) {
    case SLIDING:
      ESP_LOGCONFIG(TAG, "  Typ: Brama przesuwna");
      break;
    case SECTIONAL:
      ESP_LOGCONFIG(TAG, "  Typ: Brama segmentowa");
      break;
    case SWING:
      ESP_LOGCONFIG(TAG, "  Typ: Brama skrzydłowa");
      break;
    case BARRIER:
      ESP_LOGCONFIG(TAG, "  Typ: Szlaban");
      break;
    case UPANDOVER:
      ESP_LOGCONFIG(TAG, "  Typ: Brama uchylna");
      break;
    default:
      ESP_LOGCONFIG(TAG, "  Typ: Nieznana brama, 0x%02X", this->class_gate_);
  } // switch


  ESP_LOGCONFIG(TAG, "  Maksymalne położenie enkodera lub timera: %d", this->_max_opn);
  ESP_LOGCONFIG(TAG, "  Położenie otwartej bramy: %d", this->_pos_opn);
  ESP_LOGCONFIG(TAG, "  Położenie zamkniętej bramy: %d", this->_pos_cls);

  std::string manuf_str(this->manufacturer_.begin(), this->manufacturer_.end());
  ESP_LOGCONFIG(TAG, "  Producent: %S ", manuf_str.c_str());

  std::string prod_str(this->product_.begin(), this->product_.end());
  ESP_LOGCONFIG(TAG, "  Napęd: %S ", prod_str.c_str());

  std::string hard_str(this->hardware_.begin(), this->hardware_.end());
  ESP_LOGCONFIG(TAG, "  Sprzęt napędu: %S ", hard_str.c_str());

  std::string firm_str(this->firmware_.begin(), this->firmware_.end());
  ESP_LOGCONFIG(TAG, "  Firmware napędu: %S ", firm_str.c_str());
  
  std::string dsc_str(this->description_.begin(), this->description_.end());
  ESP_LOGCONFIG(TAG, "  Opis napędu: %S ", dsc_str.c_str());


  ESP_LOGCONFIG(TAG, "  Adres bramki: 0x%02X%02X", addr_from[0], addr_from[1]);
  ESP_LOGCONFIG(TAG, "  Adres napędu: 0x%02X%02X", addr_to[0], addr_to[1]);
  ESP_LOGCONFIG(TAG, "  Adres odbiornika: 0x%02X%02X", addr_oxi[0], addr_oxi[1]);
  
  std::string oxi_prod_str(this->oxi_product.begin(), this->oxi_product.end());
  ESP_LOGCONFIG(TAG, "  Odbiornik: %S ", oxi_prod_str.c_str());
  
  std::string oxi_hard_str(this->oxi_hardware.begin(), this->oxi_hardware.end());
  ESP_LOGCONFIG(TAG, "  Sprzęt odbiornika: %S ", oxi_hard_str.c_str());

  std::string oxi_firm_str(this->oxi_firmware.begin(), this->oxi_firmware.end());
  ESP_LOGCONFIG(TAG, "  Firmware odbiornika: %S ", oxi_firm_str.c_str());
  
  std::string oxi_dsc_str(this->oxi_description.begin(), this->oxi_description.end());
  ESP_LOGCONFIG(TAG, "  Opis odbiornika: %S ", oxi_dsc_str.c_str());
 
  ESP_LOGCONFIG(TAG, "  Automatyczne zamykanie - L1: %S ", autocls_flag ? "Tak" : "Nie");
  ESP_LOGCONFIG(TAG, "  Zamknij po fotokomórce - L2: %S ", photocls_flag ? "Tak" : "Nie");
  ESP_LOGCONFIG(TAG, "  Zawsze zamykaj - L3: %S ", alwayscls_flag ? "Tak" : "Nie");
  
}




//tworzenie polecenia sterującego
std::vector<uint8_t> NiceBusT4::gen_control_cmd(const uint8_t control_cmd) {
  std::vector<uint8_t> frame = {this->addr_to[0], this->addr_to[1], this->addr_from[0], this->addr_from[1]}; // nagłówek
  frame.push_back(CMD);  // 0x01
  frame.push_back(0x05);
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(CONTROL);
  frame.push_back(RUN);
  frame.push_back(control_cmd);
  frame.push_back(0x64); // OFFSET CMD, DPRO924 nie chciał działać z 0x00, choć pozostałe napędy reagowały na polecenia
  uint8_t crc2 = (frame[7] ^ frame[8] ^ frame[9] ^ frame[10]);
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);

  // do wypisania polecenia w logu
  //  std::string pretty_cmd = format_hex_pretty(frame);
  //  ESP_LOGI(TAG,  "Utworzono polecenie: %S ", pretty_cmd.c_str() );

  return frame;
}

// tworzenie polecenia INF z danymi i bez
std::vector<uint8_t> NiceBusT4::gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len) {
  std::vector<uint8_t> frame = {to_addr1, to_addr2, this->addr_from[0], this->addr_from[1]}; // nagłówek
  frame.push_back(INF);  // 0x08 mes_type
  frame.push_back(0x06 + len); // mes_size
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(whose);
  frame.push_back(inf_cmd);
  frame.push_back(run_cmd);
  frame.push_back(next_data); // next_data
  frame.push_back(len);
  if (len > 0) {
    frame.insert(frame.end(), data.begin(), data.end()); // blok danych
  }
  uint8_t crc2 = frame[7];
  for (size_t i = 8; i < 12 + len; i++) {
    crc2 = crc2 ^ frame[i];
  }
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);

  // do wypisania polecenia w logu
  //  std::string pretty_cmd = format_hex_pretty(frame);
  //  ESP_LOGI(TAG,  "Utworzono pakiet INF: %S ", pretty_cmd.c_str() );

  return frame;

}


void NiceBusT4::send_raw_cmd(std::string data) {

  std::vector < uint8_t > v_cmd = raw_cmd_prepare (data);
  send_array_cmd (&v_cmd[0], v_cmd.size());

}


//  Tutaj trzeba dodać sprawdzanie błędnych danych od użytkownika
std::vector<uint8_t> NiceBusT4::raw_cmd_prepare (std::string data) { // przygotowanie danych wprowadzonych przez użytkownika do wysłania
// usuwamy wszystko oprócz liter i cyfr szesnastkowych
data.erase(remove_if(data.begin(), data.end(), [](const unsigned char ch) {
    return (!(isxdigit(ch)) );
  }), data.end()); 

  //assert (data.size () % 2 == 0); // sprawdzamy parzystość
  std::vector < uint8_t > frame;
  frame.resize(0); // zerujemy rozmiar polecenia

  for (uint8_t i = 0; i < data.size (); i += 2 ) { // wypełniamy tablicę polecenia
    std::string sub_str(data, i, 2); // bierzemy 2 bajty z polecenia
    char hexstoi = (char)std::strtol(&sub_str[0], 0 , 16); // zamieniamy na liczbę
    frame.push_back(hexstoi);  // zapisujemy liczbę w elemencie ciągu nowego polecenia
  }


  return frame;

}



void NiceBusT4::send_array_cmd (std::vector<uint8_t> data) {          // wysyła break + polecenie przygotowane wcześniej w tablicy
  return send_array_cmd((const uint8_t *)data.data(), data.size());
}
void NiceBusT4::send_array_cmd (const uint8_t *data, size_t len) {
  // wysyłanie danych przez uart

  char br_ch = 0x00;                                               // dla break
  uart_flush(_uart);                                               // czyścimy uart
  uart_set_baudrate(_uart, BAUD_BREAK);                            // obniżamy baudrate
  uart_write(_uart, &br_ch, 1);                                    // wysyłamy zero z niską prędkością, długie zero
  //uart_write(_uart, (char *)&dummy, 1);
  uart_wait_tx_empty(_uart);                                       // czekamy, aż wysyłanie się zakończy. W bibliotece uart.h (esp8266 core 3.0.2) jest tu błąd, samo oczekiwanie nie wystarcza przy późniejszym uart_set_baudrate().
  delayMicroseconds(90);                                          // dodajemy opóźnienie do oczekiwania, inaczej prędkość przełączy się przed zakończeniem wysyłania. Z opóźnieniem na d1-mini uzyskałem idealny sygnał, break = 520us
  uart_set_baudrate(_uart, BAUD_WORK);                             // przywracamy roboczy baudrate
  uart_write(_uart, (char *)&data[0], len);                                // wysyłamy główny pakiet
  //uart_write(_uart, (char *)raw_cmd_buf, sizeof(raw_cmd_buf));
  uart_wait_tx_empty(_uart);                                       // czekamy na zakończenie wysyłania



  std::string pretty_cmd = format_hex_pretty((uint8_t*)&data[0], len);                    // do wypisania polecenia w logu
  ESP_LOGI(TAG,  "Wysłano: %S ", pretty_cmd.c_str() );

}


// generowanie i wysyłanie poleceń inf z konfiguracji yaml
void NiceBusT4::send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command, std::string next_data, bool data_on, std::string data_command) {
  std::vector < uint8_t > v_to_addr = raw_cmd_prepare (to_addr);
  std::vector < uint8_t > v_whose = raw_cmd_prepare (whose);
  std::vector < uint8_t > v_command = NiceBusT4::raw_cmd_prepare (command);
  std::vector < uint8_t > v_type_command = raw_cmd_prepare (type_command);
  std::vector < uint8_t > v_next_data = raw_cmd_prepare (next_data);
  std::vector < uint8_t > v_data_command = raw_cmd_prepare (data_command);


  if (data_on) {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0], v_data_command, v_data_command.size()));
  } else {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0]));
  } // else
}

// generowanie i wysyłanie poleceń ustawień do sterownika napędu z konfiguracji yaml z minimalnymi parametrami
void NiceBusT4::set_mcu(std::string command, std::string data_command) {
    std::vector < uint8_t > v_command = raw_cmd_prepare (command);
    std::vector < uint8_t > v_data_command = raw_cmd_prepare (data_command);
    tx_buffer_.push(gen_inf_cmd(0x04, v_command[0], 0xa9, 0x00, v_data_command));
  }
  
// inicjalizacja urządzenia
void NiceBusT4::init_device (const uint8_t addr1, const uint8_t addr2, const uint8_t device ) {
  if (device == FOR_CU) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, TYPE_M, GET, 0x00)); // zapytanie o typ napędu
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, MAN, GET, 0x00)); // zapytanie o producenta
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00)); //  zapytanie o firmware
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00)); //zapytanie o produkt
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00)); //zapytanie o sprzęt
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MAX, GET, 0x00));   //zapytanie o pozycję otwarcia
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MIN, GET, 0x00)); // zapytanie o pozycję zamknięcia
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00)); //zapytanie o opis
    if (is_walky)  // zapytanie o maksymalną wartość enkodera
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00, {0x01}, 1));
    else
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00));
    request_position();  // zapytanie o bieżące położenie
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, INF_STATUS, GET, 0x00)); //Stan bramy (Otwarta/Zamknięta/Zatrzymana)
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, AUTOCLS, GET, 0x00)); // Automatyczne zamykanie
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, PH_CLS_ON, GET, 0x00)); // Zamknij po fotokomórce
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, ALW_CLS_ON, GET, 0x00)); // Zawsze zamykaj
  }
  if (device == FOR_OXI) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00)); //zapytanie o produkt
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00)); //zapytanie o sprzęt    
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00)); //  zapytanie o firmware    
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00)); //zapytanie o opis    
  }
  
}

// Zapytanie o umowne bieżące położenie napędu
void NiceBusT4::request_position(void) {
  if (is_walky)
    tx_buffer_.push(gen_inf_cmd(this->addr_to[0], this->addr_to[1], FOR_CU, CUR_POS, GET, 0x00, {0x01}, 1));
  else if (!is_robus)
    tx_buffer_.push(gen_inf_cmd(FOR_CU, CUR_POS, GET));
}

// Aktualizacja bieżącego położenia napędu
void NiceBusT4::update_position(uint16_t newpos) {
  last_position_time = millis();
  _pos_usl = newpos;
  position = (_pos_usl - _pos_cls) * 1.0f / (_pos_opn - _pos_cls);
  ESP_LOGI(TAG, "Umowne położenie bramy: %d, położenie w %%: %.3f", newpos, position);
  if (position < CLOSED_POSITION_THRESHOLD) position = COVER_CLOSED;
  publish_state_if_changed();  // publikujemy stan
  
  if ((position_hook_type == STOP_UP && _pos_usl >= position_hook_value) || (position_hook_type == STOP_DOWN && _pos_usl <= position_hook_value)) {
  	ESP_LOGI(TAG, "Osiągnięto żądane położenie. Zatrzymujemy bramę");
  	send_cmd(STOP);
  	position_hook_type = IGNORE;
  }
}

// Publikacja stanu bramy przy zmianie
void NiceBusT4::publish_state_if_changed(void) {
  if (current_operation == COVER_OPERATION_IDLE) position_hook_type = IGNORE;
  if (last_published_op != current_operation || last_published_pos != position) {
    publish_state();
    last_published_op = current_operation;
    last_published_pos = position;
  }
}

}  // namespace bus_t4
}  // namespace esphome
