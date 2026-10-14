Dostępne tłumaczenie na angielski
[English](https://github.com/xdanik/Nice_BusT4)


# Komponent ESPHome do sterowania napędami Nice za pomocą protokołu Bus T4
# Nice Bus T4 protocol

Pojawiła się chęć zrozumienia protokołu sterowania bramami Nice.
Perspektywa – tanie urządzenia oparte na esp8266 do sterowania z inteligentnego domu.

Nowoczesne centrale sterujące napędami mają złącze BusT4, na które wyprowadzone są GND, +V, Can-Rx, Can-Tx. Wartość napięcia V może wynosić od 24 do 35 woltów w zależności od centrali sterującej.

# Aktualne możliwości komponentu
* Wysyłanie poleceń: "Otwórz", "Stop", "Zamknij", "Częściowe otwarcie", "Krok po kroku (SBS)" i innych za pomocą przycisków.
* Wysyłanie dowolnych poleceń HEX przez usługę "raw_command". Polecenie musi być przygotowane wcześniej lub gdzieś podejrzane. Separatorami bajtów mogą być kropki lub spacje. Przykład: 55 0c 00 03 00 81 01 05 86 01 82 01 64 e6 0c lub 55.0D.00.FF.00.66.08.06.97.00.04.99.00.00.9D.0D
* Tworzenie i wysyłanie dowolnych zapytań GET/SET przez usługę "send_inf_command". Pozwala skonfigurować urządzenie lub odczytać jego statusy.
* Wyświetlanie w logu pakietów od wszystkich urządzeń w sieci busT4.

# BusT4:

Jest to zmodyfikowany UART 19200 8n1 z uart.break o długości 519us-590us przed każdym pakietem.
Można podłączyć kilka urządzeń, w tym celu do warstwy fizycznej dodano transceivery CAN-BUS.
Transmisja fizyczna odbywa się najczęściej przez transceivery CAN, ale ramek CAN nie ma.

# Co zostało zrobione:
* Podłączyłem FTDI232 do GND, Can-Rx, Can-Tx. Pakiety są widoczne i dają się rozszyfrować.
* Analizatorem logicznym zobaczyłem kształt sygnału i zawartość pakietów, dobrałem parametry uart.
* Z powodzeniem odtworzyłem odczytany pakiet za pomocą Arduino Mega, napęd reaguje.
* Uzyskałem polecenia OPEN CLOSE itd.
* Uzyskałem bajt statusu napędu
* Odczytałem podstawowe polecenia, częściowo rozszyfrowałem znaczenie bajtów.
* Zbudowałem prototyp urządzenia, przetestowałem działanie.
* Zbudowałem sniffer do przechwytywania pakietów między OVIEW a urządzeniami busT4
* Napisałem komponent, który umożliwia sterowanie napędami i odbiornikami za pomocą protokołu BusT4
* Sprawdziłem działanie na Wingo5000 z centralą [MCA5](img/IMG_20220113_160221.jpg), [Robus RB500HS](img/3hs.jpg), Soona SO2000, [Rd400](img/rba4.jpg), [D-PRO924](img/924.jpg), [Walky WL1024C](img/walky.jpg), [SPIN 22BDKCE](img/spin.jpg).

![alt text](img/Schematic_esphome_bust4_adapter.png "Schemat adaptera bus-t4")


ESP8266 nie jest zgodny poziomem sygnału z BUS T4, należy dodać konwerter poziomów 3.3V -> 5V dla Tx na tranzystorze.
Rx ESP toleruje 5V, ale do stabilnej pracy potrzebna jest dioda. U mnie działa z przypadkową germanową, możliwe, że krzemowa również się nada.

W dalszej kolejności schemat został zmodyfikowany.
![alt text](img/Schematic_busT4adapter_xl.png "Schemat adaptera  bus-t4 ze zmodyfikowanym zasilaczem")
![alt text](img/IMG20230306201230.png "Gotowe urządzenie 2.0")

![alt text](img/hassio-bust4.png "Test działania komponentu bus-t4")


Do napędów Walky potrzebny jest schemat z transceiverem CAN. Złącze Bus-T4 jest ukryte pod zaślepką. Ten sam schemat nada się również do innych central sterujących z wyprowadzonym złączem rj-11 (6p4c), np. [mc824h](img/mc824h.jpg) lub [RBA3/C](img/rba3c.jpg)
![alt text](img/Schematic_bust4_2023-10-18.png "Schemat CAN + bus-t4")

Komponent obsługuje wysyłanie dowolnego polecenia do napędu przez usługę  ESPHome: nice_bust4_uart_raw_command w Home assystant.
```
SBS:   55 0c 00 03 00 81 01 05 86 01 82 01 64 e6 0c
Open:  55 0c 00 03 05 81 01 05 83 01 82 03 64 e4 0c
Close: 55 0c 00 03 05 81 01 05 83 01 82 04 64 e3 0c
Stop:  55 0c 00 03 00 81 01 05 86 01 82 02 64 e5 0c
```


Podczas uruchamiania i pracy ESP odpytuje urządzenia podłączone do magistrali BusT4 i wypisuje informacje o nich do logu. 
![log](img/log.png "Log")
![log](img/log2.png "Log2")

# Aktualizacje
* Dodano usługi w interfejsie komponentu ułatwiające uruchomienie procedury rozpoznawania długości skrzydła oraz procedury rozpoznawania urządzeń BlueBus bez rozbierania obudowy napędu (a nawet zdalnie).
* Dodano wypisywanie do logu konfiguracji odczytanych z urządzenia stanów L1, L2, L3 (Automatyczne
zamykanie, Zamknij po
fotokomórce, Zawsze zamykaj)
* Poprawiono kompatybilność z napędami DPRO924
* Przycisk STOP jest zawsze dostępny w User Interface obiektu
* Poprawiono kompatybilność z napędami Walky WL1024C
* Poprawiono kompatybilność z napędami Spin ([@TheGoblinHero](https://github.com/TheGoblinHero))
* Dodano funkcję ustawiania dowolnej pozycji napędu ([@TheGoblinHero](https://github.com/TheGoblinHero))

Jeśli projekt Cię zainteresował, możesz [postawić mi piwo lub kawę](https://www.tinkoff.ru/cf/12xvN3UtJkO)


