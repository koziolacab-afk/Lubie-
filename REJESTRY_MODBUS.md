# Rejestry Modbus w firmware

Stan mapowania: firmware 1.0.17 (`MelcoBEMS_SUPLA/MelcoBEMS_SUPLA.ino`).
Adresy ponizej to adresy przekazywane wprost do `readHoldingRegisters(adres, 1)`
(adresowanie od zera w ramce Modbus), a nie numeracja 4xxxx z niektorych
programow. Jezeli program konfiguracyjny numeruje rejestry od 1, sprawdz jego
konwencje adresowania przed wpisaniem tych wartosci.

Magistrala: RS-485, 9600 baud, 8N1; ESP32-S3 RX = GPIO 18, TX = GPIO 17.
Kazda pompa jest niezalezna:

| Slave ID | Pompa | Kanaly SUPLA powiazane z pompa |
| ---: | --- | --- |
| 1 | CAHV 1 | 0-5, 10-14, 43, 46-47, 66, 69 |
| 2 | CAHV 2 | 15-20, 24-28, 44, 48-49, 67, 70 |
| 3 | QAHV | 29-34, 38-42, 45, 50-51, 68, 71 |

Wszystkie wpisy w tabeli sa **odczytami holding register, funkcja 03**.
Interwal jest celem harmonogramu dla kazdej pompy osobno; rzeczywista
czestotliwosc zalezy od obciazenia magistrali i odpowiedzi urzadzen.

| Adres | Pompy | Interwal | Znaczenie / przeliczenie | Kanal SUPLA: CAHV 1 / CAHV 2 / QAHV |
| ---: | --- | ---: | --- | --- |
| 9 | wszystkie | 5 s | Kod bledu, surowa wartosc 16-bit | 5 / 20 / 34 |
| 25 | wszystkie | 5 s | System ON/OFF, surowa wartosc | 14 / 28 / 42 |
| 26 | wszystkie | 5 s | Tryb pracy, surowy kod | 11 / 25 / 39 |
| 30 | tylko QAHV | 60 s | Temperatura Thermo-off: `int16 / 100` °C | - / - / 45 |
| 67 | wszystkie | 5 s | Odszranianie, surowy kod | 10 / 24 / 38 |
| 73 | wszystkie | 5 s | Czestotliwosc HP: surowa wartosc w Hz | 3 / 18 / 32 |
| 85 | wszystkie | 5 s | Nastawa temperatury wylotu: `int16 / 100` °C | 13 / 27 / 41 |
| 99 | wszystkie | 5 s | Temperatura zewnetrzna: `int16 / 10` °C | 0 / 15 / 29 |
| 101 | wszystkie | 5 s | Temperatura wylotu wody: `int16 / 100` °C | 1 / 16 / 30 |
| 103 | wszystkie | 5 s | Temperatura wlotu wody: `int16 / 100` °C | 2 / 17 / 31 |
| 127 | wszystkie | 5 s | Praca pompy: `1` = ON, kazda inna wartosc = OFF | 4 / 19 / 33 |
| 136 | tylko CAHV | 60 s | Skladnik godzin pracy dodawany bez zmian | 43* / 44* / - |
| 137 | tylko CAHV | 60 s | Skladnik godzin pracy: setki godzin | 43* / 44* / - |

\* Kanaly 43 i 44 pokazuja wynik `rejestr 137 * 100 + rejestr 136` godzin,
gdy oba rejestry zostaly odczytane.

Kody prezentowane przez firmware: odszranianie (67): `0` Normal, `1` Standby,
`2` Defrost, `3` Wait restart. Tryb pracy (26): `0` Stop, `1` Hot water,
`2` Heating, `3` Cooling, `4` DHW contact, `5` Freeze stat, `6` Legionella,
`7` Heating Eco, `8` Mode 1, `9` Mode 2, `10` Mode 3, `11` Heat contact.
Kod bledu (9): `0x8000` jest opisywany jako OK, `0x6999` jako Comm error,
pozostale wartosci jako Fault; surowa liczba pozostaje widoczna w kanale.

## Sterowanie trybem CAHV

Kanaly SUPLA 69 i 70 zapisuja rejestr 26 funkcja Modbus 06 odpowiednio do
CAHV 1 i CAHV 2:

| Kanal SUPLA | Pompa | OFF | ON |
| ---: | --- | --- | --- |
| 69 | CAHV 1, slave 1 | `HR26 = 2` (Heating) | `HR26 = 7` (Heating Eco) |
| 70 | CAHV 2, slave 2 | `HR26 = 2` (Heating) | `HR26 = 7` (Heating Eco) |

Stan przelacznika jest synchronizowany z cyklicznym odczytem HR26. Przy
braku komunikacji albo bledzie funkcji 06 polecenie jest odrzucane, a kanal
wraca do ostatniego potwierdzonego stanu. Jezeli HR26 ma inny tryb, np.
Stop, kanal zachowuje ostatni rozpoznany wybor Heating/Heating Eco. Poczatkowy
stan kanalu to ON (Heating Eco), ale uruchomienie ESP nie wysyla zapisu.
Kazda zmiana kanalu powoduje tylko jedna probe zapisu; cyklicznie wykonywany
jest wylacznie odczyt kontrolny HR26.

## Sterowanie temperatura Thermo-off QAHV

Kanal SUPLA 71 (`QAHV Ustaw Thermo-off`) jest suwakiem temperatury CWU.
Akceptuje zakres 40,0-90,0 C i zaokragla nastawy do kroku 0,5 C. Po zmianie
suwaka firmware czeka 1 s na zakonczenie regulacji i wykonuje tylko jedna probe
zapisu funkcja Modbus 06:

| Kanal SUPLA | Pompa | Zapis |
| ---: | --- | --- |
| 71 | QAHV, slave 3 | `HR30 = temperatura C * 100` |

Uruchomienie ESP, cykliczny odczyt ani synchronizacja z SUPLA nie wysylaja
zapisu. Po bledzie komunikacji suwak wraca do ostatniej odczytanej wartosci.
Po udanym zapisie firmware zleca kontrolny odczyt HR30. Rzeczywista wartosc
odczytana z QAHV pozostaje w osobnym kanale termometru 45 (`QAHV Temperatura
Thermo-off`). Kanal udostepnia konfiguracje harmonogramu wymagana przez
aplikacje mobilna SUPLA, ale wybranie trybu Program nie zapisuje HR30.

## Wartosci pochodne

Te kanaly **nie sa osobnymi rejestrami Modbus**:

| Kanaly SUPLA: CAHV 1 / CAHV 2 / QAHV | Wartosc |
| --- | --- |
| 12 / 26 / 40 | Komunikacja Modbus: stan lacznosci z danym slave'em; przy braku odpowiedzi odpytywany jest rejestr 99 co 10 s |
| 46 / 48 / 50 | Delta temperatury: odczyt 101 minus odczyt 103, w °C |
| 47 / 49 / 51 | Zaobserwowane starty sprezarki: zliczane przejscia czestotliwosci rejestru 73 z zera na wartosc dodatnia; licznik przechowywany w NVS ESP32 |
| 66 / 67 / 68 | Szacowany czas pracy sprezarki: dodatnia czestotliwosc z rejestru 73; godziny sumowane od uruchomienia tej funkcji, zapisywane w NVS co 5 minut i po zatrzymaniu |

Czas jest liczony tylko przy swiezym odczycie czestotliwosci (do 30 s) i
polaczeniu z danym slave'em. Podczas zaniku komunikacji nie jest doliczany.
To szacunek ESP, a nie fabryczny licznik czasu pracy pompy. Rejestry 136/137
sa odczytywane jedynie dla CAHV, nie dla QAHV.

Kanaly 6-8, 21-23 i 35-37 (Firmware A1M, Licznik Modbus, Typ systemu)
sa zachowane w SUPLA dla zgodnosci identyfikatorow z istniejacym urzadzeniem,
ale **nie sa juz odpytywane**. Mozna je ukryc w SUPLA Cloud.

Pozostale kanaly nie pochodza z Modbus: 9 uruchamia OTA, 52-59 to osiem
czujnikow DS18B20, 60 to szacowana energia CWU, a 61-65 steruja piecioma
lokalnymi przekaznikami. Przelaczenie fizycznych przekaznikow 64 i 65 nadal
nie wysyla ramki Modbus; zapis HR26 realizuja wylacznie kanaly 69 i 70.
