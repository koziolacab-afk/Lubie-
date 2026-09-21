# MelcoBEMS SUPLA

Firmware ESP32-S3 dla trzech niezaleznych pomp MelcoBEMS: CAHV 1, CAHV 2 i QAHV.

- [Rejestry Modbus i kanaly SUPLA](REJESTRY_MODBUS.md)
- Szkic: [`MelcoBEMS_SUPLA/MelcoBEMS_SUPLA.ino`](MelcoBEMS_SUPLA/MelcoBEMS_SUPLA.ino)

## Tryb konfiguracji Wi-Fi

Po normalnym uruchomieniu urzadzenia przytrzymaj przycisk **BOOT** przez 5
sekund. Firmware przejdzie do trybu konfiguracji SUPLA i uruchomi punkt
dostepowy `KOTLOWNIA-3-POMPY-...`. Krotkie nacisniecie BOOT niczego nie robi,
a wejscie do trybu AP nie kasuje dotychczasowych ustawien.

Nie trzymaj BOOT podczas resetowania ani wlaczania zasilania, poniewaz GPIO 0
jest rowniez pinem wyboru trybu programowania ESP32-S3.
