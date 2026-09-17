# vibro-raw-wifi

Отдельная минимальная прошивка ESP32-C6: сырой LSM6 → Wi‑Fi **LoRa-1** / **10941090** → UDP **192.168.1.1:9500**.

Без LoRa / MQTT / ESP-NOW / calib — чтобы не ловить boot-loop на USB.

## Прошивка

```bat
BUILD.bat
FLASH.bat COMxx
```

или только flash из готового `release\`:

```bat
FLASH.bat COM27
```

В логе должны появиться строки `RAW: app_main`, `SPI OK`, `wifi…`, `got IP`.

## Приём на ПК

```bat
python ..\ml_vibration\raw_receiver.py --port 9500
```

## Версия

`0.4.0-rawwifi`
