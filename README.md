# vibro-raw-wifi

Отдельная минимальная прошивка ESP32-C6: сырой LSM6 → Wi‑Fi **LoRa-1** / **10941090** → UDP **109.248.247.168:9500** (этот сервер).

Без LoRa / MQTT / ESP-NOW / calib — чтобы не ловить boot-loop на USB.

## Прошивка

Двойной клик по `FLASH.bat` — покажет COM-порты и спросит номер.
Или сразу:

```bat
FLASH.bat COM27
```

Окно не закрывается само (есть `pause`). В логе: `RAW: app_main`, `SPI OK`, `got IP`.

Сборка из исходников: `BUILD.bat`.

## Приём на ПК

```bat
python ..\ml_vibration\raw_receiver.py --port 9500
```

## Версия

`0.4.3-rawwifi`
