# Загрузка прошивки через PlatformIO

## Использование python -m для работы с проектом

### Сборка проекта
```bash
python -m platformio run -e radix_release
```

### Загрузка прошивки
```bash
python -m platformio run -e radix_release -t upload
```

### Мониторинг Serial порта
```bash
python -m platformio device monitor -e radix_release
```

### Очистка проекта
```bash
python -m platformio run -e radix_release -t clean
```

### Проверка размера прошивки
```bash
python -m platformio run -e radix_release -t size
```

## Доступные окружения

- `cln17_v2_0_release` - CLN17 v2.0 (STM32G431CB)
- `radix_release` - Radix (STM32G431CB)

## Текущий размер прошивки (radix_release)

- **Flash**: 43,584 байт / 131,072 байт (33.3%)
- **RAM**: 5,628 байт / 32,768 байт (17.2%)

## Протоколы загрузки

По умолчанию используется **ST-Link** (`upload_protocol = stlink`).

### Загрузка на все платы через все ST-Link

В проект подключен скрипт `tools/upload_all_stlinks.py` (`extra_scripts = post:...`).
Команда `python -m platformio run -e <env> -t upload` автоматически:

- находит **все подключённые ST-Link** (Windows PnP, VID 0483, известные PID);
- прошивает **все платы параллельно** — по одному экземпляру OpenOCD на каждый ST-Link
  (привязка по серийному номеру через `hla_serial`);
- верифицирует прошивку и перезапускает платы (`verify reset`);
- возвращает ошибку, если хотя бы одна плата не прошилась.

Вывод каждого ST-Link помечается его серийным номером: `[066CFF...] OpenOCD output...`.

Список обнаруженных ST-Link без прошивки:
```bash
python tools/upload_all_stlinks.py --list
```

Прямой запуск (вручную, вне PlatformIO):
```bash
python tools/upload_all_stlinks.py .pio/build/cln17_v3_release/firmware.elf
```

Для использования DFU измените в `platformio.ini`:
```ini
upload_protocol = dfu
```

## Отладка

Для отладки через ST-Link:
```bash
python -m platformio debug -e radix_release
```

## Примечания

- Все команды используют `python -m platformio` для явного указания модуля Python
- Сборка выполняется в режиме debug с флагами `-O0` и `-save-temps`
- USB CDC включен для Serial коммуникации
