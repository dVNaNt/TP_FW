# TunePulse Chunk API (src/api/tp_api.h)

## 1. Общие принципы

- Единица обмена — **чанк фиксированного размера 8 байт** (uint64). Это ровно один
  классический CAN-кадр, поэтому один и тот же код работает и с **CAN 2.0A**, и с
  **CAN-FD**, а также через USB CDC или любой другой байто-ориентированный транспорт.
- Сериализация **zero-copy**: `ApiChunk` — union, который одновременно даёт доступ
  к сырым байтам (`b8[8]`), целому слову (`u64`) и типизированной битовой структуре
  каждого чанка. Упаковка = заполнение полей структуры, распаковка = чтение полей.
  Ни memcpy, ни циклов сериализации.
- Транспорт не зашит в API: класс `TunePulseAPI` держит **два указателя на
  аппаратные колбэки**:
  - `ApiReadCb  (ApiChunk&) -> bool` — неблокирующе забирает один чанк из интерфейса;
  - `ApiWriteCb (const ApiChunk&)` — отправляет один чанк в интерфейс.
- Байт 0 каждого чанка — всегда CMD. Многобайтовые поля — little-endian.
  Суб-байтовые поля — C-битфилды, распределение битов от LSB.
- Реализация header-only: `src/api/tp_api.h`.

## 2. Спецификация (запрошенные чанки)

### 2.1 Device Info (R) — CMD 0x01

| Byte | Поле     | Тип    | Описание        |
| :--: | :------- | :----- | :-------------- |
| 0    | CMD      | uint8  | 0x01            |
| 1    | HW VER   | uint8  | Версия железа   |
| 2-3  | FW VER   | uint16 | Версия прошивки |
| 4-7  | Device   | uint32 | Код устройства  |

### 2.2 Device UID (R) — CMD 0x02

| Byte | Поле    | Тип    | Описание          |
| :--: | :------ | :----- | :---------------- |
| 0    | CMD     | uint8  | 0x02              |
| 1-7  | Respond | uint56 | UID устройства LE |

### 2.3 Device Status (R) — CMD 0x03

| Byte | Поле   | Тип    | Описание            |
| :--: | :----- | :----- | :------------------ |
| 0    | CMD    | uint8  | 0x03                |
| 1-3  | UpTime | uint24 | Время в секундах LE |
| 4-7  | Fault  | uint32 | Биты ошибок LE      |

### 2.4 Motor Status (RW) — CMD 0x04

| Byte | Поле             | Тип   | Описание                                          |
| :--: | :--------------- | :---- | :------------------------------------------------ |
| 0    | CMD              | uint8 | 0x04                                              |
| 1    | Mode             | 4 bit | Режим управления мотором (TBD)                    |
| 1    | Motor type       | 2 bit | 0: не определён, 1: STEPPER, 2: BLDC, 3: ERROR    |
| 1    | Phase Connection | 2 bit | 0: не подключён, 1: ABC, 2: CAB, 3: ACB           |
| 2    | Pole pairs       | uint8 | 0-255 пар полюсов                                 |
| 3    | Temp             | uint8 | °C, `255` = нет данных                            |
| 4-7  | Direction        | 1 bit | `0`: CW, `1`: CCW (бит 0), остальные — резерв     |

### 2.5 Electric Status (R) — CMD 0x05

| Byte | Поле       | Тип              | Описание                |
| :--: | :--------- | :--------------- | :---------------------- |
| 0    | CMD        | uint8            | 0x05                    |
| 1    | Efficiency | uint8            | `0..255` как 0…100 %    |
| 2-3  | Power      | bfloat16 (1.8.7) | Мгновенная мощность, Вт |
| 4-5  | Supply     | bfloat16 (1.8.7) | Напряжение питания, В   |
| 6-7  | Current    | bfloat16 (1.8.7) | Пиковый ток, А          |

### 2.6 Motion Status 1 (R) — CMD 0x06

| Byte | Поле       | Тип      | Описание                       |
| :--: | :--------- | :------- | :----------------------------- |
| 0    | CMD        | uint8    | 0x06                           |
| 1    | MotionMode | uint8    | Тип исполняемой команды (TBD)  |
| 2-7  | Position   | q1.31.16 | Реальная абсолютная позиция LE |

### 2.7 Motion Status 2 (R) — CMD 0x07

| Byte | Поле     | Тип              | Описание                 |
| :--: | :------- | :--------------- | :----------------------- |
| 0    | CMD      | uint8            | 0x07                     |
| 1-3  | Velocity | float24 (1.8.15) | Мгновенная скорость, RPS |
| 4-5  | Accel    | bfloat16 (1.8.7) | Ускорение, RPS/s         |
| 6-7  | Torque   | uint16           | Значение момента         |

### 2.8 Motion Status 3 (R) — CMD 0x08

| Byte | Поле      | Тип            | Описание                           |
| :--: | :-------- | :------------- | :--------------------------------- |
| 0    | CMD       | uint8          | 0x08                               |
| 1-3  | PosErr    | Q1.7.16        | Ошибка позиции, rot.angle LE       |
| 4-5  | VelErr    | bfloat16 1.8.7 | Ошибка скорости, RPS               |
| 6-7  | Queue     | 4 bit          | Количество оставшихся команд (TBD) |
| 6-7  | TimeScale | Q1.3.8, 12 bit | Множитель времени (256 = 1.0x)     |

### 2.9 Motor Properties (RW) — **CMD 0x09** (исправлено)

> **Внимание:** в черновике этот чанк имел CMD `0x06`, который уже занят
> Motion Status 1. Здесь он переназначен на **0x09**.

| Byte | Поле       | Тип              | Описание                  |
| :--: | :--------- | :--------------- | :------------------------ |
| 0    | CMD        | uint8            | 0x09                      |
| 1    | Temp Lim   | uint8            | Макс. температура, °C     |
| 2-3  | Resistance | bfloat16 (1.8.7) | Сопротивление обмотки, Ом |
| 4-5  | Inductance | bfloat16 (1.8.7) | Индуктивность, Гн         |
| 6-7  | Torque     | bfloat16 (1.8.7) | Момент двигателя, Н·м     |

### 2.10 HeartBeat (feedback от драйвера) — 4-битный CMD = 0b0111

Не запрос/ответ: драйвер рассылает его сам по своему расписанию
(`TunePulseAPI::send_heartbeat()`). Полная раскладка 64 бит, от младших:

| Биты    | Поле      | Тип    | Описание                                      |
| :-----: | :-------- | :----- | :-------------------------------------------- |
| 0-3     | CMD       | 4 bit  | 0b0111                                        |
| 4-7     | Beats     | uint4  | Инкрементальный счётчик heartbeat-пакетов     |
| 8-11    | CMDQ      | uint4  | Остаток motion-команд (с учётом направления)  |
| 12-23   | TimeScale | Q1.3.8 | Запрошенный governor'ом множитель времени     |
| 24-27   | Status    | uint4  | Статус драйвера: TBD                          |
| 28-31   | Fault     | uint4  | Группа ошибок: TBD                            |
| 32-35   | Warn      | 4 bit  | Группа предупреждений: TBD                    |
| 36-41   | Mode      | uint6  | Текущий режим исполнения: TBD                 |
| 42-47   | Error     | Q1.1.4 | Упрощённая ошибка следования (−2.0…+1.9375)   |
| 48-55   | Load      | int8   | Нагрузка мотора, `-128` = перегрузка          |
| 56-63   | IO        | uint8  | Endstop / probe / homing и т.д.               |

### 2.11 Правила RW-чанков и транспорт

- **Чтение:** мастер отправляет чанк с корректным CMD и нулевым payload —
  драйвер отвечает чанком с актуальным состоянием.
- **Запись (0x04, 0x09):** ненулевые поля запроса применяются к профилю
  мотора через штатные сеттеры lib, затем драйвер отвечает актуальным
  состоянием. Нулевые поля означают «не менять» (для bf16 — raw word == 0).
- **Heartbeat** не запрашивается — рассылается драйвером.
- Неизвестный CMD игнорируется молча.
- Каждый чанк укладывается в один CAN 2.0 кадр (DLC 8); CAN-FD
  «переваривает» его без изменений.

## 3. Привязка к lib и текущие заглушки (STUB)

| Чанк | Данные тянутся из lib (реализовано) | Заглушки (STUB) |
| :--- | :---------------------------------- | :-------------- |
| 0x01 | —                                   | Все поля (`hw_ver`, `fw_ver`, `device_code` — публичные поля API) |
| 0x02 | —                                   | UID (`uid[7]`)   |
| 0x03 | `fault` ← `TunePulseHeart::motor_status` | `uptime_s` (инкрементировать в app-цикле) |
| 0x04 | type/phase/poles/direction ← `Motor`; temp ← `Temperature`; mode ← `OperationStatus` | `mode` = статус жизненного цикла (enum управления TBD) |
| 0x05 | Power/Current ← `Power`; Supply ← `SupplyVoltage` | `efficiency` |
| 0x06 | —                                   | Весь чанк (`motion` struct) |
| 0x07 | —                                   | Весь чанк (`motion` struct) |
| 0x08 | —                                   | Весь чанк (`motion` struct) |
| 0x09 | R/L/Kt/TempLim ← `Motor` (RW через сеттеры) | — |
| HB   | TimeScale (Q10→Q1.3.8), Error (Q1.1.4) | Status/Fault/Warn/Mode/Load/IO — аргументы вызова |

Кодеки: `bfloat16` переиспользован из lib (`motor.h`); `float24 (1.8.15)`,
int24/int48, Q1.3.8, Q1.1.4 реализованы в `tp_api.h`.

## 4. Пример интеграции (CAN)

```cpp
#include "api/tp_api.h"   // после tunepulse.h (нужен CAN_BUS)

TunePulseAPI tp_api;                       // Движок протокола

void api_can_write(const ApiChunk& c) {    // TX-колбэк: ответ на 0x400+NodeID
    CAN_BUS.send(0x400u + TP_API_NODE_ID, c.b8, 8);
}

bool api_can_read(ApiChunk& c) {           // RX-колбэк: фильтр по 0x300+NodeID
    uint32_t id; uint8_t len; uint8_t buf[8];
    if (!CAN_BUS.receive(id, buf, len)) return false;
    if (id != 0x300u + TP_API_NODE_ID || len != 8) return false;
    __builtin_memcpy(c.b8, buf, 8);
    return true;
}

void setup() {
    tp_api.attach(api_can_read, api_can_write);  // Колбэки транспорта
    tp_api.bind(&TP);                            // Живые данные из lib
}

void loop() {
    tp_api.uptime_s = millis() / 1000;     // Питаем STUB-uptime
    tp_api.poll();                         // Запрос/ответ
    // Периодически: tp_api.send_heartbeat(beats++, queue, 1024, ...);
}
```

---

# 5. План API — предложения (НЕ входили в запрос)

Ниже — то, что уже есть в драйвере/lib и напрашивается в протокол, плюс
задел на будущее. Ничего из этого в `tp_api.h` пока не реализовано.

## 5.1 Command-чанки (управление, чего не хватает)

| CMD (предлож.) | Чанк | Обоснование |
| :--- | :--- | :--- |
| 0x10 | **Motion Command 1 (W)**: PVT-сегмент — `disp22 (q1.5.16) + vel24 + dur16` | В lib уже есть `MotionExecutor` + `HistoryBuffer` + `HermiteInterpolator`, но они не подключены к контроллеру (`TunePulseHeart::tick` хардкодит `target_pos = 0`). Порты описаны в `can_api.md` (Motion). |
| 0x11 | **Motion Command 2 (W)**: S-curve move — `vel24 + acc16 + jrk16` | `ScurveProfiler` реализован и генерирует сегменты; нужен только порт в чанк. |
| 0x12 | **Queue Control (W)**: clear / pause / resume / time-scale override | `MotionGovernor::set_time_scale()` уже реализован (Q10), нужен только транспорт. |
| 0x13 | **Control Mode (RW)**: torque/velocity/position/step-dir | `TunePulseHeart::is_pos_used` и `current_target_a` уже переключают режимы — нужен формализованный enum вместо TBD-поля `Mode` в 0x04. |
| 0x14 | **Calibration Control (W/R)**: запустить калибровку угла/сопротивления/индуктивности, статус прогресса | `AngleCalibrator` и `MotorCalibration` — полноценные конечные автоматы со стадиями (`MotorStatus` enum), идеально ложатся на чанк прогресса. |
| 0x15 | **Parameter Save/Load (W/R)**: persists мотор-профиля в flash | `Motor::service_tick()` / `load_tick()` / `save_tick()` уже умеют неблокирующую запись 8-словных блоков; чанк — только триггер + статус. |
| 0x16 | **System Control (W)**: enable/disable, reset faults, reboot (включая jump в DFU — `bootloader_jump.h` уже есть) | Критично для эксплуатации. |
| 0x17 | **Error/Warn detail (R)**: развёрнутые 32-битные fault/warn биты | Поля HeartBeat — урезанные 4-битные группы; нужен полный аналог `DriverStatus::not_ready` + флаги PHY. |
| 0x18 | **Node ID / Discover (RW)** | Протокол обнаружения из `can_api.md` (побайтовый арбитраж UID) реализован только на бумаге. |

## 5.2 Телеметрия, которую уже можно отдать

- **Encoder status (R)**: `Encoder` держит Q16.16 позицию с атомарным
  64-битным чтением и sequence-lock — прямой кандидат на чанк телеметрии.
- **Motor electrical (R)**: угол (`angle_el_`), `pwm_voltage_ab`,
  `current_target_a` — уже лежат в `TunePulseHeart`, но не отдаются.
- **I2tR / thermal budget (R)**: трекер из `can_api.md` (nominal/peak/I2t,
  корень на целочисленной математике) — ещё не реализован в lib; когда
  появится, отдаётся одним чанком (load, budget, scale).
- **FOC internals (R, debug)**: `MotorVectorSynthesis` хранит ошибки
  положения/скорости (`ep`, `vel_err`) — источник для 0x06–0x08 вместо
  текущих STUB-полей.

## 5.3 Механика протокола (будущее)

- **Мультичанк-ответы**: для таблиц калибровки (4096 значений) —
  стриминг с sequence-номером в CMD-байте (`0x20 | seq`).
- **CRC/sequence в HeartBeat**: 4-битный Beats уже позволяет мастеру
  детектировать потери; при необходимости — explicit ACK.
- **Timestamp sync**: broadcast-чанк с меткой времени мастера.
- **Bootloader over CAN**: чанки 0xF0–0xF7 (erase/write/verify/crc/run) —
  уже есть USB DFU, CAN-DFU закроет полевое обновление.
- **Bus-off / error counters (R)**: `CANDriver` уже считает
  `error_count`/`rx_overrun_count` атомарно — отдать диагностическим чанком.
- **Watchdog heartbeat master->slave**: тайм-аут → безопасное состояние
  (fault-механика в lib готова).

## 5.4 Технический долг текущей реализации

- Заполнение `TunePulseHeart::motor_status` не подтверждено во всех ветках
  `tick()` — перед продакшеном проверить актуальность значения для 0x03.
- Эффективность (0x05) требует механической мощности: `P_mech = omega * T`;
  нужен доступ к оценке скорости из `MotorVectorSynthesis`/observer.
- RW-чанк 0x04 меняет профиль мотора в рантайме без переподтверждения
  калибровки — при включении контроля целостности стоит потребовать
  повторной калибровки при смене type/poles.