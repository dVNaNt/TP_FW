Напиши C++ метод который будет контролировать I2TR с этими 3 параметрами. Внутрь задается три параметра лимита мощности (nominal mw, peak mw, i2tR mwt), сопротивление обмотки (миллиом), коэффициент нормализации тока (норм->ma) и частота вызова в герцах, четко переодично.



Должен быть метод tick который получает на вход значение токов в координатах AB (нормализованный q1.15) и тип мотора (если влияет). На выходе рассчитывает мощность и доступный диапазон мощности для системы и третий параметр - допустимый current scale (должно быть супер эффективное взятие корня МАКСИМАЛЬНО БЫСТРО. Делай на целочисленной математике там где возможно и битовых сдвигах, должно быть супер эффективно 

### Discover mode

 **1. Инициализация режима**

- **Мастер → Broadcast** (`ID: 0x100`, `DLC: 0`)

	Переводит все неназначенные слейвы в режим адресации. Слейвы сбрасывают текущее состояние поиска и готовятся к побайтовому перебору UID.

 **2. Побайтовый поиск по дереву UID**

Мастер изолирует 64-битный UID побайтово ($i = 0 \dots 7$). На каждой итерации Мастер отправляет уже накопленный префикс, чтобы отсеять несовпадающие узлы:

- **Мастер → Broadcast** (`ID: 0x101`, `DLC: i`, `Data: [UID[0] ... UID[i-1]]`)

	*Примечание: Для $i = 0$ DLC равен 0, данные пустые.*

- **Слейвы → Мастер** (`ID: 0x400 | UID[i]`, `DLC: 0`)

	Отвечают только те слейвы, чей префикс `UID[0 ... i-1]` полностью совпал с данными от Мастера. Они отправляют свой следующий байт (`UID[i]`), зашитый в CAN ID.

	- **Важное аппаратное требование:** Для фреймов ответа `0x4xx` на стороне слейвов должен быть строго включен режим **DAR/NART** (отключение автоматического повтора передачи). Слейвы, проигравшие аппаратный арбитраж CAN, не должны пытаться переотправить кадр, а обязаны очистить TX-буфер и ждать следующего запроса `0x101`.

		Аппаратный арбитраж CAN автоматически определяет наименьшее значение `UID[i]`. Мастер сохраняет выигравший байт и переходит к шагу $i+1$.

 **3. Изоляция узла**

- **Мастер → Broadcast** (`ID: 0x101`, `DLC: 8`, `Data: [Полный 64-битный UID]`)

	После успешного сбора всех 8 байт Мастер отправляет полный 64-битный UID для финальной изоляции выигравшего устройства.

- **Слейв → Мастер** (`ID: 0x102`, `DLC: 8`, `Data: [Полный 64-битный UID]`)

	Совпавший слейв подтверждает изоляцию, отправляя эхо своего UID, и временно переходит в режим ожидания (Standby), чтобы не участвовать в последующих циклах поиска остальных устройств.

 **4. Назначение адреса**

Для назначения адреса используется диапазон `0x300-0x3FF`. Целевой адрес узла (`Node_ID` от 0 до 255) закладывается прямо в CAN ID, что позволяет передавать полный 64-битный UID в поле данных и защищает от конфликтов при совпадении первых 56 бит. В будущем этот же ID (`0x300 + Node_ID`) используется для отправки рабочих команд данному узлу.

- **Мастер → Целевой слейв** (`ID: 0x300 + Node_ID`, `DLC: 8`, `Data: [Полный 64-битный UID]`)

	Мастер отправляет полный UID устройства на вычисленный CAN ID. Слейв принимает этот кадр (в специальном режиме конфигурации), валидирует UID и сохраняет `Node_ID` из CAN ID в качестве своего постоянного адреса.

- **Слейв → Мастер** (`ID: 0x400 + Node_ID`, `DLC: 8`, `Data: [Полный 64-битный UID]`)

	Слейв записывает адрес в энергонезависимую память (Flash/EEPROM), обновляет аппаратные маски фильтров для работы в обычном режиме и отправляет финальное эхо на ID: 0x400 + Node_ID. 

**5. Завершение определения**

Мастер отправлят команду 0x103, после чего все устройства переходят в нормальный режим. Если устройству не назначен адрес - оно просто игнорирует любые сообщения в цепи и не отправляет ничего.



### Motion (master)

**ID: 0x300-0x3FF**. Base 0x300 + driver Address 0x00…0xFF. 0xFF is broadcast to ALL

#### Burst mode (atomic command)

##### PVT motion (variable duration)

**ID: 0x3xx; DLC: 8**

|   Name   |       Type       |  Size  |                  Description                   |
| :------: | :--------------: | :----: | :--------------------------------------------: |
| `burst`  |       bool       | 1 bit  |                    Always 1                    |
| `constT` |       bool       | 1 bit  |          Always 0 (variable duration)          |
| `disp22` |     q1.5.16      | 22 bit | 6bit signed rotations + 16bit angle resolution |
| `vel24`  | float24 (1.8.15) | 24 bit |        Velocity in rotations per second        |
| `dur16`  |      uint16      | 16 bit |      Duration in 10<sup>-4</sup> seconds       |

##### PVcTF motion (constant duration + force control)

**ID: 0x3xx; DLC: 8**

|   Name    |       Type       |  Size  |                  Description                   |
| :-------: | :--------------: | :----: | :--------------------------------------------: |
|  `burst`  |       bool       | 1 bit  |                    Always 1                    |
| `constT`  |       bool       | 1 bit  |          Always 1 (constant duration)          |
| `disp22`  |     q1.5.16      | 22 bit | 6bit signed rotations + 16bit angle resolution |
|  `vel24`  | float24 (1.8.15) | 24 bit |        Velocity in rotations per second        |
| `force16` |      uint16      | 16 bit |        Normalized force during segment         |

##### Motion ABSOLUTE Limits

|  Name   |       Type       |  Size  |          Description           |
| :-----: | :--------------: | :----: | :----------------------------: |
| `isPVT` |       bool       | 1 bit  |            Always 0            |
|  `cmd`  |      uint7       | 7 bit  |              0x7D              |
| `vel24` | float24 (1.8.15) | 24 bit |     Velocity limit in RPS      |
| `acc16` | bfloat16 (1.8.7) | 16 bit | Accel limit in RPS<sup>2</sup> |
| `jrk16` | bfloat16 (1.8.7) | 16 bit | Jerk limit in RPS<sup>3</sup>  |

##### Point to point motion (advanced) 1:

|     Name      |       Type       |  Size  | Description                                                  |
| :-----------: | :--------------: | :----: | :----------------------------------------------------------- |
|     `cmd`     |      uint4       | 4 bit  | 0x7                                                          |
|    `mode`     |      uint4       | 4 bit  | Motion mode:<br />`0`: HOLD - keep position<br />`1`: FREERUN - const vel/torq<br />`2`: FREERUN_LERP - variable vel/torq<br />`3`: FREEWHEEL - zero load rotation<br />`4`: BRAKE - braking control<br />`5`: S-CURVE - ramp point to point motion<br />`6`: PVT - interpolation motion<br />`7`: ANCHOR - setting new position<br />`8`: LIMIT - setting limit values<br />`9`: HOMING<br />`10`: STEP-DIR<br />`F`: RESET - clear buffer and HOLD |
|  `torq_trig`  |       bool       | 2 bit  | Is torque trigger used                                       |
|  `time_trig`  |       bool       | 1 bit  | Is time trigger used                                         |
|  `pos_trig`   |       bool       | 1 bit  | Is pos trigger used                                          |
|   `io_trig`   |       bool       | 1 bit  | Is IO trigger used                                           |
|    `isDur`    |       bool       | 1 bit  | Is duration used                                             |
|    `isAbs`    |       bool       | 1 bit  | `0`: Relative position<br />`1`: Absolute position           |
|    `isVel`    |       bool       | 1 bit  |                                                              |
|    `isAcc`    |       bool       | 1 bit  |                                                              |
|    `isJrk`    |       bool       | 1 bit  |                                                              |
|    `isTrq`    |       bool       | 1 bit  |                                                              |
|               |                  |        |                                                              |
| `io_mask_pos` |      uint8       | 8 bit  | IO trig mask                                                 |
| `io_mask_neg` |      uint8       | 8 bit  | IO trig mask                                                 |
|    `trq16`    | bfloat16 (1.8.7) | 16 bit | Normalized force during segment                              |







### Feedback (slave)

**ID: 0x400-0x4FF**. Base 0x400 + driver Address 0x00…0xFF

Base packet:

|  Name   |  Type  |  Size  |                   Description                   |
| :-----: | :----: | :----: | :---------------------------------------------: |
|   CMD   | uint8  | 8 bit  |                 Respond command                 |
| Respond | uint56 | 56 bit |                  Respond value                  |
|    N    | uint8  | 8 bit  | Number of commands received since last callback |




Device Info (R):

|  Name  |  Type  |  Size  |   Description    |
| :----: | :----: | :----: | :--------------: |
|  CMD   | uint8  | 8 bit  |       0x01       |
| HW VER | uint8  | 8 bit  | Hardware version |
| FW VER | uint16 | 16 bit | Firmware version |
| Device | uint32 | 32 bit |   Device code    |

Device UID (R):

|  Name   |  Type  |  Size  |   Description    |
| :-----: | :----: | :----: | :--------------: |
|   CMD   | uint8  | 8 bit  |       0x02       |
| Respond | uint56 | 56 bit | Device UID 56bit |

Device Status (R):

|  Name  |  Type  |  Size  | Description |
| :----: | :----: | :----: | :---------: |
|  CMD   | uint8  | 8 bit  |    0x03     |
| UpTime | uint24 | 24 bit |  Time in s  |
| Fault  | uint32 | 32 bit | Error bits  |

Motor Status (RW):

|       Name       | Type  |  Size  |                      Description                      |
| :--------------: | :---: | :----: | :---------------------------------------------------: |
|       CMD        | uint8 | 8 bit  |                         0x04                          |
|       Mode       |  int  | 4 bit  |               Motor control mode (TBD)                |
|    Motor type    |  int  | 2 bit  | `0`: Not defined, `1`: STEPPER; `2`: BLDC; `3`: ERROR |
| Phase Connection |  int  | 2 bit  |   `0`: Not connected, `1`: ABC; `2`: CAB; `3`: ACB    |
|    Pole pairs    | uint8 | 8 bit  |                      0-256 poles                      |
|       Temp       | uint8 | 8 bit  |            Degrees Celsius. `255`: no info            |
|    Direction     | bool  | 1 bit  |         Rotation direction `0`: CW; `1`: CCW          |
|        ?         |       | 31 bit |                                                       |

Electric Status (R):

|    Name    |       Type       |  Size  |         Description          |
| :--------: | :--------------: | :----: | :--------------------------: |
|    CMD     |      uint8       | 8 bit  |             0x05             |
| Efficiency |      uint8       | 8 bit  |     `0...256` as 0…100%      |
|   Power    | bfloat16 (1.8.7) | 16 bit | Motor instant power in watts |
|   Supply   | bfloat16 (1.8.7) | 16 bit |    Driver supply voltage     |
|  Current   | bfloat16 (1.8.7) | 16 bit |  Motor instant peak current  |

Motion Status 1:

|    Name    |   Type   |  Size  |        Description        |
| :--------: | :------: | :----: | :-----------------------: |
|    CMD     |  uint8   | 8 bit  |           0x06            |
| MotionMode |  uint8   | 8 bit  | Type of execution command |
|  Position  | q1.31.16 | 48 bit |  Real absolute position   |

Motion Status 2:

|   Name   |       Type       |  Size  |     Description      |
| :------: | :--------------: | :----: | :------------------: |
|   CMD    |      uint8       | 8 bit  |         0x07         |
| Velocity | float24 (1.8.15) | 24 bit | Instant velocity RPS |
|  Accel   | bfloat16 (1.8.7) | 16 bit |     Accel in RPS     |
|  Torque  |      uint16      | 16 bit |     Torque value     |

Motion Status 3:

|   Name    |       Type       |  Size  |            Description            |
| :-------: | :--------------: | :----: | :-------------------------------: |
|    CMD    |      uint8       | 8 bit  |               0x08                |
|  PosErr   |     Q1.7.16      | 24 bit | Position error in rotations.angle |
|  VelErr   | bfloat16 (1.8.7) | 16 bit |       Velocity error in RPS       |
|   Queue   |       int        | 4 bit  |   Amount of remaining commands    |
| TimeScale |      Q1.3.8      | 12 bit |  Motion control time multiplier   |





Motor properties (RW):

|    Name    |       Type       |  Size  |          Description           |
| :--------: | :--------------: | :----: | :----------------------------: |
|    CMD     |      uint8       | 8 bit  |              0x06              |
|  Temp Lim  |      uint8       | 8 bit  | Maximum temperature in Celsius |
| Resistance | bfloat16 (1.8.7) | 16 bit |    Motor resistance in Ohms    |
| Inductance | bfloat16 (1.8.7) | 16 bit |   Motor inductance in Henry    |
|   Torque   | bfloat16 (1.8.7) | 16 bit |     Motor torque value NM      |





HeartBeat status (feedback from driver):

|   Name    |  Type  |  Size  |                        Description                        |
| :-------: | :----: | :----: | :-------------------------------------------------------: |
|    CMD    | uint2  | 4 bit  |                          0b0111                           |
|   Beats   | uint4  | 4 bit  |           Incrementing each heart beat package            |
|   CMDQ    | uint4  | 4 bit  | Amount of remaining motion commands (direction dependent) |
| TimeScale | Q1.3.8 | 12 bit |         Motion governor requested time multiplier         |
|  Status   | uint4  | 4 bit  |                    Driver status: TBD                     |
|   Fault   | uint4  | 4 bit  |                     Fault group: TBD                      |
|   Warn    |  bool  | 4 bit  |                    Warning group: TBD                     |
|   Mode    | uint6  | 6 bit  |                Current execution mode: TBD                |
|   Error   | Q1.1.4 | 6 bit  |       Position/velocity following error simplified        |
|   Load    |  int8  | 8 bit  |           Motor load s. `-128`: motor overload            |
|    IO     | uint8  | 8 bit  |                 Endstop/probe/homing/etc                  |





Для G431 сделай файл CAN.h который будет реализовывать управление CAN контроллером в обоих режимах (CAN и CAN-FD (дефолт но с поддержкой CAN 2.0A по пакетам и на чтение и на отправку)). Поля должны автоматически вычислятся. ID всегда 11 бит. Mode всегда normal. Скорость начала и середины задается независимо, для всего контроллера. Сделай структуру данных:

ID, payload + payload size (хз как лучше сделать без векторов) в идеале так что бы было супер компактно, can-fd/can2.0A bool (влияет на размер пакета за раз, ускорение передачи итд), auto-retransmition (0 - нет, либо заданная пауза)

Так же добавь возможность сделать фильтры и прочее. Максимально эффективно по скорости и памяти. Добавь прерывания по получению новых файлов. Убедись что правильно настроен clock тактирование итд. Сделай пресеты для стандартных скоростей CAN. Убедись что корректна инициализация и прочее. 



