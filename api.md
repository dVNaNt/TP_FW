# TunePulse API

Application protocol for TunePulse drives on CAN 2.0A and CAN-FD.

---

## 1. Design rules

1. Every frame is exactly 8 bytes. That is the whole payload of a classic
	CAN 2.0A frame, so the protocol runs unchanged on a classic bus. On a CAN-FD
	link the same 8-byte frames are sent as FD frames; payloads of 9–64 bytes
	can be used in multiple register read/write or burst mode. UART/SPI/I2C must have additional CRC check for safety
5. A node without an address is silent. It accepts network management only
	and transmits nothing until an address is assigned.

---

## 2. Identifier map

11-bit standard identifiers only. Lower identifier = higher bus priority. The
map is ordered so a tripping drive outranks everything, network management comes
next, commands outrank responses, and status telemetry can never delay a command.

| Range         | Direction     | Purpose                                      |
| :------------ | :------------ | :------------------------------------------- |
| 0x000 – 0x1FF | —             | Reserved                                     |
| 0x100 + node  | node → bus    | Emergency / fault report (highest priority)  |
| 0x200 – 0x2FF | —             | Reserved                                     |
| 0x300 + node  | master → node | Real-time setpoint stream, flashing          |
| 0x400 + node  | node → master | Response channel, Discovery byte arbitration |
| 0x500 + node  | node → bus    | PulseBeat and cyclic telemetry               |
| 0x600 – 0x7FF | —             | Reserved                                     |

Node addresses: 0 = broadcast, 1–254 = assigned, 255 = unassigned/discovery.
0x2FF and 0x3FF therefore address every node at once.

### Hardware filtering

A node programs 3 FDCAN standard filter slots for master device:

| Slot | Type | Accepts | Scenario             |
| :--- | :--- | :------ | :------------------- |
| 1    | mask | 0x1xx   | Receive slave faults |
| 2    | mask | 0x4xx   | Slave responses      |
| 3    | mask | 0x5xx   | Telemetry            |

A node programs FDCAN standard filter slot for slave devices:

| Slot | Type | Accepts             | Active when      |
| :--- | :--- | :------------------ | :--------------- |
| 1    | dual | 0x300 + node, 0x3FF | address assigned |
| 2    | dual | 0x400 + node, 0x4FF | address assigned |

Everything else is rejected in hardware: traffic addressed to other drives costs
this node no interrupts at all. Non-matching standard frames, extended frames
and remote frames are all rejected globally.

## CMD Group Map

| Bit7 | Bit6 | Bit5 | Bit0-4 HEX  |          Type           | Qty  |
| :--: | :--: | :--: | :---------: | :---------------------: | :--: |
|  x   |  0   |  x   | 0x00 – 0x3F |  Read/Write registers   |  64  |
|  0   |  1   |  x   | 0x00 – 0x3F |   Read only registers   |  64  |
|  1   |  1   |  0   | 0x00 - 0x1F |  Write only registers   |  32  |
|  1   |  1   |  1   |     N/A     | Burst mode (write only) |  1   |

---

### 0x80…0xFF: Burst mode

Used for extremely efficient and dense data master->slave data streaming for very fast motion. All data needed to precise timed axis-synchronous motion in single chank

Position is mask of q1.23.24 absolute position, where mask takes only sign, 5 LSB of rotations (max 32) and 16 MSB of angle. Each segment should be less than 8 rotations, less = better. Never use relative mode here to avoid accumulative error for missed packets. 

For PVT mode time step between messages should be less than 8s. Iess = better. Time is (system time [us] / 250) % 2<sup>21</sup>. Should be not relative/duration but LSBs of system tick to avoid fault in case of missed packet

#### PVT mode

Target position and velocity at time

|  Name  |  Type   |  Size  |                      Description                      |
| :----: | :-----: | :----: | :---------------------------------------------------: |
|   RW   |  bool   | 1 bit  |                       Always 1                        |
| burst  |  uint2  | 2 bit  |                       Always 11                       |
| time21 | uint21  | 21 bit |                   Time mask [250us]                   |
| vel18  | float16 | 16 bit |                    Velocity [RPS]                     |
| pos24  | q1.7.16 | 24 bit | 7bit signed rotations + 16bit angle resolution (mask) |

#### PVcTF mode

Constant update frequency with controllable position, velocity and torque

|  Name   |   Type   |  Size  |                      Description                      |
| :-----: | :------: | :----: | :---------------------------------------------------: |
|   RW    |   bool   | 1 bit  |                       Always 1                        |
|  burst  |  uint2   | 2 bit  |                       Always 11                       |
|  frame  |  uint5   | 5 bit  |                 Every next message++                  |
| force16 | bfloat16 | 16 bit |                  Segment force [Nm]                   |
|  vel16  | bfloat16 | 16 bit |                    Velocity [RPS]                     |
|  pos24  | q1.7.16  | 24 bit | 7bit signed rotations + 16bit angle resolution (mask) |

### 0x40…0x7F: Read Only registers

#### 0x40: Batch read request command (R) - master only

Command used to get batch of registers with short request to avoid bus jam

|   Name    |   Type   |  Size  |                       Description                        |
| :-------: | :------: | :----: | :------------------------------------------------------: |
|    CMD    |  uint8   | 8 bit  |                           0x40                           |
| Registers | uint7[8] | 56 bit | List of registers to read, 0x40 as filler of empty space |

> If 0x40 stays in the end of message -> it is ignored. If it is used once between 2 registers, it counts as range. E.g [0x01 0x40 0x08 0x40 0x40 0x40 0x40 0x40] will return read of 0x01 0x02 0x03 0x04 0x05 0x06 0x07 and 0x08 registers

#### 0x41: Range read request command with subparams (R) - master only

Command used to get batch of registers with short request to avoid bus jam

|   Name    |  Type  |  Size  |        Description        |
| :-------: | :----: | :----: | :-----------------------: |
|    CMD    | uint8  | 8 bit  |           0x41            |
|  ReadCMD  | uint8  | 8 bit  |        CMD to read        |
| AddrStart | uint24 | 24 bit | Start read address/SubCMD |
|  AddrEnd  | uint24 | 24 bit |  End read address/SubCMD  |

#### 0x42-0x4F: SubCMD space

##### 0x01??: System Currents DQ

|  Name  |   Type   |  Size  | Description          |
| :----: | :------: | :----: | :------------------- |
|  CMD   |  uint8   | 8 bit  | 0x42                 |
| SubCMD |  uint8   | 8 bit  | 0x??                 |
|  Isup  | bfloat16 | 16 bit | Supply usage current |
|   Iq   | bfloat16 | 16 bit | Q-Axis current [A]   |
|   Id   | bfloat16 | 16 bit | D-Axis current [A]   |

##### 0x01??: System Currents AB

|  Name  |   Type   |  Size  | Description          |
| :----: | :------: | :----: | :------------------- |
|  CMD   |  uint8   | 8 bit  | 0x42                 |
| SubCMD |  uint8   | 8 bit  | 0x??                 |
|  Isup  | bfloat16 | 16 bit | Supply usage current |
|   Ia   | bfloat16 | 16 bit | Q-Axis current [A]   |
|   Ib   | bfloat16 | 16 bit | D-Axis current [A]   |

#### 0x50: Device code & firmware version (R)

Hardware specific data

|  Name  |  Type  |  Size  |   Description    |
| :----: | :----: | :----: | :--------------: |
|  CMD   | uint8  | 8 bit  |       0x50       |
| HW VER | uint8  | 8 bit  | Hardware version |
| FW VER | uint16 | 16 bit | Firmware version |
| Device | uint32 | 32 bit |   Device code    |

#### 0x51: System Status (R)

Quick system status

|  Name  |   Type   |  Size  |       Description        |
| :----: | :------: | :----: | :----------------------: |
|  CMD   |  uint8   | 8 bit  |           0x51           |
|   IO   |  uint8   | 8 bit  | Endstop/probe/homing/etc |
| UpTime | bfloat16 | 16 bit |         Time [s]         |
| Fault  |  uint32  | 32 bit |        Error bits        |

#### 0x52: Motor status (R)

Instant status of the motor

|    Name    |       Type       |  Size  |                         Description                          |
| :--------: | :--------------: | :----: | :----------------------------------------------------------: |
|    CMD     |      uint8       | 8 bit  |                             0x52                             |
|   Status   |      uint8       | 8 bit  |                             TBD                              |
|  Reserved  |      uint8       | 8 bit  |                             TBD                              |
| Motor temp |      uint8       | 8 bit  | Motor temperature in Celsius. 0 - data missing, 255 - overtemperature |
|   Power    | bfloat16 (1.8.7) | 16 bit |                 Motor instant power in watts                 |
|   Torque   | bfloat16 (1.8.7) | 16 bit |                     Motor instant torque                     |

#### 0x45: Driver Status (R)

Instant status of the power stage

|    Name     |       Type       |  Size  |                         Description                          |
| :---------: | :--------------: | :----: | :----------------------------------------------------------: |
|     CMD     |      uint8       | 8 bit  |                             0x45                             |
|   Status    |      uint8       | 8 bit  |                             TBD                              |
|  Reserved   |       N/A        | 16 bit |                             TBD                              |
| Driver temp |      uint8       | 8 bit  | Driver Temperature in Celsius. 0 - data missing, 255 - overtemperature |
|    Usage    |       int8       | 8 bit  | Consolidated usage of system/motor current/voltage/power limits. -128...127 as -100…100%, negative range as regen |
|   Supply    | bfloat16 (1.8.7) | 16 bit |                    Driver supply voltage                     |

#### 0x53: Motion Status 1 (R)

Motion path following status

|   Name    |       Type       |  Size  |            Description            |
| :-------: | :--------------: | :----: | :-------------------------------: |
|    CMD    |      uint8       | 8 bit  |               0x53                |
|  PosErr   |     Q1.7.16      | 24 bit | Position error in rotations.angle |
|  VelErr   | bfloat16 (1.8.7) | 16 bit |       Velocity error in RPS       |
|   Queue   |       int        | 4 bit  |   Amount of remaining commands    |
| TimeScale |      Q1.3.8      | 12 bit |  Motion control time multiplier   |

#### 0x54: Motion Status 2 (R)

Motion command status

|    Name    |   Type   |  Size  |        Description        |
| :--------: | :------: | :----: | :-----------------------: |
|    CMD     |  uint8   | 8 bit  |           0x54            |
| MotionMode |  uint8   | 8 bit  | Type of execution command |
|  Position  | q1.23.24 | 48 bit |  Real absolute position   |

#### 0x55: Motion Status 3 (R)

Motion instant status

|   Name   |       Type       |  Size  |     Description      |
| :------: | :--------------: | :----: | :------------------: |
|   CMD    |      uint8       | 8 bit  |         0x55         |
| Velocity | float24 (1.8.15) | 24 bit | Instant velocity RPS |
|  Accel   | bfloat16 (1.8.7) | 16 bit |     Accel in RPS     |
|  Torque  | bfloat16 (1.8.7) | 16 bit |     Torque value     |

#### 0x56: Self-Detected sensorless homing travel (R)

|   Name   |   Type   |  Size  | Description                |
| :------: | :------: | :----: | :------------------------- |
|   CMD    |  uint8   | 8 bit  | 0x56                       |
| Detected | q1.23.32 | 56 bit | Detected full range motion |

#### 0x57: Absolute Encoder Internal position (R)

|   Name   |   Type   |  Size  | Description           |
| :------: | :------: | :----: | :-------------------- |
|   CMD    |  uint8   | 8 bit  | 0x57                  |
| Position | q1.23.32 | 56 bit | Absolute internal pos |



#### 0x7x: PulseBeat status (R)

Regular feedback from driver with key status

|   Name    |  Type  |  Size  |                         Description                          |
| :-------: | :----: | :----: | :----------------------------------------------------------: |
|    CMD    | uint4  | 4 bit  |                            0b0111                            |
|   Beats   | uint4  | 4 bit  |             Incrementing each heart beat package             |
|   CMDQ    | uint4  | 4 bit  | Amount of remaining MT motion commands (direction dependent) |
| TimeScale | Q1.3.8 | 12 bit |          Motion governor requested time multiplier           |
|  Status   | uint4  | 4 bit  |                      Driver status: TBD                      |
|   Fault   | uint4  | 4 bit  |                       Fault group: TBD                       |
|   Warn    | uint4  | 4 bit  |                      Warning group: TBD                      |
|   Mode    | uint6  | 6 bit  |                 Current execution mode: TBD                  |
|   Error   | Q1.1.4 | 6 bit  |         Position/velocity following error simplified         |
|   Load    |  int8  | 8 bit  |        Motor load incl derating. -128: motor overload        |
|    IO     | uint8  | 8 bit  |                   Endstop/probe/homing/etc                   |

### 0xC0-0xDF: Write Only Registers

#### 0xC0-0xC3: Motion control:

SegmentID = 0 means there only single packet. May be used for Cyclic Synchronous Position (CSP) Mode, Cyclic Synchronous Velocity (CSV) Mode or Cyclic Synchronous Torque (CST) Mode. If time sync needed - you need MT1 sent as well as MT2 (for CSP) or MT3 (for CSV or CST). MT2 can be send independently (for pos only setpoint), MT3 same (for velocity or torque setpoint). If more than one MT command needs to be sent, MT1 is mandatory and SegmentID should be same for every command of this task, so SegmentID(MT1)=SegmentID(MT2)=SegmentID(MT3)=SegmentID(MT4)!=0. SegmentID ideally should be incrementing value to avoid any missing packets reconstruction collisions. 

AxisID is used to apply same path to multiple axis in same time by broadcasting all segment part-commands. Then drivers with same AxisID will execute this command.

##### 0xC0: Motion Task 1

|   Name    |  Type  |  Size  | Description                                                  |
| :-------: | :----: | :----: | :----------------------------------------------------------- |
|   `cmd`   | uint8  | 8 bit  | 0xC0                                                         |
| SegmentID | uint8  | 8 bit  | Unique SegmentID to ensure integrity in case of missing packets<br />`0`: Single frame |
|  AxisID   | uint8  | 8 bit  | Axis of ID to make possible broadcast same commands to few drivers |
|   Mode    | uint4  | 4 bit  | Motion mode:<br />`0`: HOLD - keep position<br />`1`: FREERUN - const vel/torq<br />`2`: FREERUN_LERP - variable vel/torq<br />`3`: FREEWHEEL - zero load rotation<br />`4`: BRAKE - braking control<br />`5`: S-CURVE - ramp point to point motion<br />`6`: PVT - interpolation motion<br />`7`: ANCHOR - setting new position<br />`8`: LIMIT - setting limit values<br />`9`: HOMING<br />`10`: STEP-DIR<br />`F`: RESET - clear buffer and HOLD |
|   isAbs   |  bool  | 1 bit  | `0`: Relative position<br />`1`: Absolute position           |
|  PosMode  | uint2  | 2 bit  | `0`: Not used<br />`1`: Pos target<br />`2`: Pos trigger     |
| TorqMode  | uint2  | 2 bit  | `0`: Not used<br />`1`: Torque limit<br />`2`: Torque trigger |
| TimeMode  | uint2  | 2 bit  | `0`: Not used<br />`1`: Duration target<br />`2`: Duration trigger |
| `io_trig` |  bool  | 1 bit  | Is IO trigger used                                           |
|  `isVel`  |  bool  | 1 bit  | Is velocity control used: `0`: Unused, `1`: Used             |
|  `isAcc`  |  bool  | 1 bit  | Is accel control used: `0`: Unused, `1`: Used                |
|  `isJrk`  |  bool  | 1 bit  | Is jerk control used: `0`: Unused, `1`: Used                 |
|    Res    |  bool  | 1 bit  | Reserved                                                     |
|   Time    | uint24 | 24 bit | Duration or time trigger in [100us]                          |

##### 0xC1: Motion Task 2

|   Name    |   Type   | Size  | Description                                                  |
| :-------: | :------: | :---: | :----------------------------------------------------------- |
|   `cmd`   |  uint8   | 8 bit | 0xC1                                                         |
| SegmentID |  uint8   | 8 bit | Same SegmentID as Motion Task 1<br />`0`: Single frame, absolute pos is used |
|    Pos    | q1.23.24 | 48bit | Absolute position or Displacement in [rev]                   |

##### 0xC2: Motion Task 3

|   Name    |   Type   |  Size  | Description                                            |
| :-------: | :------: | :----: | :----------------------------------------------------- |
|   `cmd`   |  uint8   | 8 bit  | 0xC2                                                   |
| SegmentID |  uint8   | 8 bit  | Same SegmentID as Motion Task 1<br />`0`: Single frame |
|  Torque   | bfloat16 | 16 bit | Segment torque [Nm]                                    |
| Reserved  |  uint8   | 8 bit  | Reserved                                               |
| Velocity  | bfloat24 | 24 bit | Velocity [RPS]                                         |

##### 0xC3: Motion Task 4

|    Name     |   Type   |  Size  | Description                                            |
| :---------: | :------: | :----: | :----------------------------------------------------- |
|    `cmd`    |  uint8   | 8 bit  | 0xC3                                                   |
|  SegmentID  |  uint8   | 8 bit  | Same SegmentID as Motion Task 1<br />`0`: Single frame |
|    Accel    | bfloat16 | 16 bit | Acceleration [RPS<sup>2</sup>]                         |
|    Jerk     | bfloat16 | 16 bit | Jerk [RPS<sup>3</sup>]                                 |
| io_mask_pos |  uint8   | 8 bit  | Probe IO trig mask                                     |
| io_mask_neg |  uint8   | 8 bit  | Probe IO trig mask                                     |

#### 0xCF: Save config

Save selected config to MCU memory

|  Name   |  Type  |  Size  | Description                           |
| :-----: | :----: | :----: | :------------------------------------ |
|   CMD   | uint8  | 8 bit  | 0xСF                                  |
| Catalog | uint24 | 24 bit | Field to save. `0xFFFFFF` - saves all |
| Keyword | uint32 | 32 bit | 0x10AD                                |

#### 0xD0: MIDI Control

Using motor as music instrument or sound indication

|   Name   |   Type   |  Size  | Description                            |
| :------: | :------: | :----: | :------------------------------------- |
|   CMD    |  uint8   | 8 bit  | 0xD0                                   |
| Waveform |  uint4   | 4 bit  | Sine/Square/Triangle/Sawtooth          |
|   Mode   |  uint4   | 4 bit  | TBD e.g. decay type etc                |
|   Tone   |  uint8   | 8 bit  | Base tone                              |
|  Volume  |  uint8   | 8 bit  | Volume, 0…255 as 0…100%                |
| Duration | bfloat16 | 16 bit | Duration [ms]                          |
|  Attack  |  uint8   | 8 bit  | Attack duty (0-255) from full duration |
|  Decay   |  uint8   | 8 bit  | Decay duty (0-255) from full duration  |

### 0x00…0x3F + 0x80 – 0xBF: Read/Write Registers

#### 0x00: Direct Register Access (R/W)

|  Name   |  Type  |  Size  |                 Description                 |
| :-----: | :----: | :----: | :-----------------------------------------: |
|   CMD   | uint8  | 8 bit  |                    0x6F                     |
| Address | uint24 | 24 bit |               Memory address                |
|  Value  | uint32 | 32 bit | Value to write (ignored for read operation) |

#### 0x01-0x0F: SubCMD space

|  Name  | Type  |  Size  |        Description         |
| :----: | :---: | :----: | :------------------------: |
|  CMD   | uint8 | 8 bit  |            0x??            |
| SubCMD | uint8 | 8 bit  |        Sub address         |
|  Data  | uint8 | 48 bit | Data/Variables to transfer |

##### 0x01??: Axis Control

|   Name    |   Type   | Size  | Description                                     |
| :-------: | :------: | :---: | :---------------------------------------------- |
|    CMD    |  uint8   | 8 bit | 0x01                                            |
|  subCMD   |  uint8   | 8 bit | 0x??                                            |
|  AxisID   |  uint8   | 8 bit | AxisID for multiaxis motion (may be duplicated) |
| Direction |   bool   | 1 bit | `0`: As is<br />`1`: Inverted                   |
| Reserved  |  uint7   | 7 bit | N/A                                             |
|  Offset   | q1.15.16 | 32bit | Position offset [rev]                           |

##### 0x01??: LED Control

|  Name   | Type  | Size  |             Description             |
| :-----: | :---: | :---: | :---------------------------------: |
|   CMD   | uint8 | 8 bit |                0x01                 |
| SubCMD  | uint8 | 8 bit |                0x??                 |
|  Mode   | uint8 | 8 bit |                 TBD                 |
| OnTime  | uint4 | 4 bit |  LED ON state duration in [0.25s]   |
| OffTime | uint4 | 4 bit |  LED OFF state duration in [0.25s]  |
|  LedID  | uint8 | 8 bit | ID of led to control. `0`: All leds |
|    R    | uint8 | 8bit  |          Red color control          |
|    G    | uint8 | 8bit  |         Green color control         |
|    B    | uint8 | 8bit  |         Blue color control          |

##### 0x01??: Gearbox control

|   Name   |   Type   |  Size  |                         Description                          |
| :------: | :------: | :----: | :----------------------------------------------------------: |
|   CMD    |  uint8   | 8 bit  |                             0x01                             |
|  SUBCMD  |  uint8   | 8 bit  |                             0x??                             |
| Backlash | bfloat16 | 16 bit |                        Backlash value                        |
| RatioMul |  int16   | 16 bit | Gearbox ratio multiplicator. Sign affects on output direction |
| RatioDiv |  uint16  | 16 bit |                    Gearbox ratio divider                     |

##### 0x01??: IO Control Map

Only 1 IO may be selected for each type. Command Pattern:

|    Name     | Type  |  Size  |          Description           |
| :---------: | :---: | :----: | :----------------------------: |
|     CMD     | uint8 | 8 bit  |              0x01              |
|   SUBCMD    | uint8 | 8 bit  |              0x??              |
|  Function   | uint8 | 8 bit  |       Device Function ID       |
|   OnPinID   | uint8 | 8 bit  |  Device Pin ID to set active   |
|  OffPinID   | uint8 | 8 bit  |  Device Pin ID to reset state  |
| OnPolarity  | bool  | 1 bit  | `0`: Direct<br />`1`: Inversed |
| OffPolarity | bool  | 1 bit  | `0`: Direct<br />`1`: Inversed |
|    State    | bool  | 1 bit  | `0`: Inactive<br />`1`: Active |
|  Reserved   |  N/A  | 21 bit |            Reserved            |

Device Funtion map:

| Funtion |    Name    |                  Description                   |
| :-----: | :--------: | :--------------------------------------------: |
|   0x0   |  StatusIO  | Read or manually write current register status |
|  0x01   |  HardStop  |    Hardstop for immideate power off driver     |
|  0x02   |    Home    |             Absolute zero position             |
|  0x03   |   LimCW    |            CW motion limit trigger             |
|  0x04   |   WarnCW   |           CW motion warning trigger            |
|  0x05   |   LimCCW   |            CCW motion limit trigger            |
|  0x06   |  WarnCCW   |           CCW motion warning trigger           |
|  0x07   | GearboxIDX |   Gearbox output full rotation index trigger   |
|  0x10   |   Probe1   |         General purpose Probe1 trigger         |
|  0x11   |   Probe2   |         General purpose Probe2 trigger         |
|  0x12   |   Probe3   |         General purpose Probe3 trigger         |
|  0x13   |   Probe4   |         General purpose Probe4 trigger         |
|  0x14   |   Probe5   |         General purpose Probe5 trigger         |
|  0x15   |   Probe6   |         General purpose Probe6 trigger         |
|  0x16   |   Probe7   |         General purpose Probe7 trigger         |
|  0x17   |   Probe8   |         General purpose Probe8 trigger         |
|   0x    |   Brake    |                 Braking output                 |

##### 0x01??: Encoder1 control (Encoder2 control absolutely same)

| **Name**   | **Type** | **Size** | **Description**                                              |
| ---------- | -------- | -------- | ------------------------------------------------------------ |
| CMD        | uint8    | 8 bit    | 0x01                                                         |
| SubCMD     | uint8    | 8 bit    | 0x??                                                         |
| Type       | uint4    | 4 bit    | `0`: Absolute<br />`1`: Incrementing<br />                   |
| Interface  | uint4    | 4 bit    | `0`: AB<br />`1`: Pulse/Dir<br /><br />`2`: Hall<br />`3`: SinCos<br />`4`: SPI<br />`5`: I2C<br />`6`: UART |
| Dir        | bool     | 1 bit    | `0`: CW<br />`1`: CCW                                        |
| Index      | bool     | 1 bit    | `0`: No<br />`1`: Yes                                        |
| Reserved   | N/A      | 6 bit    | N/A                                                          |
| EncoderPPR | uint32   | 32bit    | Encoder resolution [PPR]                                     |

##### 0x01??: Quick homing settings (R/W)

Quick search homing settings

|    Name    |   Type   |  Size  | Description                                                  |
| :--------: | :------: | :----: | :----------------------------------------------------------- |
|    CMD     |  uint8   | 8 bit  | 0x01                                                         |
|   SubCMD   |  uint8   | 8 bit  | 0x??                                                         |
|    Mode    |  uint2   | 2 bit  | Status of sensorless homing (active/inactive etc)            |
|  Trigger   |   bool   | 1 bit  | External trigger for homing (e.g. Endstop): `0`: Disable, `1`: Enabled |
| Direction  |   bool   | 1 bit  | Homing direction: 0: CW, 1: CCW                              |
|   Bidir    |   bool   | 1 bit  | Full motion range check: 0: disabled, 1: enabled             |
| Validation |   bool   | 1 bit  | Only active if `Bidir` is `1`. <br /> `0`: Auto (will perform self check),<br /> `1`: Reference displacement is provided |
|  Reserved  |   N/A    | 10 bit | Reserved                                                     |
|  Velocity  | bfloat16 | 16 bit | First search speed, [rev/s]                                  |
|   Torque   | bfloat16 | 16 bit | Torque threshold for stall detection, [Nm]                   |

##### 0x01??: Fine homing settings (R/W)

Fine search homing settings

|   Name   |   Type   |  Size  | Description                                                  |
| :------: | :------: | :----: | :----------------------------------------------------------- |
|   CMD    |  uint8   | 8 bit  | 0x01                                                         |
|  SubCMD  |  uint8   | 8 bit  | 0x??                                                         |
| Trigger  |   bool   | 1 bit  | External trigger for fine search (e.g. Endstop): `0`: Disable, `1`: Enabled |
| Retract  | bfloat16 | 16 bit | Retract before fine search [rev]                             |
| Reserved |   N/A    | 10 bit | Reserved                                                     |
| Velocity | bfloat16 | 16 bit | First search speed, [rev/s]                                  |
|  Torque  | bfloat16 | 16 bit | Torque threshold for stall detection, [Nm]                   |

##### 0x01??: CW Reference motion limit (R/W)

|   Name   |   Type   |  Size  |                         Description                          |
| :------: | :------: | :----: | :----------------------------------------------------------: |
|   CMD    |  uint8   | 8 bit  |                             0x01                             |
|  SubCMD  |  uint8   | 8 bit  |                             0x??                             |
| OffsetCW | bfloat16 | 16 bit | Expected offset from real boundary to `RefAngCW`. Must be less than 64 revs |
| RefAngCW |  uint32  | 32 bit |     Precise absolute angle reference for CW home search      |

##### 0x01??: CCW Reference motion limit (R/W)

|   Name    |   Type   |  Size  |                         Description                          |
| :-------: | :------: | :----: | :----------------------------------------------------------: |
|    CMD    |  uint8   | 8 bit  |                             0x01                             |
|  SubCMD   |  uint8   | 8 bit  |                             0x??                             |
| OffsetCCW | bfloat16 | 16 bit | Expected offset from real boundary to `RefAngCCW`. Must be less than 64 revs |
| RefAngCCW |  uint32  | 32 bit |     Precise absolute angle reference for CCW home search     |

##### 0x01??: Position Zero

Position shift to allign zero of encoder to work field zero

|    Name    |  Type  |  Size  | Description                                      |
| :--------: | :----: | :----: | :----------------------------------------------- |
|    CMD     | uint8  | 8 bit  | 0x01                                             |
|   SubCMD   | uint8  | 8 bit  | 0x??                                             |
|  Reserved  |  N/A   | 16 bit | Reserved                                         |
| RefAngZero | uint32 | 32 bit | Precise absolute angle reference for Zero search |

##### 0x01??: Position Zero offset CW

Distance between zero and CW endstop

|  Name  |   Type   |  Size  | Description     |
| :----: | :------: | :----: | :-------------- |
|  CMD   |  uint8   | 8 bit  | 0x01            |
| SubCMD |  uint8   | 8 bit  | 0x??            |
| Offset | q1.15.32 | 48 bit | Position offset |

##### 0x01??: Position Zero offset CCW

Distance between zero and CCW endstop

|  Name  |   Type   |  Size  | Description     |
| :----: | :------: | :----: | :-------------- |
|  CMD   |  uint8   | 8 bit  | 0x01            |
| SubCMD |  uint8   | 8 bit  | 0x??            |
| Offset | q1.15.32 | 48 bit | Position offset |

##### 0x01??: Motion field CW

Allowable motion from zero in CW direction

|    Name    |   Type   |  Size  | Description     |
| :--------: | :------: | :----: | :-------------- |
|    CMD     |  uint8   | 8 bit  | 0x01            |
|   SubCMD   |  uint8   | 8 bit  | 0x??            |
| Move limit | q1.15.32 | 48 bit | CW motion limit |

##### 0x01??: Internal Position Zero offset encoder

Distance between system zero and internal position

|  Name  |   Type   |  Size  | Description     |
| :----: | :------: | :----: | :-------------- |
|  CMD   |  uint8   | 8 bit  | 0x01            |
| SubCMD |  uint8   | 8 bit  | 0x??            |
| Offset | q1.15.32 | 48 bit | Position offset |

#### 0x10: Bus settings

| **Name** | **Type** | **Size** | **Description**                                              |
| -------- | -------- | -------- | ------------------------------------------------------------ |
| CMD      | uint8    | 8 bit    | 0x11                                                         |
| Bus type | uint8    | 4 bit    |                                                              |
| Comm     | uint2    | 4 bit    | `0`: Read only<br />`1`: Respond on request<br />`2`: Respond every message |
| Baud1    | uint8    | 8 bit    |                                                              |
| Baud2    | uint8    | 8 bit    | 0 if not supported                                           |
| Sync     | uint32   | 32 bit   | Current master time [x50us]                                  |

#### 0x11: Discovery assignment (R/W)

Hashed UID of the selected driver, used for node identification and address assignment.

- **Write Access:** Used to assign a node address or reset a device. To assign an address, write the target device's `HashUID48` along with the desired `Addr`. Do NOT assign the same address to different nodes to avoid collisions. Writing `Addr = 255` puts the target node into discovery mode. Sending a write command with Broadcast ID (`0`) and `Addr = 255` resets all nodes on the bus to discovery mode (`Addr = 255`). Nodes in discovery mode (`Addr = 255`) will respond ONLY to `0x11` and `0x12` commands. Writing to specified ID `Addr = 255` with HashUID = 0 resets all nodes with this assigned ID (collision case).
- **Read Access:** Used to verify node presence and ping devices. A read request sent to Broadcast ID (`0`) with a specific `HashUID48` will prompt the matching node to respond with its assigned `Addr` and echo its `HashUID48`. A read request with a specific node `Addr` and `Addr = 255` forces the node to return its `HashUID48`. 

| **Name**  | **Type** | **Size** | **Description**       |
| --------- | -------- | -------- | --------------------- |
| CMD       | uint8    | 8 bit    | 0x11                  |
| Addr      | uint8    | 8 bit    | Assigned node address |
| HashUID48 | uint48   | 48 bit   | 48-bit Device HashUID |

#### 0x12: Discovery search (R/W)

Used for dynamic node discovery on the bus.

- **Procedure:** The master initiates discovery by sending `0x12` to the Broadcast ID (`0`) with `Depth = 0`. Unassigned target nodes (`Addr = 255`) respond with `CMD = 0x12` (Read-only mode), incremented `++Depth`, and the first `Depth` bits of their `HashUID48`. The master accumulates the discovered bits and transmits them in subsequent cycles to navigate the binary search tree.
- **Address Assignment Verification:** Once the full 48-bit `HashUID48` is resolved, the master sends a `0x11` Write command with the designated `Addr`. The master must then double-check the assignment by issuing a `0x11` Read command with the target `HashUID48` to Broadcast ID (`0`), expecting a response from the newly assigned node confirming its new `Addr`.
- **Filtering:** Assigned nodes (`Addr != 255`) ignore `0x12` discovery requests unless they are explicitly reset back to `Addr = 255` via command `0x11` OR on their assigned it to be able check same address collisions without resetting them.  

| **Name**  | **Type** | **Size** | **Description**                            |
| --------- | -------- | -------- | ------------------------------------------ |
| CMD       | uint8    | 8 bit    | 0x12                                       |
| Depth     | uint8    | 8 bit    | Search depth in bits (starts at `0`)       |
| HashUID48 | uint48   | 48 bit   | Currently discovered 48-bit HashUID prefix |


#### 0x20: System Control

|      Name       | Type  | Size  | Description                                             |
| :-------------: | :---: | :---: | :------------------------------------------------------ |
|       CMD       | uint8 | 8 bit | 0x20                                                    |
|  Driver Enable  | uint8 | 8 bit | Write 0xE1 to enable driver, Write 0x0F to disable      |
| Burst mode time | uint8 | 8 bit | Burst mode step [250us]. `0` - variable time burst mode |
|      P_CAL      | bool  | 1 bit | Encoder Calibration                                     |
|      I_CAL      | bool  | 1 bit | Current sense calibration                               |
|      E_CAL      | bool  | 1 bit | Phase connection and motor check                        |
|      R_CAL      | bool  | 1 bit | Resistance calibration                                  |
|      L_CAL      | bool  | 1 bit | Inductance calibration                                  |
|      F_CAL      | bool  | 1 bit | Flux calibration                                        |
|      M_CAL      | bool  | 1 bit | Mechanical Calibration                                  |
|       TBD       |       |       |                                                         |

#### 0x21: Motor properties 1(R/W)

|       Name       |   Type   |  Size  |                         Description                          |
| :--------------: | :------: | :----: | :----------------------------------------------------------: |
|       CMD        |  uint8   | 8 bit  |                             0x21                             |
|       Mode       |          | 4 bit  |                   Motor control mode (TBD)                   |
|    Motor type    |          | 4 bit  |        0: Not defined, 1: STEPPER; 2: BLDC; 3: ERROR         |
| Phase Connection |          | 2 bit  |           0: Not connected, 1: ABC; 2: CAB; 3: ACB           |
|    Direction     |   int    | 2 bit  |       Rotation direction -1: CW, 1: CCW, -2: Undefined       |
|    Pole pairs    |  uint12  | 12 bit |                         0-4095 poles                         |
|   Max voltage    | bfloat16 | 16 bit | Maximum voltage that may be applied to phases due to insulation |
|   Max current    | bfloat16 | 16 bit |     Maximum current that may be safely applied to phases     |

#### 0x22: Motor properties 2 (RW)

Motor dynamic related properties

|    Name     |   Type   |  Size  |        Description        |
| :---------: | :------: | :----: | :-----------------------: |
|     CMD     |  uint8   | 8 bit  |           0x22            |
| Temperature |  uint8   | 8 bit  |  Temperature in Celsius   |
| Resistance  | bfloat16 | 16 bit | Motor resistance in Ohms  |
| Inductance  | bfloat16 | 16 bit | Motor inductance in Henry |
|     Kt      | bfloat16 | 16 bit |  Motor torque value Nm/A  |

#### 0x23: P<sub>Loss</sub>t properties and Overdrive (R/W)

Motor protection config

|        Name         |   Type   |  Size  |                Description                 |
| :-----------------: | :------: | :----: | :----------------------------------------: |
|         CMD         |  uint8   | 8 bit  |                    0x23                    |
| Overdrive  duration |  uint8   | 8 bit  |  Maximum duration of owerpower in seconds  |
|   Power baseline    | bfloat16 | 16 bit | All power under baseline not affect Pt [W] |
|     Peak power      | bfloat16 | 16 bit |        Maximum allowable power [W]         |
| Overpower capacity  | bfloat16 | 16 bit |    Energy budget for power draining [J]    |

> 0, NaN or inf will be ignored. 0 duration disables overdrive

#### 0x24: Motor Thermal Protection (R/W)

Motor protection config

|         Name         |       Type       |  Size  |                 Description                 |
| :------------------: | :--------------: | :----: | :-----------------------------------------: |
|         CMD          |      uint8       | 8 bit  |                    0x24                     |
| Current temperature  |      uint8       | 8 bit  |              Real temperature               |
| Temperature baseline |      uint8       | 8 bit  | Full power threshold temperature in Celsius |
|  Temperature  limit  |      uint8       | 8 bit  |         Max temperature in Celsius          |
|    Power derating    | bfloat16 (1.8.7) | 16 bit |    Max motor power derating in [%/degC]     |
|         Rth          | bfloat16 (1.8.7) | 16 bit |      Motor thermal resistance [degC/W]      |

#### 0x25: Driver thermal protection (R/W)

|        Name         | Type  |  Size  |                Description                 |
| :-----------------: | :---: | :----: | :----------------------------------------: |
|         CMD         | uint8 | 8 bit  |                    0x25                    |
| Current temperature | uint8 | 8 bit  |       Driver temperature in Celsius        |
|       OT Warn       | uint8 | 8 bit  | Overtemperature warn threshold in Celsius  |
|      OT Fault       | uint8 | 8 bit  | Overtemperature fault threshold in Celsius |
|     Hysteresis      | uint8 | 8 bit  |   Restore hysteresis voltage in Celsius    |
|      Reserved       |       | 24 bit |                  Reserved                  |

#### 0x26: Supply voltage protection (R/W)

|    Name    |   Type   |  Size  | Description                          |
| :--------: | :------: | :----: | :----------------------------------- |
|    CMD     |  uint8   | 8 bit  | 0x26                                 |
| Hysteresis | bfloat16 | 16 bit | Restore hysteresis voltage           |
|  Recovery  |  uint8   | 8 bit  | Recovery time in 100us. 0 is ignored |
|     OV     | bfloat16 | 16 bit | Overvoltage trip voltage level       |
|     UV     | bfloat16 | 16 bit | Undervoltage trip voltage level      |

#### 0x27: Driver current protection

|        Name        |   Type   |  Size  |                         Description                          |
| :----------------: | :------: | :----: | :----------------------------------------------------------: |
|        CMD         |  uint8   | 8 bit  |                             0x27                             |
| Allowed disbalance |  uint8   | 8 bit  |               0…255 as 0…100% of full current                |
|  Disbalance alpha  |  uint8   | 8 bit  |     Filtering value. `0`: ignored, `255`: no filtration      |
|      Recovery      |  uint8   | 8 bit  |             Recovery time in 100us. 0 is ignored             |
|    Peak current    | bfloat16 | 16 bit |                   Maximum current of motor                   |
|         OC         | bfloat16 | 16 bit | Overcurrent protection threshold [A]. Add at least 30% from peak motor current |

#### 0x2A: System Motion Limits

|  Name   |       Type       |  Size  |          Description           |
| :-----: | :--------------: | :----: | :----------------------------: |
|   CMD   |      uint8       | 8 bit  |              0x2A              |
| `vel24` | float24 (1.8.15) | 24 bit |     Velocity limit in RPS      |
| `acc16` | bfloat16 (1.8.7) | 16 bit | Accel limit in RPS<sup>2</sup> |
| `jrk16` | bfloat16 (1.8.7) | 16 bit | Jerk limit in RPS<sup>3</sup>  |

#### 0x2B: Error Motion Limits

Motion limits for fault conditions

|  Name   |       Type       |  Size  |          Description           |
| :-----: | :--------------: | :----: | :----------------------------: |
|   CMD   |      uint8       | 8 bit  |              0x2B              |
| `vel24` | float24 (1.8.15) | 24 bit |     Velocity limit in RPS      |
| `acc16` | bfloat16 (1.8.7) | 16 bit | Accel limit in RPS<sup>2</sup> |
| `jrk16` | bfloat16 (1.8.7) | 16 bit | Jerk limit in RPS<sup>3</sup>  |

#### 0x2D: Watchdog and PulseBeat control

|       Name       |   Type   |  Size  | Description                                                  |
| :--------------: | :------: | :----: | :----------------------------------------------------------- |
|       CMD        |  uint8   | 8 bit  | 0x2D                                                         |
|    Retry Mode    |   bool   | 1 bit  | 0: Attempt next timeout, 1: Force retry                      |
|      nRetry      |  uint7   | 7 bit  | PulseBeat send retry attempts before FaultBeat. <br />`0` - no FaultBeat, <br />`127` - ignored |
| PulseBeat Timer  | bfloat16 | 16 bit | Time between packets, 0 - watchdog disabled, NaN - Ignored   |
|      Delay       | bfloat16 | 16 bit | Delay from bus sync message (to avoid collisions)            |
| Watchdog Timeout | bfloat16 | 16 bit | Time since last message before driver safe state [sec]       |

#### 0x2F: Calibration table

|    Name     |  Type  |  Size  | Description                                                  |
| :---------: | :----: | :----: | :----------------------------------------------------------- |
|     CMD     | uint8  | 8 bit  | 0x2F                                                         |
|    Mode     | uint2  | 2 bit  | Check etc                                                    |
| Coordinates |  bool  | 1 bit  | `0`: Relative, `1`: Absolute                                 |
| Resolution  | uint2  | 2 bit  | `0`: 8 bit, `1`: 16 bit, `2`: 24 bit, `3`: 32 bit            |
|    Table    | uint3  | 3 bit  | Table to read/write to: `0`: Encoder<br />`1`: Poles<br />`2`: Cogging<br />`3`: Current->Torque<br />`4`: Speed->Torque<br />`5`: OutputPos |
|  CalSample  | uint16 | 16 bit | Sample ID                                                    |
|    Data     | uint32 | 32 bit | Read or write calibration data<br />R with 0xCA1B5I3E will return calibration size in `CalSample` |

