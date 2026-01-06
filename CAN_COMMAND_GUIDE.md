# CAN Bus Command Guide - BMU Control

## BTT6200 Output Control (0x200)

### Message Format

**CAN ID:** `0x200` (512 decimal)
**DLC:** `8 bytes`
**Format:** Standard ID (11-bit)

### Data Bytes (Little Endian!)

```
Byte 0: output_id     (0-19) - kateri output vklopiti
Byte 1: command       0=OFF, 1=ON, 2=TOGGLE
Byte 2: reserved      0x00
Byte 3: reserved      0x00
Byte 4: magic[0]      0xEF  (little endian!)
Byte 5: magic[1]      0xBE
Byte 6: magic[2]      0xAD
Byte 7: magic[3]      0xDE
```

**POMEMBNO:** Magic number `0xDEADBEEF` mora biti poslan v **LITTLE ENDIAN** formatu: `EF BE AD DE`

### Primer 1: Vklopi OUTPUT 0

```
CAN ID: 0x200
DLC: 8
Data: 00 01 00 00 EF BE AD DE
       ││ ││ └─┴─ reserved
       ││ ││
       ││ └───── command=1 (ON)
       └──────── output_id=0
```

**PCAN-View format:**
```
ID: 200h
DLC: 8
Data: 00 01 00 00 EF BE AD DE
```

**SocketCAN (Linux):**
```bash
cansend can0 200#0001.0000.EFBE.ADDE
```

**Python (python-can):**
```python
import can

bus = can.interface.Bus(channel='can0', bustype='socketcan')
msg = can.Message(
    arbitration_id=0x200,
    data=[0x00, 0x01, 0x00, 0x00, 0xEF, 0xBE, 0xAD, 0xDE],
    is_extended_id=False
)
bus.send(msg)
```

### Primer 2: Izklopi OUTPUT 5

```
CAN ID: 0x200
DLC: 8
Data: 05 00 00 00 EF BE AD DE
       ││ ││
       ││ └───── command=0 (OFF)
       └──────── output_id=5
```

### Primer 3: Toggle OUTPUT 10

```
CAN ID: 0x200
DLC: 8
Data: 0A 02 00 00 EF BE AD DE
       ││ ││
       ││ └───── command=2 (TOGGLE)
       └──────── output_id=10 (0x0A)
```

---

## BTT6200 Multi Command (0x201)

**CAN ID:** `0x201` (513 decimal)
**DLC:** `8 bytes`

### Data Bytes

```
Byte 0-3: output_mask (uint32_t, little endian)
          Bits 0-19: katere outpute kontrolirati (1=control, 0=ignore)
Byte 4-7: output_states (uint32_t, little endian)
          Bits 0-19: želeno stanje (1=ON, 0=OFF)
```

### Primer: Vklopi OUT0, OUT1, OUT2, izklopi vse druge

```
output_mask   = 0x000FFFFF (vsi outputi 0-19)
output_states = 0x00000007 (samo OUT0, OUT1, OUT2 = 1)

Data: FF FF 0F 00 07 00 00 00
      └────┴──┴─ mask (little endian)
                  └────┴──┴─ states (little endian)
```

---

## System Commands (0x202)

**CAN ID:** `0x202` (514 decimal)
**DLC:** `8 bytes`

### Data Bytes

```
Byte 0: command
        0x00 = NOP (no operation)
        0x01 = RESET_STATS (reset CAN statistics)
        0x02 = DISABLE_ALL (disable all outputs)
        0x03 = ENABLE_ALL (enable all outputs)
        0xFE = REBOOT (software reset MCU)
Byte 1-3: reserved (0x00)
Byte 4-7: magic number 0xCAFEBABE (little endian: BE BA FE CA)
```

### Primer: Disable vse outpute

```
CAN ID: 0x202
DLC: 8
Data: 02 00 00 00 BE BA FE CA
       ││ └─┴─┴─ reserved
       └──────── command=2 (DISABLE_ALL)
```

### Primer: Software reboot

```
CAN ID: 0x202
DLC: 8
Data: FE 00 00 00 BE BA FE CA
       ││
       └──────── command=0xFE (REBOOT)
```

---

## Debug Output

Če imaš vklopljen debug output (`#if 1` v `BMU_CAN_RxCallback`), boš na UART videl:

**Uspešno sprejet command:**
```
[CAN RX] ID:0x200 DLC:8 Data: 00 01 00 00 EF BE AD DE
[CAN RX] OK
```

**Napačen magic number:**
```
[CAN RX] ID:0x200 DLC:8 Data: 00 01 00 00 DE AD BE EF
[CAN RX] Processing FAILED! Errors: 1
```

**Napačen DLC:**
```
[CAN RX] ID:0x200 DLC:4 Data: 00 01 00 00 00 00 00 00
[CAN RX] Processing FAILED! Errors: 1
```

---

## Pogosti problemi

### 1. "Processing FAILED" - Magic number error

**Problem:** Magic number ni pravilen
**Vzrok:** Little endian vs big endian
**Rešitev:**
- `0xDEADBEEF` → poslji `EF BE AD DE`
- `0xCAFEBABE` → poslji `BE BA FE CA`

### 2. "Processing FAILED" - DLC error

**Problem:** DLC ni 8
**Rešitev:** Vedno pošlji 8 bajtov (tudi če so reserved 0x00)

### 3. Sporočilo sploh ne pride

**Problem:** CAN filter blokira sporočila
**Rešitev:** Filter je nastavljen na "accept all" (0x0000), če še vedno ne dela, preveri:
- CAN baudrate (500 kbps)
- CAN termination (120Ω)
- CAN high/low pravilno povezana

### 4. Output se ne vklopi

**Preverite:**
1. Ali debug output kaže `[CAN RX] OK`?
2. Ali je BTT6200 pravilen inicializiran?
3. Ali ima BTT6200 napajanje (24V)?
4. Ali je output_id pravilen (0-19)?

---

## Test Sequence

**Korak po korak test:**

```bash
# 1. Vklopi output 0
cansend can0 200#0001.0000.EFBE.ADDE

# 2. Preveri LED ali status
# Expected: OUT0 should turn ON

# 3. Izklopi output 0
cansend can0 200#0000.0000.EFBE.ADDE

# 4. Toggle output 0
cansend can0 200#0002.0000.EFBE.ADDE

# 5. Disable vse outpute
cansend can0 202#0200.0000.BEBA.FECA

# 6. Enable vse outpute
cansend can0 202#0300.0000.BEBA.FECA
```

---

## Output ID Mapping (0-19)

```
0  = BMU_OUT0_0  (Module 0, Channel 0)
1  = BMU_OUT1_0  (Module 0, Channel 1)
2  = BMU_OUT2_0  (Module 0, Channel 2)
3  = BMU_OUT3_0  (Module 0, Channel 3)
4  = BMU_OUT0_1  (Module 1, Channel 0)
5  = BMU_OUT1_1  (Module 1, Channel 1)
6  = BMU_OUT2_1  (Module 1, Channel 2)
7  = BMU_OUT3_1  (Module 1, Channel 3)
8  = BMU_OUT0_2  (Module 2, Channel 0)
9  = BMU_OUT1_2  (Module 2, Channel 1)
10 = BMU_OUT2_2  (Module 2, Channel 2)
11 = BMU_OUT3_2  (Module 2, Channel 3)
12 = BMU_OUT0_3  (Module 3, Channel 0)
13 = BMU_OUT1_3  (Module 3, Channel 1)
14 = BMU_OUT2_3  (Module 3, Channel 2)
15 = BMU_OUT3_3  (Module 3, Channel 3)
16 = BMU_OUT0_4  (Module 4, Channel 0)
17 = BMU_OUT1_4  (Module 4, Channel 1)
18 = BMU_OUT2_4  (Module 4, Channel 2)
19 = BMU_OUT3_4  (Module 4, Channel 3)
```
