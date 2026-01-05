# BMU CAN Protocol & ADC DMA

Profesionalna implementacija CAN komunikacije (500 kbps) in DMA-based ADC branja.

---

## 🚀 CAN Bus Protocol (500 kbps)

### Konfiguracija

**Baudrate**: 500 kbps
**Bus Standard**: CAN 2.0B
**ID Format**: Standard 11-bit
**Termination**: 120Ω na obeh koncih vodila

### Hardware

```
CAN1: Internal BMU communication
├─ TX: PG1
└─ RX: PG0

CAN2: External BMS/VCU communication
├─ TX: PB13
└─ RX: PB12
```

### Timing Parameters

```
APB1 Clock: 16 MHz
Prescaler: 2
Time Segment 1 (BS1): 13 TQ
Time Segment 2 (BS2): 2 TQ
Sync Jump Width: 1 TQ

Baudrate = 16 MHz / (Prescaler × (1 + BS1 + BS2))
         = 16 MHz / (2 × 16)
         = 500 kbps ✓
```

---

## 📨 CAN Messages

| ID (Hex) | Name | DLC | Rate | Description |
|----------|------|-----|------|-------------|
| 0x100 | BMU_Status | 8 | 10 Hz | System status and power flags |
| 0x101 | BMU_Temperature | 8 | 1 Hz | Temperature sensor data |
| 0x110 | LEM_Current_1 | 8 | 10 Hz | LEM sensors 1-4 currents |
| 0x111 | LEM_Current_2 | 8 | 10 Hz | LEM sensors 5-8 currents |
| 0x112 | LEM_Current_3 | 8 | 10 Hz | LEM sensors 9-10 currents |
| 0x120 | BTT6200_Status_1 | 8 | 5 Hz | BTT6200 modules 0-1 status |
| 0x121 | BTT6200_Status_2 | 8 | 5 Hz | BTT6200 modules 2-3 status |
| 0x122 | BTT6200_Status_3 | 8 | 5 Hz | BTT6200 module 4 status |
| 0x130 | FRAM_Statistics | 8 | 1 Hz | FRAM memory statistics |
| 0x140 | Alarms | 8 | Event | System alarms and errors |
| 0x1FF | Heartbeat | 8 | 1 Hz | Heartbeat with counter |

---

## 📊 Message Structures

### 0x100 - BMU Status

```c
typedef struct __attribute__((packed)) {
    uint8_t  system_state;      // 0=Init, 1=Normal, 2=Warning, 3=Error
    uint8_t  power_good_flags;  // Bit0=5V, Bit1=3V3A, Bit2=24V
    uint16_t uptime_seconds;    // System uptime [0-65535] s
    uint8_t  active_outputs;    // Number of active BTT6200 outputs
    uint8_t  active_sensors;    // Number of active LEM sensors
    uint16_t reserved;
} BMU_Status_Msg_t;  // 8 bytes
```

### 0x101 - Temperature

```c
typedef struct __attribute__((packed)) {
    int16_t temperature_C;      // Temperature × 100 (2534 = 25.34°C)
    uint8_t alert_flag;         // 1 = below 0°C alert
    uint8_t sensor_status;      // TMP1075 status
    int16_t min_temp_C;         // Min temperature × 100
    int16_t max_temp_C;         // Max temperature × 100
} BMU_Temperature_Msg_t;  // 8 bytes
```

### 0x110-0x112 - LEM Current

```c
typedef struct __attribute__((packed)) {
    int16_t current_1_mA;       // Current in mA (signed)
    int16_t current_2_mA;       // Range: -200000 to +200000 mA
    int16_t current_3_mA;
    int16_t current_4_mA;
} BMU_LEM_Current_Msg_t;  // 8 bytes
```

### 0x1FF - Heartbeat

```
Byte 0-3: Counter (uint32_t)
Byte 4:   0xBE (magic)
Byte 5:   0xEF (magic)
Byte 6-7: Reserved
```

---

## 💻 API Usage

### Inicializacija

```c
#include "bmu_can.h"

BMU_CAN_HandleTypeDef hbmucan;

// Initialize CAN @ 500 kbps
HAL_StatusTypeDef status = BMU_CAN_Init(&hbmucan, &hcan1, &hcan2);
if (status == HAL_OK) {
    // CAN ready!
}
```

### Pošiljanje sporočil

```c
// 1. Temperature message
BMU_Temperature_Msg_t temp_msg = {0};
temp_msg.temperature_C = 2534;  // 25.34°C
temp_msg.alert_flag = 0;
temp_msg.min_temp_C = 1856;     // 18.56°C
temp_msg.max_temp_C = 3012;     // 30.12°C
BMU_CAN_SendTemperature(&hbmucan, &temp_msg);

// 2. LEM Current message
BMU_LEM_Current_Msg_t lem_msg = {0};
lem_msg.current_1_mA = 12345;   // 12.345 A
lem_msg.current_2_mA = -5678;   // -5.678 A
lem_msg.current_3_mA = 0;
lem_msg.current_4_mA = 25000;   // 25.0 A
BMU_CAN_SendLEMCurrent(&hbmucan, CAN_ID_LEM_CURRENT_1, &lem_msg);

// 3. Heartbeat
static uint32_t counter = 0;
BMU_CAN_SendHeartbeat(&hbmucan, counter++);
```

### Statistika

```c
uint32_t tx_count, rx_count, error_count;
BMU_CAN_GetStats(&hbmucan, &tx_count, &rx_count, &error_count);
printf("TX:%lu RX:%lu ERR:%lu\n", tx_count, rx_count, error_count);
```

---

## 🔧 DBC File (Busmaster/CANalyzer)

### Datoteka: `BMU_CANbus.dbc`

**Uporaba**:
1. Odpri Busmaster/CANalyzer
2. File → Import → Database
3. Izberi `BMU_CANbus.dbc`
4. Vse CAN sporočila bodo avtomatsko razpoznana

**Funkcionalnosti**:
- ✅ Vse CAN ID definicije
- ✅ Signal scaling (0.01 za temperaturo, 0.001 za tok)
- ✅ Enote (°C, A, s)
- ✅ Value tables (SystemState: 0=Init, 1=Normal, ...)
- ✅ Message cycle times
- ✅ Komentarji za vse signale

### Primer v CANalyzer:

```
Message: BMU_Temperature (0x101)
├─ Temperature_C:  25.34 °C   (Raw: 2534)
├─ AlertFlag:      OK         (Raw: 0)
├─ MinTemp_C:      18.56 °C   (Raw: 1856)
└─ MaxTemp_C:      30.12 °C   (Raw: 3012)
```

---

## ⚡ ADC DMA (Continuous Reading)

### Funkcionalnost

```c
#include "adc_dma.h"

ADC_DMA_HandleTypeDef hadc_dma;

// Initialize DMA for 16 channels
ADC_DMA_Init(&hadc_dma, &hadc1, &hdma_adc1);

// Start continuous conversions
ADC_DMA_Start(&hadc_dma);

// Read single channel (non-blocking!)
uint16_t adc_value;
ADC_DMA_GetValue(&hadc_dma, ADC_DMA_LEM_1, &adc_value);

// Read all channels at once
uint16_t all_values[16];
ADC_DMA_GetAllValues(&hadc_dma, all_values);
```

### ADC Channel Mapping

| Channel | Sensor | Pin | Index |
|---------|--------|-----|-------|
| ADC_IN0 | LEM_1 | PA0 | 0 |
| ADC_IN1 | LEM_2 | PA1 | 1 |
| ADC_IN2 | LEM_3 | PA2 | 2 |
| ADC_IN3 | LEM_4 | PA3 | 3 |
| ADC_IN4 | LEM_5 | PA4 | 4 |
| ADC_IN5 | LEM_6 | PA5 | 5 |
| ADC_IN6 | LEM_7 | PA6 | 6 |
| ADC_IN7 | LEM_8 | PA7 | 7 |
| ADC_IN8 | LEM_9 | PB0 | 8 |
| ADC_IN9 | LEM_10 | PB1 | 9 |
| ADC_IN10 | IS_0 | PC0 | 10 |
| ADC_IN11 | IS_1 | PC1 | 11 |
| ADC_IN12 | IS_2 | PC2 | 12 |
| ADC_IN13 | IS_3 | PC3 | 13 |
| ADC_IN14 | IS_4 | PC4 | 14 |
| ADC_IN15 | PWR_CURRENT | PC5 | 15 |

### Prednosti DMA

✅ **Zero CPU overhead** - ADC deluje v ozadju
✅ **Circular buffer** - kontinuirano posodabljanje
✅ **16 kanalov hkrati** - vsi senzorji v enem ciklu
✅ **Conversion complete callback** - interrupt po končani konverziji
✅ **Error handling** - statistika napak

---

## 🧪 Testing

### CAN Bus Testing

**Orodja**:
- Vector CANalyzer (Windows)
- Busmaster (Open Source)
- PCAN-View (PEAK Systems)
- SavvyCAN (Linux/Mac)

**Test procedure**:
1. Poveži CAN transceiver na BMU (CAN1 ali CAN2)
2. Odpri DBC file v CANalyzer/Busmaster
3. Zaženi BMU
4. Preveri Heartbeat @ 1Hz (0x1FF)
5. Preveri Temperature @ 1Hz (0x101)
6. Preveri LEM Current @ 10Hz (0x110-0x112)

### Signal Decoding

**Temperature (0x101)**:
```
Raw bytes: 09 E6 00 00 07 40 0B C4
           ^^^^^ Temperature
Decoded: 0x09E6 = 2534 → 25.34°C ✓
```

**Current (0x110)**:
```
Raw bytes: 30 39 E9 F2 00 00 61 A8
           ^^^^^ LEM1
Decoded: 0x3039 = 12345 → 12.345 A ✓
```

---

## ⚙️ Configuration (CubeMX)

### CAN

**Connectivity → CAN1**:
```
Mode: Master
Prescaler: 2
Time Segment 1: 13 Quanta
Time Segment 2: 2 Quanta
Auto Bus-Off: Enabled
Auto Retransmission: Enabled
```

**Connectivity → CAN2**:
```
(Same as CAN1)
```

### ADC DMA (Optional - za aktiviranje)

**Analog → ADC1**:
```
Mode: Scan Conversion
Continuous Conversion: Enabled
DMA Continuous Requests: Enabled
Number of Conversions: 16
Scan Mode: Enabled

Channels: IN0, IN1, ..., IN15
```

**DMA Settings**:
```
DMA Request: ADC1
Direction: Peripheral to Memory
Mode: Circular
Data Width: Half Word (16-bit)
Increment Address: Memory only
```

---

## 📝 Diagnostics

### UART Output (115200 baud)

```
=== CAN Bus Protocol ===
CAN bus initialized OK (500 kbps)

Temperature: 25.34 C
LEM_1: 12.345 A  LEM_2: -5.678 A  LEM_3: 0.000 A
[CAN] TX:30 RX:0 ERR:0

Temperature: 25.35 C
LEM_1: 12.340 A  LEM_2: -5.680 A  LEM_3: 0.000 A
[CAN] TX:60 RX:0 ERR:0
```

**CAN Statistics vsako 10s**:
- TX: Število poslanih sporočil
- RX: Število prejetih sporočil
- ERR: Število napak

---

## 🚨 Troubleshooting

### Problem: CAN TX ne dela

**Rešitev**:
1. Preveri CAN transceiver (TJA1050/MCP2551)
2. Preveri 120Ω termination
3. Preveri baudrate (mora biti enak na vseh vozliščih)
4. Preveri CAN_H in CAN_L napeljavo

### Problem: DMA ne dela

**Rešitev**:
1. Konfiguriraj DMA v CubeMX (glej Configuration)
2. Odkomentiraj `ADC_DMA_Init()` in `ADC_DMA_Start()` v main.c
3. Preveri da je `hdma_adc1` inicializiran

### Problem: Napačne vrednosti

**Rešitev**:
- Preveri byte order (Little Endian!)
- Preveri scaling factor v DBC (0.01, 0.001)
- Preveri signed/unsigned interpretacijo

---

## 📚 References

- **CAN 2.0B Specification**: Bosch CAN Specification
- **DBC Format**: Vector DBC File Format Specification
- **STM32 HAL**: UM1725 STM32F4 HAL User Manual
- **Busmaster**: https://rbei-etas.github.io/busmaster/

---

**Avtor**: BMU IOC Project
**Datum**: 2025
**Version**: 1.0
**License**: Proprietary
