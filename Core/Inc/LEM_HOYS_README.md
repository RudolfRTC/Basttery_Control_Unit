# LEM HOYS-S/SP33 Current Sensor Driver

Profesionalna knjižnica za LEM HOYS serijo Hall-effect current sensorjev.

## 📋 Podprti modeli

| Model | Nominal Current | Sensitivity | Max Current |
|-------|----------------|-------------|-------------|
| HOYS-6A | ±6 A | 25 mV/A | ±12 A |
| HOYS-15A | ±15 A | 10 mV/A | ±30 A |
| HOYS-25A | ±25 A | 6 mV/A | ±50 A |
| HOYS-50A | ±50 A | 3 mV/A | ±100 A |
| HOYS-100A | ±100 A | 1.5 mV/A | ±200 A |

## 🚀 Hitri začetek

### 1. Inicializacija

```c
#include "lem_config.h"

// Inicializiraj vse 10 LEM sensorje
HAL_StatusTypeDef status = LEM_Config_Init(&hadc1);
if (status == HAL_OK) {
    // Uspešna inicializacija
}
```

### 2. Kalibracija (POMEMBNO!)

**OPOMBA**: Kalibracijo izvedi **brez toka** (0A) skozi senzorje!

```c
// Kalibriraj vse senzorje (50-100 vzorcev priporočeno)
HAL_StatusTypeDef cal_status = LEM_Config_CalibrateAll(100);
if (cal_status == HAL_OK) {
    // Kalibracija uspešna
}
```

### 3. Branje toka

```c
float current_A;

// Preberi tok na senzorju LEM_1 (sensor_id = 0)
LEM_Config_ReadCurrentFiltered(0, &current_A);
printf("Current: %.3f A\n", current_A);

// ALI preberi vse senzorje naenkrat
float currents[10];
LEM_Config_ReadAllCurrents(currents);
for (int i = 0; i < 10; i++) {
    printf("LEM_%d: %.3f A\n", i+1, currents[i]);
}
```

### 4. Overcurrent detection

```c
uint16_t oc_flags = 0;
LEM_Config_CheckOvercurrents(&oc_flags);

if (oc_flags != 0) {
    // Overcurrent detected!
    if (oc_flags & (1 << 0)) {
        // LEM_1 overcurrent
    }
    if (oc_flags & (1 << 1)) {
        // LEM_2 overcurrent
    }
    // itd...
}
```

## 🔧 Napredne funkcije

### Prilagoditev filtra

```c
// Nastavi filter depth na 16 vzorcev za LEM_1
LEM_HOYS_SetFilterDepth(&lem_sensors[0], 16);
```

### Branje raw ADC vrednosti

```c
uint16_t adc_value;
LEM_HOYS_ReadRawADC(&lem_sensors[0], &adc_value);
```

### Custom kalibracija

```c
// Nastavi custom zero offset in sensitivity
LEM_HOYS_SetCalibration(&lem_sensors[0], 2048, 6.0f);
```

### Diagnostika

```c
// Izpiši info o senzorju preko UART
LEM_Config_PrintSensorInfo(&huart1, 0);  // LEM_1
```

Output:
```
=== LEM_1 (HOYS-25A) ===
Nominal: 25.0A, Max: 50.0A
Calibrated: YES
Zero offset: 2048 ADC, Sensitivity: 6.00 mV/A
Last reading: 12.345 A
Overcurrent events: 0
Status: 0x00 (OK)
```

## 📊 Status flags

```c
LEM_Status_t status = LEM_HOYS_GetStatus(&lem_sensors[0]);

if (status & LEM_STATUS_OVERCURRENT) {
    // Overcurrent detection
}
if (status & LEM_STATUS_OUT_OF_RANGE) {
    // Current out of measurement range
}
if (status & LEM_STATUS_NOT_CALIB) {
    // Sensor not calibrated
}
if (status & LEM_STATUS_ADC_ERROR) {
    // ADC read error
}
```

## 🔌 Hardware konfiguracija

### ADC kanali (STM32F413)

| Sensor | ADC Channel | Pin | OC Pin |
|--------|-------------|-----|--------|
| LEM_1 | ADC_IN0 | PA0 | PA12 |
| LEM_2 | ADC_IN1 | PA1 | PB11 |
| LEM_3 | ADC_IN2 | PA2 | PB14 |
| LEM_4 | ADC_IN3 | PA3 | PC6 |
| LEM_5 | ADC_IN4 | PA4 | PC13 |
| LEM_6 | ADC_IN5 | PA5 | PE3 |
| LEM_7 | ADC_IN6 | PA6 | PE9 |
| LEM_8 | ADC_IN7 | PA7 | PF15 |
| LEM_9 | ADC_IN8 | PB0 | PG8 |
| LEM_10 | ADC_IN9 | PB1 | PG9 |

### LEM HOYS pinout

```
LEM HOYS-S/SP33
===============
Pin 1: +Vcc (3.3V or 5V)
Pin 2: GND
Pin 3: Vout (analog output)
Pin 4: OC (overcurrent - active HIGH)
```

## ⚡ Napotki za uporabo

### ✅ DOBRA PRAKSA

```c
// 1. Inicializiraj ADC pred LEM inicializacijo
MX_ADC1_Init();

// 2. Počakaj na stabilizacijo napajanja
HAL_Delay(100);

// 3. Inicializiraj LEM senzorje
LEM_Config_Init(&hadc1);

// 4. POMEMBNO: Kalibracija pri 0A!
// Prepričaj se, da NI toka skozi senzorje
LEM_Config_CalibrateAll(100);

// 5. Uporabljaj filtriran read za stabilnejše vrednosti
LEM_Config_ReadCurrentFiltered(0, &current_A);
```

### ❌ IZOGIBAJ SE

```c
// NE kalibriraj med obratovanjem (s tokom)
// LEM_Config_CalibrateAll(100);  // ❌ Napaka!

// NE uporabljaj premajhnega filtra za hrupne signale
// LEM_HOYS_SetFilterDepth(&lem_sensors[0], 1);  // ❌ Preveč hrupa

// NE pozabi preveriti return status
// LEM_Config_ReadCurrent(0, &current_A);  // ❌ Brez preverjanja
```

## 📈 Tipične vrednosti

### Offset voltage (pri 0A)
- Tipično: 1.65V (pri 3.3V napajanju)
- ADC vrednost: ~2048 (12-bit ADC)

### Občutljivost (HOYS-25A)
- Sensitivity: 6 mV/A
- Primer: 10A → 60mV offset → ~1.71V ali 1.59V

### Območje meritev
- Nominal: ±25A
- Max: ±50A (2x nominal)

## 🐛 Troubleshooting

### Problem: Vse vrednosti 0A
**Rešitev**: Preveri ali je ADC pravilno inicializiran

```c
// Preveri raw ADC vrednost
uint16_t adc_val;
LEM_HOYS_ReadRawADC(&lem_sensors[0], &adc_val);
printf("ADC: %u\n", adc_val);  // Mora biti ~2048
```

### Problem: Nepravilne vrednosti
**Rešitev**: Ponovno kalibriraj pri 0A

```c
LEM_Config_CalibrateAll(100);
```

### Problem: Overcurrent constant trigger
**Rešitev**: Preveri OC pin pull-up/pull-down

```c
// OC pin mora biti pravilno konfiguriran v CubeMX
// Tipično: Input, No pull-up/pull-down
```

## 📝 Primer uporabe

Glej `main.c` za popoln primer integracije z:
- Temperature logging
- FRAM storage
- BTT6200 driver
- UART diagnostics

## 📚 Dodatne informacije

- **Datasheet**: LEM HOYS-S/SP33 datasheet
- **STM32 HAL**: STM32F4 HAL documentation
- **Projekt**: Battery Control Unit (BMU IOC)

---

**Avtor**: BMU IOC Project
**Datum**: 2025
**Version**: 1.0
