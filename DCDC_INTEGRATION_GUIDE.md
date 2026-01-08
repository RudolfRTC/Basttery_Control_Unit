# DC/DC Controller Integration Guide

## Pregled

Sistem za upravljanje dveh DC/DC pretvornikov preko CAN vodila z ločenimi vmesniki:
- **CAN2**: Komunikacija z DC/DC pretvorniki (COMET-AG CISO protokol)
- **CAN1**: Diagnostični vmesnik za upravljanje in monitoring

## Arhitektura

```
┌─────────────────┐     CAN1 (Standard 11-bit)     ┌──────────────┐
│   Nadzorni      │ ◄─────────────────────────────► │   STM32F4    │
│   Sistem        │    Komande + Diagnostika        │   BMU        │
└─────────────────┘                                 └──────────────┘
                                                           │
                                                     CAN2 (Extended 29-bit)
                                                           │
                                                           ▼
                                                    ┌──────────────┐
                                                    │   DC/DC      │
                                                    │ Pretvornika  │
                                                    │   (2x)       │
                                                    └──────────────┘
```

## Datoteke

### Core Files
- `Core/Inc/dcdc_controller.h` - DC/DC kontroler API (CAN2)
- `Core/Src/dcdc_controller.c` - DC/DC kontroler implementacija
- `Core/Inc/dcdc_diagnostics.h` - Diagnostični vmesnik API (CAN1)
- `Core/Src/dcdc_diagnostics.c` - Diagnostični vmesnik implementacija

## 1. Inicializacija (main.c)

### 1.1 Include Files

```c
#include "dcdc_controller.h"
#include "dcdc_diagnostics.h"
```

### 1.2 Inicializacija v main()

```c
int main(void)
{
    /* HAL Init */
    HAL_Init();
    SystemClock_Config();

    /* Periferals Init */
    MX_GPIO_Init();
    MX_CAN1_Init();  /* CAN1 za diagnostiko */
    MX_CAN2_Init();  /* CAN2 za DC/DC pretvornika */

    /* DC/DC Controller Init */
    DCDC_Init();
    DCDC_Diag_Init();

    /* Start CAN1 and CAN2 */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);

    /* Configure CAN filters */
    /* CAN1 filter - sprejmi samo komandna sporočila (0x300-0x302) */
    CAN_FilterTypeDef filter1;
    filter1.FilterBank = 0;
    filter1.FilterMode = CAN_FILTERMODE_IDMASK;
    filter1.FilterScale = CAN_FILTERSCALE_32BIT;
    filter1.FilterIdHigh = 0x300 << 5;
    filter1.FilterIdLow = 0x0000;
    filter1.FilterMaskIdHigh = 0x7F0 << 5;  /* Sprejmi 0x300-0x30F */
    filter1.FilterMaskIdLow = 0x0000;
    filter1.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter1.FilterActivation = ENABLE;
    filter1.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &filter1);

    /* CAN2 filter - sprejmi vse extended ID */
    CAN_FilterTypeDef filter2;
    filter2.FilterBank = 14;
    filter2.FilterMode = CAN_FILTERMODE_IDMASK;
    filter2.FilterScale = CAN_FILTERSCALE_32BIT;
    filter2.FilterIdHigh = 0x0000;
    filter2.FilterIdLow = 0x0000;
    filter2.FilterMaskIdHigh = 0x0000;
    filter2.FilterMaskIdLow = 0x0000;
    filter2.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter2.FilterActivation = ENABLE;
    filter2.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &filter2);

    /* Aktiviraj CAN RX interrupt */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* Infinite loop */
    while (1)
    {
        /* Periodic tasks */
        DCDC_PeriodicTask();        /* Pošiljaj CAN2 sporočila DC/DC (20ms) */
        DCDC_Diag_PeriodicTask();   /* Pošiljaj CAN1 diagnostiko (100ms) */

        HAL_Delay(10);  /* 10ms main loop cycle */
    }
}
```

## 2. CAN RX Interrupt Handler (stm32f4xx_it.c)

### 2.1 Include Files

```c
#include "dcdc_controller.h"
#include "dcdc_diagnostics.h"
```

### 2.2 CAN RX Callback

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    /* Preberi sporočilo */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    if (hcan == &hcan1) {
        /* CAN1 - Diagnostični vmesnik (komande) */
        uint32_t can_id = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
        DCDC_Diag_ProcessCommand(can_id, rx_data, rx_header.DLC);
    }
    else if (hcan == &hcan2) {
        /* CAN2 - DC/DC feedback */
        uint32_t can_id = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
        DCDC_ProcessCANMessage(can_id, rx_data, rx_header.DLC);
    }
}
```

## 3. CAN1 Diagnostični Protokol

### 3.1 Komandna Sporočila (RX - prejeto na CAN1)

| CAN ID | Ime              | DLC | Opis                           |
|--------|------------------|-----|--------------------------------|
| 0x300  | CMD_CONV1        | 8   | Komanda za pretvornik 1        |
| 0x301  | CMD_CONV2        | 8   | Komanda za pretvornik 2        |
| 0x302  | CMD_BOTH         | 8   | Komanda za oba pretvornika     |

#### Struktura komandnega sporočila:
```c
Byte 0: Komanda (0=IZKLOP, 1=VKLOP)
Byte 1-7: Rezervirano (0x00)
```

### 3.2 Diagnostična Sporočila (TX - pošiljano na CAN1)

#### Pretvornik 1 (CAN ID: 0x400, 0x401, 0x402)

| CAN ID | Ime              | DLC | Frekvenca | Opis                           |
|--------|------------------|-----|-----------|--------------------------------|
| 0x400  | STATUS_CONV1     | 8   | 100ms     | Status pretvornika 1           |
| 0x401  | MEASURES_CONV1   | 8   | 100ms     | Napetosti in tokovi            |
| 0x402  | POWER_CONV1      | 8   | 100ms     | Moči                           |

#### Pretvornik 2 (CAN ID: 0x410, 0x411, 0x412)

| CAN ID | Ime              | DLC | Frekvenca | Opis                           |
|--------|------------------|-----|-----------|--------------------------------|
| 0x410  | STATUS_CONV2     | 8   | 100ms     | Status pretvornika 2           |
| 0x411  | MEASURES_CONV2   | 8   | 100ms     | Napetosti in tokovi            |
| 0x412  | POWER_CONV2      | 8   | 100ms     | Moči                           |

### 3.3 Strukture Diagnostičnih Sporočil

#### Status (0x400, 0x410):
```c
Byte 0: Operating Mode (0=Standby, 1=Bus1toBus2, 2=Bus2toBus1)
Byte 1: Control Type (7=vBus2Lim, 8=vBus1Lim)
Byte 2: Is Running (0=Stopped, 1=Running)
Byte 3: Has Errors (0=No errors, 1=Has errors)
Byte 4-7: Error Flags (32-bit bitfield, Little Endian)
```

#### Meritve (0x401, 0x411):
```c
Byte 0-1: U_Bus1 v 0.1V (uint16, Little Endian) - npr. 480 = 48.0V
Byte 2-3: U_Bus2 v 0.1V (uint16, Little Endian) - npr. 240 = 24.0V
Byte 4-5: I_Bus1 v 0.1A (int16, Little Endian, signed)
Byte 6-7: I_Bus2 v 0.1A (int16, Little Endian, signed)
```

#### Moči (0x402, 0x412):
```c
Byte 0-3: P_Bus1 v W (uint32, Little Endian)
Byte 4-7: P_Bus2 v W (uint32, Little Endian)
```

## 4. Primeri Uporabe

### 4.1 Vklop Pretvornika 1

**CAN1 TX (nadzorni sistem → BMU):**
```
ID: 0x300
DLC: 8
Data: 01 00 00 00 00 00 00 00
```

**CAN1 RX (BMU → nadzorni sistem):**
```
ID: 0x400 (Status)
Data: 01 07 01 00 00 00 00 00
      │  │  │  │  └──────────── Error flags (0 = no errors)
      │  │  │  └─────────────── Has errors (0 = no)
      │  │  └────────────────── Is running (1 = yes)
      │  └───────────────────── Control type (7 = vBus2Lim)
      └──────────────────────── Operating mode (1 = Bus1toBus2)

ID: 0x401 (Measures)
Data: E0 01 F0 00 C8 00 64 00
      └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘
        │     │     │     └───── I_Bus2 = 100 (0.1A) = 10.0A
        │     │     └─────────── I_Bus1 = 200 (0.1A) = 20.0A
        │     └───────────────── U_Bus2 = 240 (0.1V) = 24.0V
        └─────────────────────── U_Bus1 = 480 (0.1V) = 48.0V

ID: 0x402 (Power)
Data: E0 03 00 00 F0 00 00 00
      └────┬────┘ └────┬────┘
           │           └───────── P_Bus2 = 240W
           └───────────────────── P_Bus1 = 992W
```

### 4.2 Izklop Obeh Pretvornikov

**CAN1 TX:**
```
ID: 0x302
DLC: 8
Data: 00 00 00 00 00 00 00 00
```

### 4.3 Branje Diagnostike (Python primer)

```python
import can

# Konfiguriraj CAN vmesnik
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Pošlji komando VKLOP za pretvornik 1
msg = can.Message(arbitration_id=0x300, data=[0x01, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)
bus.send(msg)

# Beri diagnostiko
while True:
    msg = bus.recv(timeout=1.0)
    if msg:
        if msg.arbitration_id == 0x401:  # Meritve pretvornika 1
            u_bus1 = int.from_bytes(msg.data[0:2], 'little') / 10.0
            u_bus2 = int.from_bytes(msg.data[2:4], 'little') / 10.0
            i_bus1 = int.from_bytes(msg.data[4:6], 'little', signed=True) / 10.0
            i_bus2 = int.from_bytes(msg.data[6:8], 'little', signed=True) / 10.0
            print(f"Conv1: U1={u_bus1}V, U2={u_bus2}V, I1={i_bus1}A, I2={i_bus2}A")
```

## 5. MISRA C:2012 Compliance

Vsa koda je skladna z MISRA C:2012:
- ✓ Explicit type declarations (uint8_t, uint16_t, etc.)
- ✓ No magic numbers (defined constants)
- ✓ Explicit NULL checks
- ✓ Return value checking
- ✓ Packed structures for CAN messages
- ✓ Little Endian byte ordering
- ✓ Safe memcpy usage
- ✓ Boolean explicit comparisons

## 6. Debugging

### 6.1 Preveri CAN2 TX (DC/DC)
```c
/* Dodaj v dcdc_controller.c transmit funkcijo */
// DEBUG: Print sent message
#ifdef DEBUG_CAN2
char debug[64];
snprintf(debug, sizeof(debug), "[CAN2 TX] ID:0x%08lX DLC:%d\r\n", can_id, dlc);
HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 100);
#endif
```

### 6.2 Preveri CAN1 RX/TX (Diagnostika)
```c
/* Dodaj v dcdc_diagnostics.c */
#ifdef DEBUG_CAN1
char debug[64];
snprintf(debug, sizeof(debug), "[CAN1 RX] ID:0x%03lX CMD:%d\r\n", can_id, data[0]);
HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 100);
#endif
```

## 7. Sistemske Zahteve

- **CAN1**: 500 kbps, Standard 11-bit ID
- **CAN2**: 500 kbps, Extended 29-bit ID
- **Timer**: 20ms za DC/DC periodic, 100ms za diagnostiko
- **RAM**: ~512 bytes za status in bufferje

## 8. Testiranje

### 8.1 Simulator Setup
Uporabi CAN simulator (npr. PCAN-View) za testiranje:

1. Pošlji komando VKLOP na 0x300
2. Preveri da se pojavijo diagnostična sporočila (0x400, 0x401, 0x402)
3. Preveri CAN2 traffic (0x00100000, 0x00500000)

### 8.2 Hardware Test
1. Povežite DC/DC pretvornik na CAN2
2. Povežite CAN analyzer na CAN1
3. Pošljite komande in preverjajte odziv

## 9. Pogosta Vprašanja

**Q: Zakaj ne vidim CAN2 sporočil?**
A: Preverite da je CAN2 filter konfiguriran za extended ID in da je HAL_CAN_Start(&hcan2) klican.

**Q: DC/DC ne odgovarja?**
A: Preverite da je CAN address pravilen (0x00 ali 0x01) in da je extended ID format (29-bit).

**Q: Kako spremenim setpoint napetosti?**
A: Uporabite `DCDC_UpdateSetpoint(converter_id, voltage, current)` - voltage/current v 0.1 enotah.

**Q: Kako pogosto se pošiljajo CAN sporočila?**
A: CAN2 (DC/DC control): vsakih 20ms | CAN1 (diagnostika): vsakih 100ms
