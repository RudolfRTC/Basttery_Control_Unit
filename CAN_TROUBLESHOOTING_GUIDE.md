# CAN RX Troubleshooting Guide - "ČE ŠE VEDNO NE DELA"

## Kako uporabiti ta guide:

1. **Kompiliraj kodo** z diagnostiko (commit 0c5f8c8)
2. **Zaženi program** na BMU
3. **Poglej UART terminal** - diagnostika se avtomatično zažene
4. **Najdi tvoj symptom** spodaj
5. **Sledi navodilom** za rešitev

---

## 📊 DIAGNOSTIKA - Kaj vidiš?

### SCENARIJ 1: Diagnostika se NE izpiše

**Symptomi:**
```
CAN bus initialized OK (500 kbps)
CAN1 Error Code: 0x00000000
[... nothing else ...]
```

**Vzrok:** Diagnostika ni vključena ali program crash-a

**Rešitev:**
```c
// V main.c linija 248, preveri:
#if 1  // MORA biti 1, ne 0!
CAN_RunDiagnostics(&hcan1, &huart1);
#endif
```

Če je 1 in še vedno nič:
→ Program se crash-a v diagnostiki
→ Preveri stack size (povečaj na 0x1000)

---

### SCENARIJ 2: TEST 1 kaže napake

**Primer output:**
```
TEST 1: CAN Peripheral State
  CAN State: 0x03 (NOT READY - ERROR!)
  CAN Error Code: 0x00000004
  ✗ ERROR DETECTED!
    - Ack Error (no other node!)
```

#### Problem A: "Ack Error (no other node!)"
**Pomen:** CAN sporočilo poslano, ampak **noben drug node ni ACK-al**

**To je NORMALNO če:**
- Testiraš brez drugega CAN node-a na busu
- PCAN adapter ni connected
- CAN termination (120Ω) manjka

**Rešitev:**
1. Preveri da je PCAN connected in "Bus: ON"
2. Dodaj 120Ω termination resistor na CAN H/L
3. ALI: Ignoriraj - to je pričakovano pri testiranju

**To NI problem za RX!** Lahko še vedno sprejemaš sporočila.

#### Problem B: "Bus-Off"
**Pomen:** Preveč napak, CAN controller se je izključil

**Rešitev:**
```c
// V main.c po BMU_CAN_Init():
HAL_CAN_ResetError(&hcan1);
HAL_CAN_Stop(&hcan1);
HAL_CAN_Start(&hcan1);
```

#### Problem C: "CAN State: 0x00 (NOT READY)"
**Pomen:** CAN sploh ni inicializiran

**Rešitev:**
- Preveri da `BMU_CAN_Init()` vrne HAL_OK
- Preveri clock enable za CAN1

---

### SCENARIJ 3: TEST 2 kaže NVIC problem

**Primer output:**
```
TEST 2: NVIC Interrupt Configuration
  ✗ CAN1_RX0 interrupt DISABLED in NVIC!
    Priority: 0
  ✗ CAN1_TX interrupt DISABLED in NVIC!
```

**🔴 TO JE PROBLEM!** Interrupt handlerji niso pravilno povezani.

**Rešitev:**

1. **Preveri stm32f4xx_hal_msp.c:**
```c
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
  if(hcan->Instance==CAN1)
  {
    // MORA biti tukaj:
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);  // ← KRITIČNO!
  }
}
```

2. **Preveri stm32f4xx_it.c:**
```c
// MORA obstajati:
void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}
```

3. **Preveri da je compiled:**
```bash
arm-none-eabi-nm build/your.elf | grep CAN1_RX0_IRQHandler
# Mora pokazati: 080xxxxx T CAN1_RX0_IRQHandler
```

4. **Če ni:**
→ Datoteka ni bila dodana v build system
→ Preveri Makefile ali IDE project settings

---

### SCENARIJ 4: TEST 3 kaže IER problem

**Primer output:**
```
TEST 3: CAN Hardware Registers
  CAN1->IER:  0x00008F00
  ✗ FIFO0 interrupt DISABLED in CAN!
```

**Pomen:** Interrupt enable bit ni nastavljen v CAN peripheral

**Rešitev:**

Preveri `BMU_CAN_Init()`:
```c
HAL_StatusTypeDef BMU_CAN_Init(...)
{
  // ...
  HAL_CAN_Start(hcan1);

  // TO MORA BIT TUKAJ:
  HAL_CAN_ActivateNotification(hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  //                                   ^^^^^^^^^^^^^^^^^^^^^^^^^ POMEMBNO!

  return HAL_OK;
}
```

Brez tega, hardware interrupt se NE sproži!

---

### SCENARIJ 5: TEST 4 loopback FAIL

**Primer output:**
```
TEST 4: CAN Loopback Test
  Testing internal loopback mode...
  → Sent test message ID:0x123 in loopback
  ✗ No message received in loopback
  ✗ Loopback test FAILED
  → CAN peripheral hardware problem!
```

**🔴 RESNO!** CAN hardware sploh ne dela.

**Možni vzroki:**
1. CAN clock ni pravilno nastavljen
2. GPIO pins niso pravilno configured (AF9)
3. Hardware fault na chip-u

**Rešitev:**

1. **Preveri CAN clock:**
```c
// V HAL_CAN_MspInit():
__HAL_RCC_CAN1_CLK_ENABLE();  // MORA biti klican!
```

2. **Preveri GPIO alternate function:**
```c
// V HAL_CAN_MspInit():
GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;  // MORA biti AF9!
HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
```

3. **Preveri APB1 clock:**
```c
// CAN1 je na APB1, mora biti 16 MHz
// V system_stm32f4xx.c ali main.c SystemClock_Config()
```

4. **Če vse OK:**
→ Hardware problem (chip fault)
→ Poskusi CAN2 namesto CAN1

---

### SCENARIJ 6: Vse testi OK, ampak RX NE dela

**Primer output:**
```
TEST 1: ✓ No errors
TEST 2: ✓ CAN1_RX0 interrupt ENABLED
TEST 3: ✓ FIFO0 interrupt ENABLED in CAN
TEST 4: ✓ Loopback test PASSED

[... posljem CAN message ...]
[... NI debug output-a ...]
```

**Možni vzroki:**

#### A: Filter blokira sporočila

**Preveri filter config:**
```c
HAL_StatusTypeDef BMU_CAN_ConfigureFilter(CAN_HandleTypeDef* hcan)
{
  CAN_FilterTypeDef filter;

  // Za accept ALL:
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;  // 0x0000 = accept all!
  filter.FilterMaskIdLow = 0x0000;

  // MORA biti:
  filter.FilterActivation = ENABLE;  // NOT DISABLE!

  return HAL_CAN_ConfigFilter(hcan, &filter);
}
```

#### B: Callback se ne pokliče

**Dodaj test v main.c:**
```c
// V main loop:
if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
  HAL_UART_Transmit(&huart1, "FIFO HAS MESSAGE!\r\n", 19, 100);

  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
    HAL_UART_Transmit(&huart1, "MESSAGE READ OK!\r\n", 18, 100);
  }
}
```

Če vidiš "FIFO HAS MESSAGE!" ampak NI interrupt:
→ **NVIC interrupt handler NI povezan!**

#### C: Debug output ne dela

**Preveri da je vključeno:**
```c
// V bmu_can.c linija 551:
#if 1  // MORA biti 1!
extern UART_HandleTypeDef huart1;
char debug_buf[150];
snprintf(debug_buf, sizeof(debug_buf), "[CAN RX] ...");
HAL_UART_Transmit(&huart1, ...);
#endif
```

#### D: Interrupt stack overflow

**Povečaj stack size:**
```
// V .ld file ali startup file:
_Min_Stack_Size = 0x1000;  // Povečaj iz 0x400
```

---

### SCENARIJ 7: Vidim [CAN RX] ampak "Processing FAILED"

**Primer output:**
```
[CAN RX] ID:0x200 DLC:8 Data: 00 01 00 00 DE AD BE EF
[CAN RX] Processing FAILED! Errors: 1
```

**Pomen:** Sporočilo je prispelo, ampak validacija faila!

#### Problem A: Magic number napačen

**Vidiš:**
```
Data: 00 01 00 00 DE AD BE EF
                    ^^^^^^^^^^^ BIG endian - NAROBE!
```

**Pošlji:**
```
Data: 00 01 00 00 EF BE AD DE
                    ^^^^^^^^^^^ LITTLE endian - PRAVILNO!
```

#### Problem B: DLC < 8

**Vidiš:**
```
[CAN RX] ID:0x200 DLC:4 Data: ...
```

**Pošlji DLC=8**, ne manj!

#### Problem C: Output ID >= 20

**Vidiš:**
```
Data: 14 01 00 00 ...  (OutputID = 0x14 = 20)
      ^^^ > 19 = out of range!
```

**Pošlji OutputID 0-19** (0x00 do 0x13)

---

## 🎯 Quick Decision Tree

```
Diagnostika se NE izpiše?
  ├─ YES → Preveri #if 1 v main.c, stack size
  └─ NO  → Nadaljuj

TEST 1 fail?
  ├─ Ack Error → NORMALNO, ignoriraj
  ├─ Bus-Off → Reset CAN, preveri termination
  └─ CAN State = 0 → Init fail, preveri clocks

TEST 2 fail (NVIC disabled)?
  ├─ YES → KRITIČNO! Dodaj interrupt handlers
  └─ NO  → Nadaljuj

TEST 3 fail (IER bit not set)?
  ├─ YES → Dodaj HAL_CAN_ActivateNotification()
  └─ NO  → Nadaljuj

TEST 4 fail (Loopback)?
  ├─ YES → Hardware problem! Preveri GPIO AF, clocks
  └─ NO  → CAN hardware OK!

Vse OK ampak RX ne dela?
  ├─ Filter blokira → Accept all (mask 0x0000)
  ├─ FIFO ima msg ampak ni interrupt → NVIC problem
  ├─ Debug output OFF → Vklopi #if 1
  └─ Stack overflow → Povečaj stack

[CAN RX] Processing FAILED?
  ├─ Magic number → Little endian (EF BE AD DE)
  ├─ DLC < 8 → Pošlji 8 bajtov
  └─ Output ID → 0-19 samo
```

---

## 📞 Zadnja možnost - Pošlji mi OUTPUT

Če še vedno ne dela, **kopiraj CELOTEN output** diagnostike in mi pošlji:

```
=== ZAČETEK ===
[vse od CAN bus initialized... do DIAGNOSTICS COMPLETE]
=== KONEC ===
```

Plus povej:
1. Ali uporabljaš PCAN-USB ali drug adapter?
2. Ali je drugi node na busu (ali si sam)?
3. Kakšno sporočilo pošiljaš (ID, DLC, Data)?
4. Kateri compiler (GCC version)?
5. Ali build warning-i?

S temi informacijami bom TOČNO vedel kaj je narobe!

---

## ✅ Če DELA

Če vidiš:
```
[CAN RX] ID:0x200 DLC:8 Data: 00 01 00 00 EF BE AD DE
[CAN RX] OK
```

**BRAVO! Dela! 🎉**

Sedaj lahko:
1. Izključiš diagnostiko (`#if 0` v main.c)
2. Izključiš debug output (`#if 0` v bmu_can.c)
3. Uporabljaš CAN control normalno!

Pošlji:
```bash
# Output 0 ON
cansend can0 200#0001.0000.EFBE.ADDE

# Output 5 OFF
cansend can0 200#0500.0000.EFBE.ADDE

# Disable VSE
cansend can0 202#0200.0000.BEBA.FECA
```

**Uživaj! 🚀**
