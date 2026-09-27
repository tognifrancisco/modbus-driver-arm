/******************************************************************************
 * @file        main.c
 * @brief       Main Application - Modbus RTU Slave Controller for F280049C (64-pin PM)
 * @version     1.0.1
 * @date        2026-06-04
 * @author      togni / adaptado para F280049C
 *
 * @description Main application for F280049C microcontroller implementing a
 *              Modbus RTU slave device. Controls LEDs based on Modbus holding
 *              registers and handles communication via SCI-A interface.
 *
 * @features    - Modbus RTU protocol implementation (Functions 0x03, 0x06)
 *              - Real-time LED control via holding registers
 *              - Interrupt-driven UART communication
 *              - Timer-based frame timeout detection
 *
 *
 * @copyright   (c) 2025-2026 Company Name. All rights reserved.
 ******************************************************************************/

// Included Files
#include "f28004x_device.h"
#include "f28004x_examples.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h> /* memset */

// Defines
#define SYSCLK_HZ               100000000UL
#define LSPCLK_HZ               25000000UL
#define BAUD_RATE               115200
#define MODBUS_ADDRESS          0x01
#define MODBUS_HOLD_REG_CNT     32
#define MODBUS_R_HOLD_REG       0x03
#define MODBUS_W_SING_REG       0x06
#define MODBUS_W_MULT_REG       0x10
#define MODBUS_RX_BUFF_SIZE     256
#define MODBUS_TX_BUFF_SIZE     256

#define TEST_LED_01     23
#define TEST_LED_02     34

#define ISO3088_RX      11
#define ISO3088_TX      12
#define ISO3088_RDE     8

typedef unsigned char   uint8_t;

// Globals
typedef struct
{
    uint8_t  address;
    uint8_t  function;
    uint8_t  rx_buffer[MODBUS_RX_BUFF_SIZE];
    uint16_t rx_index;
    uint32_t rx_timeout;
    uint16_t holding_reg[MODBUS_HOLD_REG_CNT];
    uint8_t  tx_buffer[MODBUS_TX_BUFF_SIZE];
    uint8_t  tx_lenght;
    uint16_t tx_crc;
    bool     ready;
} modbus_ctx;

uint8_t  dummy;

// Contexto Modbus global
modbus_ctx modbus;

// Function Prototypes
void ConfigGpio(void);
void Timer0_Init(void);
void SetupInterrupts(void);
void SCI_Init(void);
void SCI_SendBuffer(uint8_t *buffer, uint8_t length);
void SCI_SetBaudRate(uint32_t baudRate);
void SCI_EnableTx(void);
void SCI_EnableRx(void);
void Modbus_Init(void);
void Modbus_Poll(void);
uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len);

// Service Routines (ISR)
__interrupt void CpuTimer0ISR(void);
//__interrupt void SciaRxISR(void); // se va a usar con SUPERVISOR
__interrupt void ScibRxISR(void);

// Main
void main(void)
{
    // Initialize device clock and peripherals
    InitSysCtrl();
    // LSPCLK = SYSCLK / (LOSPCP * 2)  -> 2 * 2 = 4  => 100 MHz / 4 = 25 MHz
    //ClkCfgRegs.LOSPCP.all = 2; //010,LSPCLK = / 4 (default on reset)
    ConfigGpio();
    // Initialize the PIE control registers to all interrupts disabled and flags cleared.
    InitPieCtrl();
    // Disable CPU interrupts and clear all CPU interrupt flags
    IER = 0x0000;
    IFR = 0x0000;
    // Initialize the PIE vector table with pointers to the shell Interrupt
    InitPieVectTable();
    // Map ISR functions
    EALLOW;
    PieVectTable.TIMER0_INT = &CpuTimer0ISR;
    //PieVectTable.SCIA_RX_INT = &SciaRxISR;
    PieVectTable.SCIB_RX_INT = &ScibRxISR;
    EDIS;
    // Initialize Timer0
    Timer0_Init();
    // Initialize SCI
    SCI_Init();
    // Setup interrupts
    SetupInterrupts();
    // Initialize Modbus
    Modbus_Init();

    GPIO_WritePin(TEST_LED_01,1);
    GPIO_WritePin(TEST_LED_02,1);

    for(;;)
    {
        //GPIO_WritePin(KIT_PIN_LED5, !(modbus.holding_reg[0] & 0x0001));
        //GPIO_WritePin(KIT_PIN_LED4, !(modbus.holding_reg[1] & 0x0001));
        //GPIO_WritePin(TEST_LED_01,!GPIO_ReadPin(TEST_LED_01));
        //GPIO_WritePin(TEST_LED_02,!GPIO_ReadPin(TEST_LED_02));
        //DELAY_US(500000);

        if(modbus.ready)
            Modbus_Poll();
    }
}


void ConfigGpio(void)
{
    // Initialize GPIO
    InitGpio();
    // SCI-A TX
    //GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
    //GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);
    // SCI-A RX
    //GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
    //GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PULLUP);

    /*
    GPIO    MUX     MCU            ISO3088
                    -----------    -----------------
    12      6       TX    ---->    DI PIN 6 (naranja)
    11      6       RX    <----    RO PIN 3 (amarillo)
                    RDE   ---->    DE PIN 4 (blanco)
    */

    // SCI-B RX
    GPIO_SetupPinMux(ISO3088_RX, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(ISO3088_RX, GPIO_INPUT, GPIO_PULLUP);
    // SCI-B TX
    GPIO_SetupPinMux(ISO3088_TX, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(ISO3088_TX, GPIO_OUTPUT, GPIO_ASYNC);
    // RE/DE del transceiver
    GPIO_SetupPinMux(ISO3088_RDE, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(ISO3088_RDE, GPIO_OUTPUT, GPIO_PUSHPULL);
    // LEDs
    GPIO_SetupPinMux(TEST_LED_01, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TEST_LED_01, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TEST_LED_02, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TEST_LED_02, GPIO_OUTPUT, GPIO_PUSHPULL);
}

//==========
//  MODBUS
//==========

/* MAPPING

    hr[0]     (tag hr1 en HMI)    STATE   > X X X X X X X X X X X X RF2 RF1 RF0 EN
          EN = ENABLE (encendido/apagado de maquina)
          RF2-RF0 = codigo de boton remoto

    hr[1]     (tag hr2 en HMI)    FAIL    > X X X X F11 F10 F9 F8 F7 F6 F5 F4 F3 F2 F1 F0

          F0  V (tensiones en PCB fuente)
          F1  GIRO/ACELER
          F2  UART1 (DSP <> HMI)
          F3  UART2 (DSP <> SUP)
          F4  FEED (pelotas en tubo alim)
          F5  MOT 1 (falla motor universal 1, superior)
          F6  MOT 2 (falla motor universal 2, inferior)
          F7  TILT (falla motor DC 1)
          F8  AZIMUTH (falla motor DC 2)
          F9  FBK 1 (falla sensor Hall 1)
          F10 FBK 2 (falla sensor Hall 2)
          F11 POW (falla AC 220 gral.)

     hr[2]    (tag hr3 en HMI)    VEL (pelota)

     hr[3]    (tag hr4 en HMI)    TARGET   > X X X X X SP3 SP2 SP1 SP0 P1 P0 S4 S3 S2 S1 S0
            S4-S0       SECTOR (0-31)
            P1-P0       PIQUE (30,40,50,60 GRADOS)
            SP3-SP0     SPIN (SIGNADO, 8 VALORES)
            X           RESERVADO

     hr[4]    (tag hr5 en HMI)    TILT
     hr[5]    (tag hr6 en HMI)    AZIMUTH
     hr[6]    (tag hr7 en HMI)    KP 1  (0-100.00)
     hr[7]    (tag hr8 en HMI)    KI 1  (0-100.00)
     hr[8]    (tag hr9 en HMI)    KD 1  (0-100.00)
     hr[9]    (tag hr10 en HMI)   RPM 1 (0-100.00) DEFINIR MAX!
     hr[10]   (tag hr11 en HMI)   KP 2  (0-100.00)
     hr[11]   (tag hr12 en HMI)   KI 2  (0-100.00)
     hr[12]   (tag hr13 en HMI)   KD 2  (0-100.00)
     hr[13]   (tag hr14 en HMI)   RPM 2 (0-100.00) DEFINIR MAX!

*/

void Timer0_Init(void)
{
    // Enable CPUTIMER0 clock
    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;
    // stop CPU-Timer 0
    CpuTimer0Regs.TCR.bit.TSS = 1; // 1h (R/W) = Reads of 1 indicate that the CPU-timer is stopped.
    // Counter decrements PRD+1 times each period
    CpuTimer0Regs.PRD.all = ((SYSCLK_HZ / BAUD_RATE) * 35) - 1; // 10 bits por dato, 3.5 char timeout
    // Set pre-scale counter to divide by 1 (SYSCLKOUT)
    CpuTimer0Regs.TPR.all  = 0;
    CpuTimer0Regs.TPRH.all  = 0;

    /* 1h (R/W) = When you write a 1 to TRB, the TIMH:TIM is loaded with
    the value in the PRDH:PRD, and the prescaler counter (PSCH:PSC)
    is loaded with the value in the timer dividedown register (TDDRH:TDDR).*/
    CpuTimer0Regs.TCR.bit.TRB = 1;

    CpuTimer0Regs.TCR.bit.TIE = 1; // 1h (R/W) = The CPU-Timer interrupt is enabled
    //CpuTimer0Regs.TCR.bit.TSS = 0; // To start or restart the CPU-timer, set TSS to 0 -------------------------------------------
}

void SCI_Init(void)
{
    CpuSysRegs.PCLKCR7.bit.SCI_B = 1;
    // 1. SCI en reset, ambos FIFOs en reset, modo FIFO habilitado
    ScibRegs.SCICTL1.bit.SWRESET = 0; // SCI software reset (active low).
    ScibRegs.SCIFFTX.bit.SCIRST = 0; //  A write of 0 will cause a SW RESET
    ScibRegs.SCIFFTX.bit.TXFIFORESET = 0; //0h (R/W) = Reset the FIFO pointer to zero and hold in reset
    ScibRegs.SCIFFTX.bit.SCIFFENA = 1; //SCI FIFO enable
    // 2. Formato de carÃ¡cter y habilitaciÃ³n de RX/TX
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 0; //Write 0 to reset the FIFO pointer to zero, and hold in reset.
    ScibRegs.SCIFFRX.bit.RXFFIENA = 0; // se deshabilita momentaneamente INT

    /*
    In FIFO mode, the SPI can interrupt the CPU upon a match condition between the current receive FIFO status
    (RXFFST) and the receive FIFO interrupt level (RXFFIL). If RXFFST is greater than or equal to RXFFIL, the
    receive FIFO interrupt flag (RXFFINT) is set. SPIRXINT is triggered in the PIE block, if RXFFINT is set and the
    receive FIFO interrupt is enabled (RXFFIENA = 1).
    */
    //ScibRegs.SCIFFRX.bit.RXFFST = x; // FIFO status, 1h (R/W) = Receive FIFO has 1 words
    ScibRegs.SCIFFRX.bit.RXFFIL = 8; // (RXFFST >= RXFFIL) ---> interrupt ----------------------------------------!!!!

    ScibRegs.SCICCR.bit.SCICHAR = 7; //7h (R/W) = SCICHAR_LENGTH_8
    ScibRegs.SCICCR.bit.PARITYENA = 0; //0h (R/W) = Parity disabled
    ScibRegs.SCICCR.bit.STOPBITS = 0; //0h (R/W) = One stop bit
    ScibRegs.SCICTL1.bit.TXENA = 1; //1h (R/W) = Transmitter enabled
    ScibRegs.SCICTL1.bit.RXENA = 1; //1h (R/W) = Send received characters to SCIRXEMU and SCIRXBUF
    ScibRegs.SCICTL2.bit.TXINTENA = 0; //0h (R/W) = Disable TXRDY interrupt
    ScibRegs.SCICTL2.bit.RXBKINTENA = 1; //1h (R/W) = Enable RXRDY/BRKDT interrupt
    // 3. Baudrate
    SCI_SetBaudRate(BAUD_RATE);
    // 4. InterrupciÃ³n
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1; //Write 1 to clear RXFFINT flag in bit 7
    ScibRegs.SCIFFRX.bit.RXFFIENA = 1; //1h (R/W) = RX FIFO interrupt is enabled.
    // 5. Sacar ambos FIFOs de reset
    ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
    // 6. Liberar el reset principal del SCI (CRÃ�TICO)
    ScibRegs.SCIFFTX.bit.SCIRST = 1; //  A write of 0 will cause a SW RESET
    // 7. Liberar el reset por software
    ScibRegs.SCICTL1.bit.SWRESET = 1; //SCI software reset (active low).
    // 8. Poner el transceiver en modo recepciÃ³n
    SCI_EnableRx();
}

// Set baud rate
void SCI_SetBaudRate(uint32_t baudRate)
{
    // BRR = LSPCLK / (SCI Asynchronous Baud * 8) - 1 = (HBAUD << 8) + (LBAUD)
    uint32_t baudRateDiv;
    baudRateDiv = LSPCLK_HZ / (baudRate * 8) - 1;
    //SciaRegs.SCILBAUD.all = baudRateDiv & 0xFF;
    //SciaRegs.SCIHBAUD.all = (baudRateDiv >> 8) & 0xFF;
    ScibRegs.SCILBAUD.all = baudRateDiv & 0xFF;
    ScibRegs.SCIHBAUD.all = (baudRateDiv >> 8) & 0xFF;
}

// Transmit a buffer of data (solo envia SCIB)
void SCI_SendBuffer(uint8_t *buffer, uint8_t length)
{
    SCI_EnableTx();
    uint8_t i;
    for(i = 0; i < length; i++)
    {
        // Wait until TX FIFO
        while(ScibRegs.SCICTL2.bit.TXRDY == 0) {} // 0h (R/W) = SCITXBUF is full
        ScibRegs.SCITXBUF.all = buffer[i];
    }
    SCI_EnableRx();
}

void SCI_EnableTx()
{
    while(ScibRegs.SCICTL2.bit.TXEMPTY == 0) {}
    GPIO_WritePin(ISO3088_RDE,1);   // DE/RE = HIGH (modo TX)
    DELAY_US(100); // OJO con este retardo, acomodar si falla modbus!! 50
}

void SCI_EnableRx()
{
    while(ScibRegs.SCICTL2.bit.TXEMPTY == 0) {}
    DELAY_US(200); // OJO con este retardo, acomodar si falla modbus!! 150
    GPIO_WritePin(ISO3088_RDE,0);   // DE/RE = LOW (modo RX)
}

void Modbus_Init(void)
{
    modbus.address = 0;
    modbus.rx_index = 0;
    modbus.rx_timeout = 0;
    modbus.ready = 0;

    uint16_t i;
    for(i = 0; i < MODBUS_HOLD_REG_CNT; i++)
        modbus.holding_reg[i] = 50;

    memset(modbus.rx_buffer, 0, MODBUS_RX_BUFF_SIZE);
    memset(modbus.tx_buffer, 0, MODBUS_TX_BUFF_SIZE);
}

void Modbus_Poll(void)
{
    uint16_t len = modbus.rx_index;
    modbus.rx_index = 0;
    modbus.ready = 0;

    //if(len < 4) //--------------------------------------------------------------------------------
    //    return;

    modbus.address  = modbus.rx_buffer[0];
    modbus.function = modbus.rx_buffer[1];

    if(modbus.address != MODBUS_ADDRESS && modbus.address != 0)
        return;



    uint16_t crc_recv = modbus.rx_buffer[len-2] | (modbus.rx_buffer[len-1] << 8);
    uint16_t crc_calc = Modbus_CRC16((uint8_t*)modbus.rx_buffer, len - 2);
    if(crc_recv != crc_calc)
        return;

    switch(modbus.function)
    {
        case MODBUS_R_HOLD_REG:
        {
            uint16_t start = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
            uint16_t count = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];

            if(count > MODBUS_HOLD_REG_CNT)
                return;

            modbus.tx_buffer[0] = MODBUS_ADDRESS;
            modbus.tx_buffer[1] = MODBUS_R_HOLD_REG;
            modbus.tx_buffer[2] = count * 2;

            uint16_t i;
            for(i = 0; i < count; i++)
            {
                modbus.tx_buffer[3 + 2*i] = modbus.holding_reg[start+i] >> 8;
                modbus.tx_buffer[4 + 2*i] = modbus.holding_reg[start+i] & 0xFF;
            }
            modbus.tx_lenght = 3 + (count * 2);
        }
        break;

        case MODBUS_W_SING_REG:
        {
            uint16_t reg = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
            uint16_t val = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
            if(reg < MODBUS_HOLD_REG_CNT)
                modbus.holding_reg[reg] = val;
            memcpy(modbus.tx_buffer, modbus.rx_buffer, 6);
            modbus.tx_lenght = 6;
        }
        break;

        default:
            return;
    }

    modbus.tx_crc = Modbus_CRC16(modbus.tx_buffer, modbus.tx_lenght);
    modbus.tx_buffer[modbus.tx_lenght++] = modbus.tx_crc & 0xFF;
    modbus.tx_buffer[modbus.tx_lenght++] = modbus.tx_crc >> 8;
    SCI_SendBuffer(modbus.tx_buffer, modbus.tx_lenght);
}

uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t j;
    for(i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for(j = 0; j < 8; j++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
    return crc;
}

void SetupInterrupts(void)
{
    IER |= M_INT1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // timer0
    IER |= M_INT9;
    //PieCtrlRegs.PIEIER9.bit.INTx1 = 1; // scia
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1; // scib
    EINT;
    ERTM;
}

__interrupt void CpuTimer0ISR(void)
{
    //if(modbus.rx_timeout < 5)
    //    modbus.rx_timeout++;
    //if((modbus.rx_timeout == 4) && (modbus.rx_index > 0))
    //    modbus.ready = 1;
    //PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    // 10 bits por dato, 3.5 char timeout
    // el timer esta configurado para ((SYSCLK_HZ / BAUD_RATE) * 35) - 1 = 30379
    if (ScibRegs.SCIFFRX.bit.RXFFST == 8) // sin cambios, bytes quietos
    {
        //uint8_t i;
        for (modbus.rx_index = 0; modbus.rx_index < 8; modbus.rx_index++)
            modbus.rx_buffer[modbus.rx_index] = ScibRegs.SCIRXBUF.all;
        ScibRegs.SCIFFRX.bit.RXFIFORESET = 0; // limpia FIFO
        ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
        modbus.ready = true;
    }

    CpuTimer0Regs.TCR.bit.TSS = 1;          // detener timer0
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    asm(" NOP");
}





__interrupt void ScibRxISR(void) // MODBUS
{
    //modbus.rx_buffer[modbus.rx_index++] = ScibRegs.SCIRXBUF.all;
    //modbus.rx_timeout = 0;
    //ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    //PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

    CpuTimer0Regs.TCR.bit.TSS = 0; // To start or restart the CPU-timer, set TSS to 0
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1; // limpia flag int
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    asm(" NOP");
}

__interrupt void SciaRxISR(void) // SUPERVISOR
{
    dummy = SciaRegs.SCIRXBUF.all;
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    asm(" NOP");
}
