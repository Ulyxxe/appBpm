//-----------------------------------------------------------------------------
//  OLedAffi_Fonc.c
// 
//-----------------------------------------------------------------------------

#include "OLedAffi_Vars.h"



/* L'adresse semble etre 0x3C sur 7 bits, soit 0x78 sur 8 bits */
#define LCD_SLAVE_ADDR      0x78


void    TIVA_I2C1_InitModule(void);
short   TIVA_I2C1_SendData(char slaveAdr, short nbdata, char* pBuff);


static void    AOLED_WriteCmde(char cmde)
//-------------------------------------------------------------
//-------------------------------------------------------------
{   char Buff[4];

    Buff[0] = 0x00;     Buff[1] = cmde;   // 00 indique le mode commande
    TIVA_I2C1_SendData(LCD_SLAVE_ADDR, 2, Buff);
}

static void    AOLED_WriteData(char data)
//-------------------------------------------------------------
//-------------------------------------------------------------
{    char Buff[4];

    Buff[0] = 0x40;     Buff[1] = data;   // 40 indique le mode commande
    TIVA_I2C1_SendData(LCD_SLAVE_ADDR, 2, Buff);
}

static void     AOLED_SetAdrsLin(char addrs)
//-------------------------------------------------------------
//-------------------------------------------------------------
{
    addrs = 0xB0 | (addrs & 0x07);
    AOLED_WriteCmde(addrs);
}

static void     AOLED_SetAdrsCol(char addrs)
//-------------------------------------------------------------
//-------------------------------------------------------------
{   char msb, lsb;

    addrs += 2;
    msb = 0x10 | (addrs >> 4);     lsb = addrs & 0x0F;
    AOLED_WriteCmde(msb);
    AOLED_WriteCmde(lsb);
}


void    AOLED_InvertDisplay(int invert)
//-------------------------------------------------------------
//-------------------------------------------------------------
{
    if (invert)
        AOLED_WriteCmde(0xA7);      // INVERT DISPLAY
    else
        AOLED_WriteCmde(0xA6);      // NORMAL DISPLAY
}

void    AOLED_FillScreen(char value)
//-------------------------------------------------------------
//-------------------------------------------------------------
{    short numcol, numlin, idln;

    for (numlin = 0; numlin < 8; numlin++) {
        AOLED_SetAdrsLin(numlin);
        AOLED_SetAdrsCol(0);
        for (numcol = 0; numcol < 128; numcol++) {
            AOLED_WriteData(value);
        }
    }
}

void    AOLED_DisplayImage(char *pBuff)
//-------------------------------------------------------------
//-------------------------------------------------------------
{    short numcol, numlin, idln;

    for (numlin = 0; numlin < 8; numlin++) {
        AOLED_SetAdrsLin(numlin);
        AOLED_SetAdrsCol(0);
        idln = numlin * 128;
        for (numcol = 127; numcol >= 0; numcol--) {
            AOLED_WriteData(pBuff[idln + numcol]);
        }
    }
}

void    AOLED_DisplayCarac(int x, int y, char car)
//-------------------------------------------------------------
//-------------------------------------------------------------
{   short numcol, debcol;      char valcol;

    x = 128 - x;
    AOLED_SetAdrsLin(y);
    AOLED_SetAdrsCol(x - 5);
    debcol = (car - 32) * 5;
    for(numcol = 4; numcol >= 0; numcol--) {
        valcol = Isep_Font[debcol + numcol];
        AOLED_WriteData(valcol);
    }
}

void    AOLED_DisplayTexte(int x, int y, char *texte)
//-------------------------------------------------------------
//-------------------------------------------------------------
{
    while (*texte) {
        AOLED_DisplayCarac(x, y, *texte);
        x += 6;     texte++;
    }
}

const char Ecole[] = "  Ecole d'Ingenieurs";
const char Numrq[] = "     du Numerique";

void    AOLED_AffiLogoIsep(void)
//-------------------------------------------------------------
//-------------------------------------------------------------
{
    AOLED_DisplayImage((char*)Isep_Logo);
    AOLED_DisplayTexte(1, 6, (char*)Ecole);
    AOLED_DisplayTexte(1, 7, (char*)Numrq);
}

void    AOLED_InitScreen(void)
//-------------------------------------------------------------
//-------------------------------------------------------------
{
  Serial.println("    +++ Debut AOLED_InitScreen");
    TIVA_I2C1_InitModule();
  Serial.println("    +++ Fin 1 AOLED_InitScreen");
    delay(1000);
    AOLED_WriteCmde(0xAE);     // Set display OFF
  Serial.println("    ---    WriteCmde display OFF");

    AOLED_WriteCmde(0xD4);     // Set Display Clock Divide Ratio / OSC Frequency
    AOLED_WriteCmde(0x80);     // Display Clock Divide Ratio / OSC Frequency

    AOLED_WriteCmde(0xA8);     // Set Multiplex Ratio
    AOLED_WriteCmde(0x3F);     // Multiplex Ratio for 128x64 (64-1)

    AOLED_WriteCmde(0xD3);     // Set Display Offset
    AOLED_WriteCmde(0x00);     // Display Offset

    AOLED_WriteCmde(0x40);     // Set Display Start Line 0

    AOLED_WriteCmde(0x8D);     // Set Charge Pump
    AOLED_WriteCmde(0x14);     // Charge Pump (0x10 External, 0x14 Internal DC/DC)

    AOLED_WriteCmde(0xA0);     // Set Segment Re-Map
    AOLED_WriteCmde(0xC8);     // Set Com Output Scan Direction

    AOLED_WriteCmde(0xDA);     // Set COM Hardware Configuration
    AOLED_WriteCmde(0x12);     // COM Hardware Configuration

    AOLED_WriteCmde(0x81);     // Set Contrast
    AOLED_WriteCmde(0xCF);     // Contrast

    AOLED_WriteCmde(0xD9);     // Set Pre-Charge Period
    AOLED_WriteCmde(0xF1);     // Set Pre-Charge Period (0x22 External, 0xF1 Internal)

    AOLED_WriteCmde(0xDB);     // Set VCOMH Deselect Level
    AOLED_WriteCmde(0x40);     // VCOMH Deselect Level

    AOLED_WriteCmde(0xB0);

    AOLED_WriteCmde(0xA4);     // Set all pixels OFF
    AOLED_WriteCmde(0xA6);     // Set display not inverted
    AOLED_WriteCmde(0xAF);     // Set display On
    
  Serial.println("    +++ Fin 2 AOLED_InitScreen");
}


#define SYSCTL_RCGCGPIO			(*((volatile unsigned int *)0x400FE608))
#define SYSCTL_RCGCI2C			(*((volatile unsigned int *)0x400FE620))
#define SYSCTL_SRI2C			(*((volatile unsigned int *)0x400FE520))

#define GPIO_PORTA_AFSEL		(*((volatile unsigned int *)0x40004420))
#define GPIO_PORTA_ODR			(*((volatile unsigned int *)0x4000450C))
#define GPIO_PORTA_PUR          (*((volatile unsigned int *)0x40004510))
#define GPIO_PORTA_DEN			(*((volatile unsigned int *)0x4000451C))
#define GPIO_PORTA_PCTL			(*((volatile unsigned int *)0x4000452C))

#define GPIO_PORTE_AFSEL		(*((volatile unsigned int *)0x40024420))
#define GPIO_PORTE_ODR			(*((volatile unsigned int *)0x4002450C))
#define GPIO_PORTE_DEN			(*((volatile unsigned int *)0x4002451C))
#define GPIO_PORTE_PCTL			(*((volatile unsigned int *)0x4002452C))


#define I2C1_BASE       0x40021000
#define I2C2_BASE       0x40022000

#define I2Cx_MSA        0x0000
#define I2Cx_MCS        0x0004
#define I2Cx_MDR        0x0008
#define I2Cx_MTPR       0x000C
#define I2Cx_MIMR       0x0010
#define I2Cx_MCR        0x0020
#define I2Cx_MBMON      0x002C
#define I2Cx_MCR2       0x0038
#define I2Cx_PC         0x0FC4
#define I2C_FIFOCTL     0x0F04

#define MCS_RUN         0x01
#define MCS_START       0x02
#define MCS_STOP        0x04
#define MCS_ACK         0x08
#define MCS_BUSY        0x01
#define MCS_ERROR       0x02


volatile unsigned int *pAdrModule;


static short    I2Cx_SendData(int modBase, char slaveAdr, short nbdata, char *pBuff)
//--------------------------------------------------------------------
//--------------------------------------------------------------------
{    unsigned int value, cmde;    short nerr;

  Serial.println("    ---    1 I2Cx_SendData");
    nerr = -2;
    if (nbdata < 0 || nbdata > 127)
        goto sortie;
  Serial.println("    ---    2 I2Cx_SendData");
    pAdrModule = (unsigned int*)modBase;
    *(pAdrModule + I2Cx_MSA) = slaveAdr & 0x00FE;
    nerr = -3;
    value = *pBuff++;   nbdata--;
    *(pAdrModule + I2Cx_MDR) = value;
    cmde = MCS_RUN | MCS_START;
  Serial.println("    ---    3 I2Cx_SendData");
    while (1) {
        if (nbdata == 0)
            cmde |= MCS_STOP;
    Serial.print("    --- cmde = 0x");  Serial.println(cmde, HEX);
        *(pAdrModule + I2Cx_MCS) = cmde;
        do {
            value = *(pAdrModule + I2Cx_MCS);
    Serial.print("    --- value = 0x");  Serial.println(value, HEX);
        } while ((value & MCS_BUSY) != 0);
  Serial.println("    ---    4 I2Cx_SendData");
        if ((value & MCS_ERROR) != 0) {
            if (nbdata != 0) {
                *(pAdrModule + I2Cx_MCS) = MCS_STOP;
            }
            goto sortie;
        }
        if (nbdata == 0)
            break;
        value = *pBuff++;   nbdata--;
        *(pAdrModule + I2Cx_MDR) = value;
        cmde = MCS_RUN;
  Serial.println("    ---    5 I2Cx_SendData");
    }
    nerr = 0;
sortie:
  Serial.println("    ---    6 I2Cx_SendData");
    return nerr;
}

static void    I2C1_IniPorts(void)
//--------------------------------------------------------------------
// I2C-1  => PA6 = I2C1SCL, PA7 = I2C1SDA
//--------------------------------------------------------------------
{
    int value;

    // Clock setting for I2C-1  (
    //--------------------------------------------------------
    SYSCTL_RCGCGPIO |= 0x01;        // Enable system clock to PORTA
    SYSCTL_RCGCI2C  |= 0x02;        // Enable clock to I2C-1 module
    SYSCTL_SRI2C    |= 0x02;        // start Reset
    value  = 0;                     // dummy
    value += 3;
    SYSCTL_SRI2C     = 0x00;        // end Reset

    // Setting of PA6 = I2C1SCL, PA7 = I2C1SDA
    //--------------------------------------------------------
    GPIO_PORTA_AFSEL |= 0xC0;       // PA7,PA6 sets an alternate function
    GPIO_PORTA_ODR   |= 0x80;       // PA7-SDA sets as open drain pin
    GPIO_PORTA_PUR   &= ~(0x0C0);
//    GPIO_PORTA_PUR   |= 0x0C0;
    value = GPIO_PORTA_PCTL;
    value = (value & 0x00FFFFFF) | 0x33000000;
    GPIO_PORTA_PCTL   = value;      // make PA7,PA6 as I2C-SDA/SCL
    GPIO_PORTA_DEN   |= 0xC0;       // Enable PA7,PA6 as a digital pin
}

static void    I2Cx_InitModule(short nmodule)
//--------------------------------------------------------------------
//--------------------------------------------------------------------
{
  Serial.println("       +++ Debut I2C1_IniPorts");
    if (nmodule == 1) {
        I2C1_IniPorts();
        pAdrModule = (unsigned int*)I2C1_BASE;
    }
    else
        goto sortie;
  Serial.println("       +++ Fin I2C1_IniPorts");

    delay(4);      // Obligatoire
  Serial.println("       +++ Debut I2Cx_InitModule");

    *(pAdrModule + I2Cx_PC)     = 0x00;     // Disable High-speed mode
//    *(pAdrModule + I2Cx_MTPR)   = 0x09;     // Fast mode 400 Khz
    *(pAdrModule + I2Cx_MTPR)   = 0x27;     // Normal mode 100 Khz
    *(pAdrModule + I2Cx_MIMR)   = 0x00;     // No interrupt
    *(pAdrModule + I2Cx_MCR2)   = 0x60;     // Filter glitch 6 sys-clk
    *(pAdrModule + I2Cx_MCR)    = 0x50;     // Master mode only

    //clear I2C FIFOs
    *(pAdrModule + I2C_FIFOCTL) = 80008000;
sortie:
  Serial.println("       +++ Fin I2Cx_InitModule");
    return;
}

void    TIVA_I2C1_InitModule(void)
//--------------------------------------------------------------------
//--------------------------------------------------------------------
{
  Serial.println("    +++ Debut TIVA_I2C1_InitModule");
    I2Cx_InitModule(1);
  Serial.println("    +++ Fin   TIVA_I2C1_InitModule");
}

short   TIVA_I2C1_SendData(char slaveAdr, short nbdata, char* pBuff)
//--------------------------------------------------------------------
//--------------------------------------------------------------------
{    short nerr;

    nerr = I2Cx_SendData(I2C1_BASE, slaveAdr, nbdata, pBuff);
    return nerr;
}
