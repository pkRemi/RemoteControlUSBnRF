#ifndef MAIN_C
#define MAIN_C

#include "USB/usb.h"
#include "HardwareProfile.h"
#include "USB/usb_function_hid.h"
#include <delays.h>
#include <string.h>
/** CONFIGURATION **************************************************/
#pragma config PLLDIV   = 5         // (20 MHz crystal on PICDEM FS USB board)
#pragma config CPUDIV   = OSC1_PLL2
#pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
#pragma config FOSC     = HSPLL_HS
#pragma config FCMEN    = OFF
#pragma config IESO     = OFF
#pragma config PWRT     = OFF
#pragma config BOR      = ON
#pragma config BORV     = 3
#pragma config VREGEN   = ON      //USB Voltage Regulator
#pragma config WDT      = OFF
#pragma config WDTPS    = 32768
#pragma config MCLRE    = ON
#pragma config LPT1OSC  = OFF
#pragma config PBADEN   = OFF
//      #pragma config CCP2MX   = ON
#pragma config STVREN   = ON
#pragma config LVP      = OFF
//      #pragma config ICPRT    = OFF       // Dedicated In-Circuit Debug/Programming
#pragma config XINST    = OFF       // Extended Instruction Set
#pragma config CP0      = OFF
#pragma config CP1      = OFF
//      #pragma config CP2      = OFF
//      #pragma config CP3      = OFF
#pragma config CPB      = OFF
//      #pragma config CPD      = OFF
#pragma config WRT0     = OFF
#pragma config WRT1     = OFF
//      #pragma config WRT2     = OFF
//      #pragma config WRT3     = OFF
#pragma config WRTB     = OFF       // Boot Block Write Protection
#pragma config WRTC     = OFF
//      #pragma config WRTD     = OFF
#pragma config EBTR0    = OFF
#pragma config EBTR1    = OFF
//      #pragma config EBTR2    = OFF
//      #pragma config EBTR3    = OFF
#pragma config EBTRB    = OFF
/** VARIABLES ******************************************************/
#pragma udata

//The ReceivedDataBuffer[] and ToSendDataBuffer[] arrays are used as
//USB packet buffers in this firmware.  Therefore, they must be located in
//a USB module accessible portion of microcontroller RAM.
#pragma udata USB_VARIABLES=0x500

unsigned char ReceivedDataBuffer[64];// RX_DATA_BUFFER_ADDRESS;
unsigned char ToSendDataBuffer[64];// TX_DATA_BUFFER_ADDRESS;

#pragma udata


USB_HANDLE USBOutHandle = 0; //USB handle.  Must be initialized to 0 at startup.
USB_HANDLE USBInHandle = 0; //USB handle.  Must be initialized to 0 at startup.
BOOL blinkStatusValid = TRUE;
char telemetry[32] = {0};   // Telemetry data
char newTelemetry = 0;      // Telemetry status byte
int nRFpolldelay = 400000000;       // Delay counter for nRF
int LCDcount = 0;                  // Temporary counter for LCD testing
char commStatus = 0;                // Communication status for RF
char commMessage = 0;
/** PRIVATE PROTOTYPES *********************************************/
void BlinkUSBStatus(void);
BOOL Switch2IsPressed(void);
BOOL Switch3IsPressed(void);
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
WORD_VAL ReadPOT(int channel);

void mInitSPI(void);
void mInitnRF(void);
char readnRFstatus(void);
void sendSPI1string(char *spistring, int size);
char readnRFbyte(char command);
char readnRFsingle(void);
void readnRFstring(char *strptr, char length);
void ByteWriteSPI(unsigned char OpCode, unsigned char Data );
void sendnRFstring(unsigned char *nRFstring, int size);
unsigned char LDPageWriteSPI( unsigned char OpCode, unsigned char *wrptr, unsigned char strlength );
void WriteSPI1(char data_out);
void sendLCDsingle(char data);
void sendLCDnibble(char data, BOOL RS);
void mInitLCD(void);
void sendLCDchar(char data, BOOL RS);
void sendLCDstring(char *string);
void sendLCDstringROM(rom char *string);
void sendLCDcls(void);
void sendLCDclearline2(void);


/** VECTOR REMAPPING ***********************************************/

//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
//the reset, high priority interrupt, and low priority interrupt
//vectors.  However, the current Microchip USB bootloader
//examples are intended to occupy addresses 0x00-0x7FF or
//0x00-0xFFF depending on which bootloader is used.  Therefore,
//the bootloader code remaps these vectors to new locations
//as indicated below.  This remapping is only necessary if you
//wish to program the hex file generated from this project with
//the USB bootloader.  If no bootloader is used, edit the
//usb_config.h file and comment out the following defines:
//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
#else
#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
#endif

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
extern void _startup(void); // See c018i.c in your C18 compiler dir
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS

void _reset(void) {
    _asm goto _startup _endasm
}
#endif
#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS

void Remapped_High_ISR(void) {
    _asm goto YourHighPriorityISRCode _endasm
}
#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS

void Remapped_Low_ISR(void) {
    _asm goto YourLowPriorityISRCode _endasm
}

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
//Note: If this project is built while one of the bootloaders has
//been defined, but then the output hex file is not programmed with
//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
//As a result, if an actual interrupt was enabled and occured, the PC would jump
//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
//would effective reset the application.

//To fix this situation, we should always deliberately place a
//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
//hex file of this project is programmed with the bootloader, these sections do not
//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
//programmed using the bootloader, then the below goto instructions do get programmed,
//and the hex file still works like normal.  The below section is only required to fix this
//scenario.
#pragma code HIGH_INTERRUPT_VECTOR = 0x08

void High_ISR(void) {
    _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
}
#pragma code LOW_INTERRUPT_VECTOR = 0x18

void Low_ISR(void) {
    _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
}
#endif	//end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

#pragma code


//These are your actual interrupt handling routines.
#pragma interrupt YourHighPriorityISRCode

void YourHighPriorityISRCode() {
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.
#if defined(USB_INTERRUPT)
    USBDeviceTasks();
#endif

} //This return will be a "retfie fast", since this is in a #pragma interrupt section
#pragma interruptlow YourLowPriorityISRCode

void YourLowPriorityISRCode() {
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.

} //This return will be a "retfie", since this is in a #pragma interruptlow section


/** DECLARATIONS ***************************************************/

#pragma code

/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/

void main(void) {
    InitializeSystem();

#if defined(USB_INTERRUPT)
    USBDeviceAttach();
#endif

    while (1) {
#if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        // this function periodically.  This function will take care
        // of processing and responding to SETUP transactions
        // (such as during the enumeration process when you first
        // plug in).  USB hosts require that USB devices should accept
        // and process SETUP packets in a timely fashion.  Therefore,
        // when using polling, this function should be called
        // regularly (such as once every 1.8ms or faster** [see
        // inline code comments in usb_device.c for explanation when
        // "or faster" applies])  In most cases, the USBDeviceTasks()
        // function does not take very long to execute (ex: <100
        // instruction cycles) before it returns.
#endif


        // Application-specific tasks.
        // Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();
    }//end while
}//end main

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void) {
    ADCON1 |= 0x0F; // Default all pins to digital




    //	The USB specifications require that USB peripheral devices must never source
    //	current onto the Vbus pin.  Additionally, USB peripherals should not source
    //	current on D+ or D- when the host/hub is not actively powering the Vbus line.
    //	When designing a self powered (as opposed to bus powered) USB peripheral
    //	device, the firmware should make sure not to turn on the USB module and D+
    //	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
    //	firmware needs some means to detect when Vbus is being powered by the host.
    //	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
    // 	can be used to detect when Vbus is high (host actively powering), or low
    //	(host is shut down or otherwise not supplying power).  The USB firmware
    // 	can then periodically poll this I/O pin to know when it is okay to turn on
    //	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
    //	peripheral device, it is not possible to source current on D+ or D- when the
    //	host is not actively providing power on Vbus. Therefore, implementing this
    //	bus sense feature is optional.  This firmware can be made to use this bus
    //	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
    //	HardwareProfile.h file.
#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

    //	If the host PC sends a GetStatus (device) request, the firmware must respond
    //	and let the host know if the USB peripheral device is currently bus powered
    //	or self powered.  See chapter 9 in the official USB specifications for details
    //	regarding this request.  If the peripheral device is capable of being both
    //	self and bus powered, it should not return a hard coded value for this request.
    //	Instead, firmware should check if it is currently self or bus powered, and
    //	respond accordingly.  If the hardware has been configured like demonstrated
    //	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
    //	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2"
    //	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
    //	has been defined in HardwareProfile - (platform).h, and that an appropriate I/O pin
    //  has been mapped	to it.
#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif

    UserInit();

    USBDeviceInit(); //usb_device.c.  Initializes USB module SFRs and firmware
    //variables to known states.
}//end InitializeSystem

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:
 *
 *****************************************************************************/
void UserInit(void) {
    //Initialize all of the LED pins
    mInitAllLEDs();

    //Initialize all of the push buttons
    //mInitAllSwitches();
    TRISBbits.TRISB4=1; // nRF interrupt pin
    TRISBbits.TRISB5=1;
    //Initialize I/O pin and ADC settings to collect potentiometer measurements
    mInitPOT();

    //Initialize SPI

    mInitSPI();

    //Init nFR
    mInitnRF();
    //initialize the variable holding the handle for the last
    // transmission
    USBOutHandle = 0;
    USBInHandle = 0;

    blinkStatusValid = TRUE;
    // Init LCD
    mInitLCD();
}//end UserInit

void mInitSPI(void)
{
    SSPSTATbits.SMP = 0;  //0 = Input data sampled at middle of data output time
    SSPSTATbits.CKE = 1; //0 = Transmit occurs on transition from Idle to active clock state
    SSPCON1bits.CKP = 0;    //0 = Idle state for clock is a low level
    SSPCON1bits.SSPM = 0b0010; //0001 = SPI Master mode, clock = FOSC/16
    TRISCbits.TRISC7 = 0; // for SDO
    TRISBbits.TRISB1 = 0; // for SCK
    TRISBbits.TRISB2 = 0; // for /CS RB2
    TRISBbits.TRISB3 = 0; // for CE RB3
    PORTBbits.RB3 = 0;
    CE_nRF = 0;             // Set CE low
    CS_nRF = 1;             // Set /CS high
    SSPCON1bits.SSPEN = 1; //1 = Enables serial port and configures SCK, SDO, SDI and SS as serial port pins


}

void mInitnRF(void)
{
    int sl;
    char spistring[6] = {0x20, 0b00001001, 1, 2, 3, 4};
    sl = 2;

    sendSPI1string(spistring, sl);
    spistring[0] = 0x2a; // Set RX_ADDR_P0 address
    spistring[1] = 0xa5;
    spistring[2] = 0xd6;
    spistring[3] = 0x65;
    spistring[4] = 0xcb;
    spistring[5] = 0x2a;
    sl = 6;
    sendSPI1string(spistring, sl);

    spistring[0] = 0x30; // Set TX_ADDR address
    sendSPI1string(spistring, sl);


    spistring[0] = 0x31; // Payload width
    spistring[1] = 22;   //Width
    sl = 2;
    sendSPI1string(spistring, sl);
    ByteWriteSPI(0x3D, 0b00000110);  //Enables dynamic payload length and Payload with ACK
    ByteWriteSPI(0x3C, 0b00000001);  //Enables dynamic payload length on data pipe 0
    ByteWriteSPI(0x21, 0b00111111);  //Enables Auto Ack


    ByteWriteSPI(0xE2 , 0xFF); //Flush RX buffer
    ByteWriteSPI(0xE1 , 0xFF); //Flush TX buffer
    ByteWriteSPI(0x27 , 0b01110000); // Clear all flags
    ByteWriteSPI(0x20 , 0b00001011); // Powerup
    CE_nRF = 1;     // Set CE high to start receiving data
    Delay10KTCYx(24); // Powerup delay 5ms * 48MHz = 240000
}
void readnRFstring(char *strptr, char length)
{
    CS_nRF = 0;
    SSPBUF = 0x61;
    while(!PIR1bits.SSPIF); // Wait until transmit has finished
    Delay10TCYx(20);        // Delay needed from Errata
    PIR1bits.SSPIF = 0;    // Reset interrupt flag

    while(length)
    {
        length--;
        SSPBUF = 0xFF;
        while(!PIR1bits.SSPIF); // Wait until transmit has finished
        Delay10TCYx(20);        // Delay needed from Errata
        PIR1bits.SSPIF = 0;    // Reset interrupt flag
        *strptr++ = SSPBUF;    // Writes buffer to string then increments the pointer
    }

    CS_nRF = 1;
}
char readnRFsingle(void)
{
    char rdata;
    CS_nRF = 0;
    SSPBUF = 0x61;
    while(!PIR1bits.SSPIF); // Wait until transmit has finished
    Delay10TCYx(20);        // Delay needed from Errata
    PIR1bits.SSPIF = 0;    // Reset interrupt flag
    SSPBUF = 0xFF;
    while(!PIR1bits.SSPIF); // Wait until transmit has finished
    Delay10TCYx(20);        // Delay needed from Errata
    PIR1bits.SSPIF = 0;    // Reset interrupt flag
    rdata = SSPBUF;
    CS_nRF = 1;
    return rdata;
}
char readnRFbyte(char command)
{
    char rdata;
    CS_nRF = 0;
    SSPBUF = command;
    while(!PIR1bits.SSPIF); // Wait until transmit has finished
    Delay10TCYx(20);        // Delay needed from Errata
    PIR1bits.SSPIF = 0;    // Reset interrupt flag
    SSPBUF = 0xFF;
    while(!PIR1bits.SSPIF); // Wait until transmit has finished
    Delay10TCYx(20);        // Delay needed from Errata
    PIR1bits.SSPIF = 0;    // Reset interrupt flag
    rdata = SSPBUF;
    CS_nRF = 1;
    return rdata;
}
//char readnRFstatus(void)
//{
//    char rdata;
//    CS_nRF = 0;
//    SSPBUF = 0xFF;          // Send dummy string
//    while(!PIR1bits.SSPIF); // Wait until transmit has finished
//    PIR1bits.SSPIF = 0;    // Reset interrupt flag
//    rdata = SSPBUF;
//    CS_nRF = 1;
//    return rdata;
//}
char readnRFstatus(void)
{
    char rdata;
    CS_nRF = 0;
//    SSPBUF = 0xFF;          // Send dummy string
//    while(!PIR1bits.SSPIF); // Wait until transmit has finished
//    PIR1bits.SSPIF = 0;    // Reset interrupt flag
    WriteSPI1 ( 0x07 );         // Send Write OpCode
    WriteSPI1 ( 0xFF );           // Write Byte to device
    rdata = SSPBUF;
    CS_nRF = 1;
    return rdata;
}
void sendSPI1string(char *spistring, int size)
{
    int n=0;
    CS_nRF = 0;                // Set /CS to low
    for(n=0; n<size;n++)
    {
         SSPBUF = spistring[n];        //Transmit the data from the string
         while(!PIR1bits.SSPIF); // Wait until transmit has finished
         Delay10TCYx(20);        // Delay needed from Errata
         PIR1bits.SSPIF = 0;    // Reset interrupt flag
    }
    CS_nRF = 1;        // Set /CS to high
}
void sendLCDsingle(char data)
{
    LATDbits.LATD3 = 0;    // Set strobe to low (should be redundant)
    SSPBUF = data;         //Transmit the data from the string
    while(!PIR1bits.SSPIF);// Wait until transmit has finished
    Delay10TCYx(20);       // Delay needed from Errata
    PIR1bits.SSPIF = 0;    // Reset interrupt flag
    LATDbits.LATD3 = 1;    // Set strobe to high
    Delay10TCYx(2);       // Wait for minimum 200ns
    LATDbits.LATD3 = 0;    // Set strobe to low
}
void sendLCDnibble(char data, BOOL RS)
{
    char datao = 0;
    datao = (data << 4);
    if (RS)
        datao = datao | 0b00000010;

    sendLCDsingle(datao);           // Select data or instruction register
    datao = datao | 0b00000001;
    sendLCDsingle(datao);           // Set Enable
    datao = datao & 0b11111110;
    sendLCDsingle(datao);           // Reset Enable (strobe)

}
void mInitLCD(void)
{
    char ssworld[20];
    sendLCDsingle(0x00);
    Delay10KTCYx(72);              // Delay 15ms
    sendLCDnibble(0b0011, 0);
    Delay10KTCYx(5);              // Delay 4.1ms
    sendLCDnibble(0b0011, 0);
    Delay100TCYx(8);              // Delay 100us
    sendLCDnibble(0b0011, 0);
    Delay10KTCYx(5);              // Delay 4.1ms
    sendLCDnibble(0b0010, 0);
    sendLCDchar(0x28, 0); //4bit 2line 5x8
    sendLCDchar(0b00001100, 0); //Blinking cursor
    sendLCDchar(0x04, 0); // cursor dir
    //sendLCDchar(0x85, 0); // no idea
    //sendLCDchar(0x06, 0);
    sendLCDcls();
    //sendLCDchar(0x80, 0);


//    sendLCDnibble(0b0010, 0);
//    // Now the LCD is in 4 bit mode
//    sendLCDnibble(0b1000, 0);       // 2 line, 5x8 dots
//    sendLCDnibble(0b1000, 0);       // Display, cursor and blinking off
//    sendLCDnibble(0b0001, 0);       // Clear screen, return home
//    sendLCDnibble(0b0110, 0);       // Inc cursor to the right when writing and don’t shift screen
//    sendLCDnibble(0b1111, 0);       // Display, cursor and blinking on
    sendLCDstringROM("BalancingBallBot");
    sendLCDchar(0xC0, 0);
 //char *strcpy (auto char *s1, auto const char *s2);
    //*strcpy(ssworld, "Is Cool!");
    strcpypgm2ram( ssworld, "Is Cool!!");
    sendLCDstring(ssworld);
    //sendLCDstringROM("BEER -> YES");
}
void sendLCDclearline2(void)
{
    sendLCDchar(0xC0, 0);
    sendLCDstringROM("                ");
    sendLCDchar(0xC0, 0);
    
}
void sendLCDcls(void)
{
    sendLCDchar(0x01, 0);         // Clear screen
    Delay10KTCYx(3);              // Delay 4.1ms
}
void sendLCDchar(char data, BOOL RS)
{
    char datao = 0;
    datao = data >> 4;
    sendLCDnibble(datao, RS); // Send ms nibble
    datao = data;
    sendLCDnibble(datao, RS); // Send ls nibble
}
void sendLCDstringROM(rom char *string)
{
  char n = 0;                    //
  char temp;                     //
  while(temp = string[n++])       //
    sendLCDchar(temp, 1);            //
}
void sendLCDstring(char *string)
{
  char n = 0;                    //
  char temp;                     //
  while(temp = string[n++])       //
    sendLCDchar(temp, 1);            //

}
/********************************************************************
*     Function Name:    ByteWriteSPI                              *
*     Parameters:       OpCode/register, data.                      *
*     Description:      Writes Data Byte to SPI device              *
*                                                                   *
********************************************************************/
void ByteWriteSPI(unsigned char OpCode, unsigned char Data )
{
  CS_nRF = 0;                   // Select Device
  WriteSPI1 ( OpCode );         // Send Write OpCode
  WriteSPI1 ( Data );           // Write Byte to device
  CS_nRF = 1;                   // Deselect device
  //SPI1STATbits.SPITBF = 0;      //Clear Transmit Buffer Full Status bit
}
/********************************************************************
*     Function Name : WriteSPI1                                     *
*     Description   : This routine writes a single byte/word to     *
*                     the SPI bus.                                  *
*     Parameters    : Single data byte/word for SPI bus             *
*     Return Value  : None                                          *
********************************************************************/

void WriteSPI1(char data_out)
{
    SSPBUF = data_out;                /*  byte write  */
    while(!PIR1bits.SSPIF); // Wait until transmit has finished
    Delay10TCYx(20);        // Delay needed from Errata
    PIR1bits.SSPIF = 0;    // Reset interrupt flag
    //data_out = SPI1BUF;               //Avoiding overflow when reading
}

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessIO(void) {
// Read telemetry
    int nn;
    char nRFstatus = 0;
    unsigned char serString[64];
    unsigned char nRFregisters[10];
    int iii;
    unsigned char debugTemp;
    while(1)
    {
        if((commStatus == 0) && commMessage != 2)//(commMessage != 2)
        {
            sendLCDclearline2();
            sendLCDstringROM("RF comm : FAIL");
            commMessage = 2;
        }
        if((commStatus == 1) && (commMessage != 3))
        {
            sendLCDclearline2();
            sendLCDstringROM("RF comm : OK");
            commMessage = 3;
        }
        
        for(iii=0; iii<10;iii++)
        {
            nRFregisters[iii] = readnRFbyte(iii);
        }
        if (!PORTBbits.RB4) // Check nRF Interrupt pin
        {

            nRFstatus = readnRFstatus();
            if (nRFstatus & 0b01000000)
            {
                Delay10TCYx(100);
                //LATDbits.LATD3 = 1;
                //LATDbits.LATD2 = !LATDbits.LATD2;
                readnRFstring(telemetry, 22);
                ByteWriteSPI(0x27 , 0b01000000);
                //ByteWriteSPI(0xE2 , 0xFF); //Flush SPI
                nRFpolldelay = 400000000;
                if(telemetry[0]!=0x42)
                {
                    LATDbits.LATD2 = 1;
                    readnRFstatus();
                    ByteWriteSPI(0x00, 0xFF);
                    mInitnRF(); //Reinitialize nRF if there is an error...
                    newTelemetry = 0;
                    commStatus = 0;
                }
                else
                {
                    LATDbits.LATD2 = 0;
                    newTelemetry = 1;
                }
                //LATDbits.LATD3 = 0;

            }

        }
        if(telemetry[21]==0x44)
        {
            sendLCDclearline2();
            sendLCDstringROM("0x44 received");
        }

        LCDcount++;
        if(newTelemetry)
        {
            LCDcount = 0;
            commStatus = 1;
        }
        if(LCDcount >= 5000)
        {
            commStatus = 0;
            LCDcount = 0;
        }


        //Blink the LEDs according to the USB device status
        if (blinkStatusValid) {
            BlinkUSBStatus();
        }

        // User Application USB tasks
        if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

        //Check if we have received an OUT data packet from the host
        if (!HIDRxHandleBusy(USBOutHandle)) {
            //We just received a packet of data from the USB host.
            //Check the first byte of the packet to see what command the host
            //application software wants us to fulfill.
            switch (ReceivedDataBuffer[0]) //Look at the data the host sent, to see what kind of application specific command it sent.
            {
                case 0x80: //Toggle LEDs command
                    blinkStatusValid = FALSE; //Stop blinking the LEDs automatically, going to manually control them now.
                    sendLCDclearline2();
                    sendLCDstringROM("LED Manual");
                    if (mGetLED_1() == mGetLED_2()) {
                        mLED_1_Toggle();
                        mLED_2_Toggle();
                    } else {
                        if (mGetLED_1()) {
                            mLED_2_On();
                        } else {
                            mLED_2_Off();
                        }
                    }
                    break;
                case 0x81: //Get push button state
                    //Check to make sure the endpoint/buffer is free before we modify the contents
                    if (!HIDTxHandleBusy(USBInHandle)) {
                        ToSendDataBuffer[0] = 0x81; //Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
                        if (sw3 == 1) //pushbutton not pressed, pull up resistor on circuit board is pulling the PORT pin high
                        {
                            ToSendDataBuffer[1] = 0x01;
                        } else //sw3 must be == 0, pushbutton is pressed and overpowering the pull up resistor
                        {
                            ToSendDataBuffer[1] = 0x00;
                        }
                        //Prepare the USB module to send the data packet to the host
                        USBInHandle = HIDTxPacket(HID_EP, (BYTE*) & ToSendDataBuffer[0], 64);
                    }
                    break;
                case 0x82: //Motor enable toggle command
                    sendLCDclearline2();
                    sendLCDstringROM("Toggle Motors");
                    serString[0] = 0x82;
                    ByteWriteSPI(0xE1 , 0xFF); //Flush TX buffer
                    LDPageWriteSPI(0xA8, serString, 32);
                    break;

                case 0x37: //Read POT command.  Uses ADC to measure an analog voltage on one of the ANxx I/O pins, and returns the result to the host
                {
                    WORD_VAL w;
                    //Check to make sure the endpoint/buffer is free before we modify the contents
                    if (!HIDTxHandleBusy(USBInHandle)) {
                        w = ReadPOT(0); //Use ADC to read the I/O pin voltage.  See the relevant HardwareProfile - xxxxx.h file for the I/O pin that it will measure.
                        //Some demo boards, like the PIC18F87J50 FS USB Plug-In Module board, do not have a potentiometer (when used stand alone).
                        //This function call will still measure the analog voltage on the I/O pin however.  To make the demo more interesting, it
                        //is suggested that an external adjustable analog voltage should be applied to this pin.
                        ToSendDataBuffer[0] = 0x37; //Echo back to the host the command we are fulfilling in the first byte.  In this case, the Read POT (analog voltage) command.
                        ToSendDataBuffer[1] = w.v[0]; //Measured analog voltage LSB
                        ToSendDataBuffer[2] = w.v[1]; //Measured analog voltage MSB
                        w = ReadPOT(1); //Use ADC to read the I/O pin voltage.  See the relevant HardwareProfile - xxxxx.h file for the I/O pin that it will measure.
                        ToSendDataBuffer[3] = w.v[0]; //Measured analog voltage LSB
                        ToSendDataBuffer[4] = w.v[1]; //Measured analog voltage MSB

                        //Prepare the USB module to send the data packet to the host
                        USBInHandle = HIDTxPacket(HID_EP, (BYTE*) & ToSendDataBuffer[0], 64);
                    }
                }
                    break;

                case 0x42: //Read telemetry command.  Takes the received telemetry and sends it via USB
                {
                    //WORD_VAL w;
                    //Check to make sure the endpoint/buffer is free before we modify the contents
                    if (!HIDTxHandleBusy(USBInHandle)) {

                        if(newTelemetry)
                        {
                            ToSendDataBuffer[0] = 0x42; //Echo back to the host the command we are fulfilling in the first byte.
                            for (nn = 1; nn <22 ; nn++)
                            {
                                ToSendDataBuffer[nn] = telemetry[nn];
                            }
                        newTelemetry = 0;
                        debugTemp = ToSendDataBuffer[21];
                        }
                        else
                        {
                            ToSendDataBuffer[0] = 0x41; //No new telemetry

                        }
                        //Prepare the USB module to send the data packet to the host
                        USBInHandle = HIDTxPacket(HID_EP, (BYTE*) & ToSendDataBuffer[0], 64);
                    }
                }
                    break;
            }
            //Re-arm the OUT endpoint, so we can receive the next OUT data packet
            //that the host may try to send us.
            USBOutHandle = HIDRxPacket(HID_EP, (BYTE*) & ReceivedDataBuffer, 64);
        }
    }

}//end ProcessIO
/********************************************************************
*     Function Name:    sendnRFstring                               *
*     Parameters:       pointer to string, length of string.        *
*     Description:      Writes a string to the transmit FIFO of nRF *
*                       and toggels the CE line to start            *
*                       transmission.                               *
*                                                                   *
********************************************************************/
void sendnRFstring(unsigned char *nRFstring, int size)
{
    LDPageWriteSPI(0xA0, nRFstring, size);

    Delay10TCYx(50);  // Delay more than 10us * 48MHz = 480 TCY
    CE_nRF = 1;
    Delay10TCYx(50);
    CE_nRF = 0;
}
/********************************************************************
*     Function Name:    LDPageWriteSPI                              *
*     Parameters:       Opcode, pointer addr, string length         *
*     Description:      Writes data string to SPI device            *
*                       OpCode is the register that receives the    *
*                       data                                        *
*                                                                   *
********************************************************************/
unsigned char LDPageWriteSPI( unsigned char OpCode, unsigned char *wrptr, unsigned char strlength )
{
  CS_nRF = 0;                           // Select Device
  WriteSPI1 ( OpCode );                 // send OpCode
  sendSPI1string ( wrptr, strlength );    // Write Page to device
  CS_nRF = 1;                           // Deselect Device
  //SPI1STATbits.SPITBF = 0;              //Clear Transmit Buffer Full Status bit
  return ( 0 );
}
/******************************************************************************
 * Function:        WORD_VAL ReadPOT(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          WORD_VAL - the 10-bit right justified POT value
 *
 * Side Effects:    ADC buffer value updated
 *
 * Overview:        This function reads the POT and leaves the value in the
 *                  ADC buffer register
 *
 * Note:            None
 *****************************************************************************/
WORD_VAL ReadPOT(int channel) {
    WORD_VAL w;

    w.Val = 0;
    switch(channel){
    case 0:
        ADCON0 = 0x01; // Select channel 0 (AN0);
        break;
    case 1:
        ADCON0 = 0x05; // Select channel 1 (AN1);
        break;
    }
    
    ADCON0bits.GO = 1; // Start AD conversion
    while (ADCON0bits.GO); // Wait for conversion
    w.v[0] = ADRESL;
    w.v[1] = ADRESH;

    return w;
}//end ReadPOT

/********************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs
 *                  corresponding to the USB device state.
 *
 * Note:            mLED macros can be found in HardwareProfile.h
 *                  USBDeviceState is declared and updated in
 *                  usb_device.c.
 *******************************************************************/
void BlinkUSBStatus(void) {
    static WORD led_count = 0;

    if (led_count == 0)led_count = 10000U;
    led_count--;

#define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
#define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
#define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
#define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if (USBSuspendControl == 1) {
        if (led_count == 0) {
            mLED_1_Toggle();
            if (mGetLED_1()) {
                mLED_2_On();
            } else {
                mLED_2_Off();
            }
        }//end if
    } else {
        if (USBDeviceState == DETACHED_STATE) {
            mLED_Both_Off();
        } else if (USBDeviceState == ATTACHED_STATE) {
            mLED_Both_On();
        } else if (USBDeviceState == POWERED_STATE) {
            mLED_Only_1_On();
        } else if (USBDeviceState == DEFAULT_STATE) {
            mLED_Only_2_On();
        } else if (USBDeviceState == ADDRESS_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        } else if (USBDeviceState == CONFIGURED_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle();
                if (mGetLED_1()) {
                    mLED_2_Off();
                } else {
                    mLED_2_On();
                }
            }//end if
        }
    }

}//end BlinkUSBStatus




// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA* each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

// Note *: The "usb_20.pdf" specs indicate 500uA or 2.5mA, depending upon device classification. However,
// the USB-IF has officially issued an ECN (engineering change notice) changing this to 2.5mA for all
// devices.  Make sure to re-download the latest specifications to get all of the newest ECNs.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void) {
    //Example power saving code.  Insert appropriate code here for the desired
    //application behavior.  If the microcontroller will be put to sleep, a
    //process similar to that shown below may be used:

    //ConfigureIOPinsForLowPower();
    //SaveStateOfAllInterruptEnableBits();
    //DisableAllInterruptEnableBits();
    //EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
    //Sleep();
    //RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
    //RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

    //IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is
    //cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause
    //things to not work as intended.


#if defined(__C30__) || defined __XC16__
    //This function requires that the _IPL level be something other than 0.
    //  We can set it here to something other than
#ifndef DSPIC33E_USB_STARTER_KIT
    _IPL = 1;
    USBSleepOnSuspend();
#endif
#endif
}

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *
 *					This call back is invoked when a wakeup from USB suspend
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void) {
    // If clock switching or other power savings measures were taken when
    // executing the USBCBSuspend() function, now would be a good time to
    // switch back to normal full power run mode conditions.  The host allows
    // 10+ milliseconds of wakeup time, after which the device must be
    // fully back to normal, and capable of receiving and processing USB
    // packets.  In order to do this, the USB module must receive proper
    // clocking (IE: 48MHz clock must be available to SIE for full speed USB
    // operation).
    // Make sure the selected oscillator settings are consistent with USB
    // operation before returning from this function.
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void) {
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void) {
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

    // Typically, user firmware does not need to do anything special
    // if a USB error occurs.  For example, if the host sends an OUT
    // packet to your device, but the packet gets corrupted (ex:
    // because of a bad connection, or the user unplugs the
    // USB cable during the transmission) this will typically set
    // one or more USB error interrupt flags.  Nothing specific
    // needs to be done however, since the SIE will automatically
    // send a "NAK" packet to the host.  In response to this, the
    // host will normally retry to send the packet again, and no
    // data loss occurs.  The system will typically recover
    // automatically, without the need for application firmware
    // intervention.

    // Nevertheless, this callback function is provided, such as
    // for debugging purposes.
}

/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void) {
    USBCheckHIDRequest();
}//end

/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void) {
    // Must claim session ownership if supporting this request
}//end

/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This
 *					callback function should initialize the endpoints
 *					for the device's usage according to the current
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void) {
    //enable the HID endpoint
    USBEnableEndpoint(HID_EP, USB_IN_ENABLED | USB_OUT_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);
    //Re-arm the OUT endpoint for the next packet
    USBOutHandle = HIDRxPacket(HID_EP, (BYTE*) & ReceivedDataBuffer, 64);
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function will only be able to wake up the host if
 *                  all of the below are true:
 *
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior,
 *                  as a USB device that has not been armed to perform remote
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex:
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup.
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void) {
    static WORD delay_count;

    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager
    //properties page for the USB device, power management tab, the
    //"Allow this device to bring the computer out of standby." checkbox
    //should be checked).
    if (USBGetRemoteWakeupStatus() == TRUE) {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if (USBIsBusSuspended() == TRUE) {
            USBMaskInterrupts();

            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FALSE; //So we don't execute this code again,
            //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at
            //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yeilds
            //5ms+ total delay since start of idle).
            delay_count = 3600U;
            do {
                delay_count--;
            } while (delay_count);

            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1; // Start RESUME signaling
            delay_count = 1800U; // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 0; //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}

/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size) {
    switch (event) {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            //Add application specific callback task or callback function here if desired.
            //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
            //FEATURE (endpoint halt) request on an application endpoint which was
            //previously armed (UOWN was = 1).  Here would be a good place to:
            //1.  Determine which endpoint the transaction that just got terminated was
            //      on, by checking the handle value in the *pdata.
            //2.  Re-arm the endpoint if desired (typically would be the case for OUT
            //      endpoints).
            break;
        default:
            break;
    }
    return TRUE;
}

/** EOF main.c *************************************************/

#endif
