/*************************************************** 
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// #include "ST7796Driver.h"
#include "ST7796Driver.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

#ifdef ENABLE_ST77XX_FRAMEBUFFER
#define DEBUG_ASYNC_UPDATE
//#define DEBUG_ASYNC_LEDS
#ifdef DEBUG_ASYNC_LEDS
  #define DEBUG_PIN_1 0
  #define DEBUG_PIN_2 1
  #define DEBUG_PIN_3 2
#endif

volatile short _dma_dummy_rx;

ST7796Driver *ST7796Driver::_dmaActiveDisplay[3] = {0, 0, 0};

#if defined(__MK66FX1M0__) 
 DMASetting   ST7796Driver::_dmasettings[3][4];
#endif

#if defined(__IMXRT1062__)  // Teensy 4.x
// On T4 Setup the buffers to be used one per SPI buss... 
// This way we make sure it is hopefully in uncached memory
ST7735DMA_Data ST7796Driver::_dma_data[3];   // one structure for each SPI buss... 
#endif

#endif

// Constructor when using software SPI.  All output pins are configurable.
ST7796Driver::ST7796Driver(uint8_t cs, uint8_t rs, uint8_t sid, uint8_t sclk, uint8_t rst,uint16_t s_width,uint16_t s_height) : GFXcanvas16(s_width,s_height)
{
  
	_cs   = cs;
	_rs   = rs;
	_sid  = sid;
	_sclk = sclk;
	_rst  = rst;
	_rot = 0xff;
	hwSPI = false;
  _fb1 =	(uint16_t*)extmem_malloc(s_width*s_height*2);

	_dma_state = 0;
	_screenHeight = s_height;
	_screenWidth = s_width;	
	
	_width = _screenWidth;
	_height = _screenHeight;
	
	cursor_y  = cursor_x    = 0;
	textsize_x  = 1;
	textsize_y  = 1;
	textcolor = textbgcolor = 0xFFFF;
	wrap      = true;
	font      = NULL;
	gfxFont   = NULL;
	setClipRect();
	setOrigin();
}


/***************************************************************/
/*     Teensy 3.0, 3.1, 3.2, 3.5, 3.6                          */
/***************************************************************/
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

inline FLASHMEM void ST7796Driver::waitTransmitComplete(void)  {
    uint32_t tmp __attribute__((unused));
    while (!(_pkinetisk_spi->SR & SPI_SR_TCF)) ; // wait until final output done
    tmp = _pkinetisk_spi->POPR;                  // drain the final RX FIFO word
}

inline FLASHMEM void ST7796Driver::waitTransmitComplete(uint32_t mcr) {
    uint32_t tmp __attribute__((unused));
    while (1) {
        uint32_t sr = _pkinetisk_spi->SR;
        if (sr & SPI_SR_EOQF) break;  // wait for last transmit
        if (sr &  0xF0) tmp = _pkinetisk_spi->POPR;
    }
    _pkinetisk_spi->SR = SPI_SR_EOQF;
    _pkinetisk_spi->MCR = mcr;
    while (_pkinetisk_spi->SR & 0xF0) {
        tmp = _pkinetisk_spi->POPR;
    }
}

inline FLASHMEM void ST7796Driver::spiwrite(uint8_t c)
{
	// pass 1 if we actually are setup to with MOSI and SCLK on hardware SPI use it...
	if (_pspi) {
		_pspi->transfer(c);
		return;
	}

	for (uint8_t bit = 0x80; bit; bit >>= 1) {
		*datapin = ((c & bit) ? 1 : 0);
		*clkpin = 1;
		*clkpin = 0;
	}
}

inline FLASHMEM void ST7796Driver::spiwrite16(uint16_t d)
{
	// pass 1 if we actually are setup to with MOSI and SCLK on hardware SPI use it...
	if (_pspi) {
		_pspi->transfer16(d);
		return;
	}
	spiwrite(d >> 8);
	spiwrite(d);
}

FLASHMEM void ST7796Driver::writecommand(uint8_t c)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
	} else {
		*rspin = 0;
		spiwrite(c);
	}
}

FLASHMEM void ST7796Driver::writecommand_last(uint8_t c) {
	if (hwSPI) {
		uint32_t mcr = _pkinetisk_spi->MCR;
		_pkinetisk_spi->PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
		waitTransmitComplete(mcr);
	} else {
		*rspin = 0;
		spiwrite(c);
	}
}

FLASHMEM void ST7796Driver::writedata(uint8_t c)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
	} else {
		*rspin = 1;
		spiwrite(c);
	}
}

FLASHMEM void ST7796Driver::writedata_last(uint8_t c)
{
	if (hwSPI) {
		uint32_t mcr = _pkinetisk_spi->MCR;
		_pkinetisk_spi->PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
		waitTransmitComplete(mcr);
	} else {
		*rspin = 1;
		spiwrite(c);
	}
}

FLASHMEM void ST7796Driver::writedata16(uint16_t d)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
	} else {
		*rspin = 1;
		spiwrite16(d);
	}
}


FLASHMEM void ST7796Driver::writedata16_last(uint16_t d)
{
	if (hwSPI) {
		uint32_t mcr = _pkinetisk_spi->MCR;
		_pkinetisk_spi->PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1) | SPI_PUSHR_EOQ;
		waitTransmitComplete(mcr);
	} else {
		*rspin = 1;
		spiwrite16(d);
	}
}


#define CTAR_24MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
#define CTAR_16MHz   (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
#define CTAR_12MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
#define CTAR_8MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
#define CTAR_6MHz    (SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))
#define CTAR_4MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))

FLASHMEM void ST7796Driver::setBitrate(uint32_t n)
{
	if (n >= 24000000) {
		ctar = CTAR_24MHz;
	} else if (n >= 16000000) {
		ctar = CTAR_16MHz;
	} else if (n >= 12000000) {
		ctar = CTAR_12MHz;
	} else if (n >= 8000000) {
		ctar = CTAR_8MHz;
	} else if (n >= 6000000) {
		ctar = CTAR_6MHz;
	} else {
		ctar = CTAR_4MHz;
	}
	SIM_SCGC6 |= SIM_SCGC6_SPI0;
	_pkinetisk_spi->MCR = SPI_MCR_MDIS | SPI_MCR_HALT;
	_pkinetisk_spi->CTAR0 = ctar | SPI_CTAR_FMSZ(7);
	_pkinetisk_spi->CTAR1 = ctar | SPI_CTAR_FMSZ(15);
	_pkinetisk_spi->MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
}


/***************************************************************/
/*     Teensy 4.                                               */
/***************************************************************/
#elif defined(__IMXRT1062__)  // Teensy 4.x
inline FLASHMEM void ST7796Driver::spiwrite(uint8_t c)
{
//Serial.println(c, HEX);
	if (_pspi) {
		_pspi->transfer(c);
	} else {
		// Fast SPI bitbang swiped from LPD8806 library
		for(uint8_t bit = 0x80; bit; bit >>= 1) {
			if(c & bit) DIRECT_WRITE_HIGH(_mosiport, _mosipinmask);
			else        DIRECT_WRITE_LOW(_mosiport, _mosipinmask);
			DIRECT_WRITE_HIGH(_sckport, _sckpinmask);
			asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");
			DIRECT_WRITE_LOW(_sckport, _sckpinmask);
		}
	}
}

FLASHMEM void ST7796Driver::writecommand(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) /*| LPSPI_TCR_CONT*/);
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitFifoNotFull();
	} else {
		DIRECT_WRITE_LOW(_dcport, _dcpinmask);
		spiwrite(c);
	}
}

FLASHMEM void ST7796Driver::writecommand_last(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7));
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_LOW(_dcport, _dcpinmask);
		spiwrite(c);
	}

}

FLASHMEM void ST7796Driver::writedata(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(c);
	}
} 

FLASHMEM void ST7796Driver::writedata_last(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(c);
	}
} 


FLASHMEM void ST7796Driver::writedata16(uint16_t d)
{
	if (hwSPI) {
		maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_CONT);
		_pimxrt_spi->TDR = d;
		_pending_rx_count++;	//
		waitFifoNotFull();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(d >> 8);
		spiwrite(d);
	}
} 

FLASHMEM void ST7796Driver::writedata16_last(uint16_t d)
{
	if (hwSPI) {
		maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15));
		_pimxrt_spi->TDR = d;
//		_pimxrt_spi->SR = LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(d >> 8);
		spiwrite(d);
	}
} 

FLASHMEM void ST7796Driver::setBitrate(uint32_t n)
{
	if (n >= 8000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV2);
	} else if (n >= 4000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV4);
	} else if (n >= 2000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV8);
	} else {
		SPI.setClockDivider(SPI_CLOCK_DIV16);
	}
}


/***************************************************************/
/*     Teensy LC                                               */
/***************************************************************/
#elif defined(__MKL26Z64__)

FLASHMEM void ST7796Driver::waitTransmitComplete()  {
	if(!_pkinetisl_spi) return; // Software SPI don't do anything
	while (_data_sent_not_completed) {
		uint16_t timeout_count = 0xff; // hopefully enough 
		while (!(_pkinetisl_spi->S & SPI_S_SPRF) && timeout_count--) ; // wait 
		uint8_t d __attribute__((unused));
		d = _pkinetisl_spi->DL;
		d = _pkinetisl_spi->DH;
		_data_sent_not_completed--; // We hopefully received our data...
	}
}

FLASHMEM void ST7796Driver::spiwrite16(uint16_t data)  {
	if (_pkinetisl_spi) {
		if (!(_pkinetisl_spi->C2 & SPI_C2_SPIMODE)) {
			// Wait to change modes until any pending output has been done.
			waitTransmitComplete();
			_pkinetisl_spi->C2 = SPI_C2_SPIMODE; // make sure 8 bit mode.
		}
		uint8_t s;
		do {
			s = _pkinetisl_spi->S;
			 // wait if output buffer busy.
			// Clear out buffer if there is something there...
			if  ((s & SPI_S_SPRF)) {
				uint8_t d __attribute__((unused));
				d = _pkinetisl_spi->DL;
				d = _pkinetisl_spi->DH;
				_data_sent_not_completed--; 	// let system know we sent something	
			}

		} while (!(s & SPI_S_SPTEF) || (s & SPI_S_SPRF));

		_pkinetisl_spi->DL = data; 		// output low byte
		_pkinetisl_spi->DH = data >> 8; // output high byte
		_data_sent_not_completed++; 	// let system know we sent something	
	} else {
		// call bitbang functions	
		spiwrite(data >> 8);
		spiwrite(data);
	}
}

inline FLASHMEM void ST7796Driver::spiwrite(uint8_t c)
{
//Serial.println(c, HEX);
	if (_pkinetisl_spi) {
		if (_pkinetisl_spi->C2 & SPI_C2_SPIMODE) {
			// Wait to change modes until any pending output has been done.
			waitTransmitComplete();
			_pkinetisl_spi->C2 = 0; // make sure 8 bit mode.
		}
		while (!(_pkinetisl_spi->S & SPI_S_SPTEF)) ; // wait if output buffer busy.
		// Clear out buffer if there is something there...
		if  ((_pkinetisl_spi->S & SPI_S_SPRF)) {
			uint8_t d __attribute__((unused));
			d = _pkinetisl_spi->DL;
			_data_sent_not_completed--;
		} 
		_pkinetisl_spi->DL = c; // output byte
		_data_sent_not_completed++; // let system know we sent something	

	} else {
		// Fast SPI bitbang swiped from LPD8806 library
		for(uint8_t bit = 0x80; bit; bit >>= 1) {
			if(c & bit) *dataport |=  datapinmask;
			else        *dataport &= ~datapinmask;
			*clkport |=  clkpinmask;
			*clkport &= ~clkpinmask;
		}
	}
}

FLASHMEM void ST7796Driver::writecommand(uint8_t c)
{
	setCommandMode();
	spiwrite(c);
}
FLASHMEM void ST7796Driver::writecommand_last(uint8_t c)
{
	setCommandMode();
	spiwrite(c);
	waitTransmitComplete();
}

FLASHMEM void ST7796Driver::writedata(uint8_t c)
{
	setDataMode();
	spiwrite(c);
} 

FLASHMEM void ST7796Driver::writedata_last(uint8_t c)
{
	setDataMode();
	spiwrite(c);
	waitTransmitComplete();
} 

FLASHMEM void ST7796Driver::writedata16(uint16_t d)
{
	setDataMode();
	spiwrite16(d);
} 

FLASHMEM void ST7796Driver::writedata16_last(uint16_t d)
{
	setDataMode();
	spiwrite16(d);
	waitTransmitComplete();
	_pkinetisl_spi->C2 = 0; // Set back to 8 bit mode...
	_pkinetisl_spi->S;	// Read in the status;
} 

FLASHMEM void ST7796Driver::setBitrate(uint32_t n)
{
	if (n >= 8000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV2);
	} else if (n >= 4000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV4);
	} else if (n >= 2000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV8);
	} else {
		SPI.setClockDivider(SPI_CLOCK_DIV16);
	}
}
#endif //#if defined(__SAM3X8E__)


// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
static const uint8_t PROGMEM
  cmd7796[]={                  // Initialization commands for 7796 screens
    9,                       // 9 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      150,                     //    150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x05,                   //     16-bit color
      10,                     //     10 ms delay
    ST7735_MADCTL , 1      ,  //  4: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    ST7735_CASET  , 4      ,  //  5: Column addr set, 4 args, no delay:
      0x00, 
      0x00,                   //     XSTART = 0
      320>>8, 
      320&0xFF,                    //      XEND = 320
    ST7735_RASET  , 4      ,  // 6: Row addr set, 4 args, no delay:
      0x00, 
      0x00,                   //     YSTART = 0
      480>>8, 
      480 & 0xFF,             //      YEND = 480
    ST7735_INVON ,   DELAY,   // 7: hack
      10,
    ST7735_NORON  ,   DELAY,  // 8: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 9: Main screen turn on, no args, w/delay
    255 };  

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
FLASHMEM void ST7796Driver::commandList(const uint8_t *addr)
{
	uint8_t  numCommands, numArgs;
	uint16_t ms;

	beginSPITransaction();
	numCommands = pgm_read_byte(addr++);		// Number of commands to follow
	//Serial.printf("CommandList: numCmds:%d\n", numCommands); Serial.flush();
	while(numCommands--) {				// For each command...
		writecommand_last(pgm_read_byte(addr++));	//   Read, issue command
		numArgs  = pgm_read_byte(addr++);	//   Number of args to follow
		ms       = numArgs & DELAY;		//   If hibit set, delay follows args
		numArgs &= ~DELAY;			//   Mask out delay bit
		while(numArgs > 1) {			//   For each argument...
			writedata(pgm_read_byte(addr++)); //   Read, issue argument
			numArgs--;
		}

		if (numArgs) writedata_last(pgm_read_byte(addr++)); //   Read, issue argument - wait until this one completes
		if(ms) {
			ms = pgm_read_byte(addr++);	// Read post-command delay time (ms)
			if(ms == 255) ms = 500;		// If 255, delay for 500 ms
			//Serial.printf("delay %d\n", ms); Serial.flush();
			endSPITransaction();
			delay(ms);
			beginSPITransaction();
		}
	}
	endSPITransaction();
}


// Initialization code common to both 'B' and 'R' type displays
FLASHMEM void ST7796Driver::commonInit(const uint8_t *cmdList, uint8_t mode)
{
	_colstart  = _rowstart = 0; // May be overridden in init func
  	_ystart = _xstart = 0;
	// Teensy 4
  #if defined(__IMXRT1062__)  // Teensy 4.x 
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;
	if (SPI.pinIsMOSI(_sid) && SPI.pinIsSCK(_sclk)) {
		_pspi = &SPI;
		_spi_num = 0;          // Which buss is this spi on? 
		_pimxrt_spi = &IMXRT_LPSPI4_S;  // Could hack our way to grab this from SPI object, but...

	} else if (SPI1.pinIsMOSI(_sid) && SPI1.pinIsSCK(_sclk)) {
		_pspi = &SPI1;
		_spi_num = 1;          // Which buss is this spi on? 
		_pimxrt_spi = &IMXRT_LPSPI3_S;
	} else if (SPI2.pinIsMOSI(_sid) && SPI2.pinIsSCK(_sclk)) {
		_pspi = &SPI2;
		_spi_num = 2;          // Which buss is this spi on? 
		_pimxrt_spi = &IMXRT_LPSPI1_S;
	} else _pspi = nullptr;

	if (_pspi) {
		hwSPI = true;
		_pspi->begin();
		_pending_rx_count = 0;
		_spiSettings = SPISettings(ST7735_SPICLOCK, MSBFIRST, mode);
		_pspi->beginTransaction(_spiSettings); // Should have our settings. 
		_pspi->transfer(0);	// hack to see if it will actually change then...
		_pspi->endTransaction();
		_spi_tcr_current = _pimxrt_spi->TCR; // get the current TCR value 
//		uint32_t *phack = (uint32_t* )&_spiSettings;
//		Serial.printf("SPI Settings: TCR: %x %x (%x %x)\n", _spi_tcr_current, _pimxrt_spi->TCR, phack[0], phack[1]);
		// Hack to get hold of the SPI Hardware information... 
	 	uint32_t *pa = (uint32_t*)((void*)_pspi);
		_spi_hardware = (SPIClass::SPI_Hardware_t*)(void*)pa[1];
	
	} else {
		hwSPI = false;
		_sckport = portOutputRegister(_sclk);
		_sckpinmask = digitalPinToBitMask(_sclk);
		pinMode(_sclk, OUTPUT);	
		DIRECT_WRITE_LOW(_sckport, _sckpinmask);

		_mosiport = portOutputRegister(_sid);
		_mosipinmask = digitalPinToBitMask(_sid);
		pinMode(_sid, OUTPUT);	
		DIRECT_WRITE_LOW(_mosiport, _mosipinmask);

	}
	if (_cs != 0xff) {
		_csport = portOutputRegister(_cs);
		_cspinmask = digitalPinToBitMask(_cs);
		pinMode(_cs, OUTPUT);	
		DIRECT_WRITE_HIGH(_csport, _cspinmask);		
	} else _csport = 0;

	if (_pspi && _pspi->pinIsChipSelect(_rs)) {
	 	uint8_t dc_cs_index = _pspi->setCS(_rs);
	 	_dcport = 0;
	 	_dcpinmask = 0;
	 	dc_cs_index--;	// convert to 0 based
		_tcr_dc_assert = LPSPI_TCR_PCS(dc_cs_index);
    	_tcr_dc_not_assert = LPSPI_TCR_PCS(3);
	} else {
		//Serial.println("ST7796Driver: Error not DC is not valid hardware CS pin");
		_dcport = portOutputRegister(_rs);
		_dcpinmask = digitalPinToBitMask(_rs);
		pinMode(_rs, OUTPUT);	
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		_tcr_dc_assert = LPSPI_TCR_PCS(0);
    	_tcr_dc_not_assert = LPSPI_TCR_PCS(1);
	}
	maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));
  #endif
	// BUGBUG
//	digitalWrite(_cs, LOW);
	if (_rst != 0xff) {
		pinMode(_rst, OUTPUT);
		digitalWrite(_rst, HIGH);
		delay(50);
		digitalWrite(_rst, LOW);
		delay(25);
		digitalWrite(_rst, HIGH);
		delay(50);
	}

	if(cmdList) commandList(cmdList);
  // setRotation(0);
}


// Initialization for ST7735B screens
FLASHMEM void ST7796Driver::init(void)
{
	commonInit(cmd7796);
}

//changed
FLASHMEM void ST7796Driver::setDiffBuffers(ST7796_T4::DiffBuffBase* diff1, ST7796_T4::DiffBuffBase* diff2){
  waitUpdateAsyncComplete();
  if (diff1) {
    _diff1 = diff1;
    _diff2 = diff2;
  } else {
    _diff1 = diff2;
    _diff2 = diff1;
  }
}
FLASHMEM void ST7796Driver::updateScreenWithDiffs(){
  _diff1->computeDiff(_fb1,getBuffer(), 0, _diff_gap, true, _compare_mask);
  _flush_cache(_fb1, 480*320 * 2);
  // _diff1->computeDiff(_fb1, nullptr,_we_allocated_buffer, 0,480, 0,320, 0, 1, _diff_gap, true, _compare_mask);  // create diff while async update
  // uint32_t parsed=0;
  // int stride = 1,xmin=0,xmax=480,ymin=0,ymax=320;
  
  // endSPITransaction();
  _diff1->initRead();
  int x,y,len;
  int sc1=_diff1->readDiff(x,y,len,0);
  if(sc1<0) return;
  // int prev_x=x;int prev_y=y;
  beginSPITransaction();
  writecommand(ST7735_CASET);
  writedata16(x);
  writedata16(320);
  writecommand(ST7735_RASET);
  writedata16(y);
  writedata16(480);


  int prev_x,prev_y;
  prev_x=x;prev_y=y;
  while(1){
    int new_a=_diff1->readDiff(x,y,len,99999);
    if(new_a<0) break;
    // a+=new_a+1;
    if (x != prev_x) {
      writecommand(ST7735_CASET);
      writedata16(x);
      prev_x = x;
    }
    if (y != prev_y) {
      writecommand(ST7735_RASET);
      writedata16(y);
      prev_y = y;
    }
    writecommand(ST7735_RAMWR);
    push_pixels(_fb1,x,y,len);
  }
  writecommand_last(ST7735_NOP);
  endSPITransaction();
  
}
FLASHMEM void ST7796Driver::push_pixels(uint16_t* fb,int x,int y,int len){
  // len=len<<1;
  const uint16_t* p = fb + x + (y * 320);
  while (len-- > 0) { writedata16(*p++);}
}

FLASHMEM void ST7796Driver::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	beginSPITransaction();
	setAddr(x0, y0, x1, y1);
	writecommand(ST7735_RAMWR); // write to RAM
	// The setAddrWindow/pushColor will only work if SPI is kept active during this loop...
	endSPITransaction();
}


FLASHMEM void ST7796Driver::pushColor(uint16_t color, boolean last_pixel)
{
	//beginSPITransaction();
	if (last_pixel) {
		writedata16_last(color);
		endSPITransaction();
	} else {
		writedata16(color);
	}
}

//#include "glcdfont.c"
extern "C" const unsigned char glcdfont[];

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

FLASHMEM void ST7796Driver::setRotation(uint8_t m)
{
  GFXcanvas16::setRotation(m);
	//Serial.printf("Setting Rotation to %d\n", m);
	beginSPITransaction();
	writecommand(ST7735_MADCTL);
	rotation = m % 4; // can't be higher than 3
	switch (rotation) {
	case 0:
     	if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80)) {
			writedata_last(MADCTL_MX | MADCTL_MY | MADCTL_BGR);
		} else {
			writedata_last(MADCTL_MX | MADCTL_MY | MADCTL_BGR);
		}
		_width  = _screenWidth;
		_height = _screenHeight;
	    _xstart = _colstart;
	    _ystart = _rowstart;
		break;
	case 1:
     	if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80)) {
			writedata_last(MADCTL_MX | MADCTL_BGR);
		} else {
			writedata_last(MADCTL_MX | MADCTL_BGR);
		}
		_height = _screenWidth;
		_width  = _screenHeight;
     	_ystart = _colstart;
     	_xstart = _rowstart;
		break;
	case 2:
     	if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80)) {
			writedata_last(MADCTL_BGR);
		} else {
			writedata_last(MADCTL_BGR);
		}
		_width  = _screenWidth;
		_height = _screenHeight;
     	_xstart = _colstart;
     	// hack to make work on a couple different displays
     	_ystart = (_rowstart==0 || _rowstart==32)? 0 : 1;//_rowstart;
		break;
	case 3:
     	if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80)) {
			writedata_last(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
		} else {
			writedata_last(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
		}
		_width = _screenHeight;
		_height = _screenWidth;
     	_ystart = _colstart;
     	// hack to make work on a couple different displays
     	_xstart = (_rowstart==0 || _rowstart==32)? 0 : 1;//_rowstart;
		break;
	}
	_rot = rotation;	// remember the rotation... 
	endSPITransaction();

	//Serial.printf("SetRotation(%d) _xstart=%d _ystart=%d _width=%d, _height=%d\n", _rot, _xstart, _ystart, _width, _height);

	
	setClipRect();
	setOrigin();
	
	cursor_x = 0;
	cursor_y = 0;
}

FLASHMEM void ST7796Driver::setRowColStart(uint16_t x, uint16_t y) {
	_rowstart = x;
	_colstart = y;
	if (_rot != 0xff) setRotation(_rot);
}


FLASHMEM void ST7796Driver::invertDisplay(boolean i)
{
	beginSPITransaction();
	writecommand_last(i ? ST7735_INVON : ST7735_INVOFF);
	endSPITransaction();

}

/*!
 @brief   Adafruit_SPITFT Send Command handles complete sending of commands and const data
 @param   commandByte       The Command Byte
 @param   dataBytes         A pointer to the Data bytes to send
 @param   numDataBytes      The number of bytes we should send
 */
FLASHMEM void ST7796Driver::sendCommand(uint8_t commandByte, const uint8_t *dataBytes, uint8_t numDataBytes) {
    beginSPITransaction();

    writecommand_last(commandByte); // Send the command byte
  
    while (numDataBytes > 1) {
	  writedata(*dataBytes++); // Send the data bytes
	  numDataBytes--;
    }
    if (numDataBytes) writedata_last(*dataBytes);
  
    endSPITransaction();
}

FLASHMEM FLASHMEM uint16_t ST7796Driver::readPixel(int16_t x, int16_t y)
{
	uint16_t colors = 0;
	readRect(x, y, 1, 1, &colors);
	return colors;
}


// Now lets see if we can read in multiple pixels
FLASHMEM void ST7796Driver::readRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors)
{
	// Use our Origin. 
	x+=_originx;
	y+=_originy;
	//BUGBUG:: Should add some validation of X and Y
  uint16_t * pfbPixel_row = &_fb1[ y*_width + x];
  for (;h>0; h--) {
    uint16_t * pfbPixel = pfbPixel_row;
    for (int i = 0 ;i < w; i++) {
      *pcolors++ = *pfbPixel++;
    }
    pfbPixel_row += _width;
  }
  return;  
}

// Now lets see if we can writemultiple pixels
FLASHMEM void ST7796Driver::writeRect(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pcolors)
{
	if (x == CENTER)
		x = (_width - w) / 2;
	if (y == CENTER)
		y = (_height - h) / 2;

	x+=_originx;
	y+=_originy;

	uint16_t x_clip_left = 0;  // How many entries at start of colors to skip at start of row
	uint16_t x_clip_right = 0;    // how many color entries to skip at end of row for clipping
	// Rectangular clipping 

	// See if the whole thing out of bounds...
	if((x >= _displayclipx2) || (y >= _displayclipy2)) return;
	if (((x+w) <= _displayclipx1) || ((y+h) <= _displayclipy1)) return;

	// In these cases you can not do simple clipping, as we need to synchronize the colors array with the
	// We can clip the height as when we get to the last visible we don't have to go any farther. 
	// also maybe starting y as we will advance the color array. 
 	if(y < _displayclipy1) {
 		int dy = (_displayclipy1 - y);
 		h -= dy; 
 		pcolors += (dy*w); // Advance color array to 
 		y = _displayclipy1; 	
 	}
	if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;
	// For X see how many items in color array to skip at start of row and likewise end of row 
	if(x < _displayclipx1) {
		x_clip_left = _displayclipx1-x; 
		w -= x_clip_left; 
		x = _displayclipx1; 	
	}

	if((x + w - 1) >= _displayclipx2) {
		x_clip_right = w;
		w = _displayclipx2  - x;
		x_clip_right -= w; 
	} 

	if (!skip_buffer) {
		    uint16_t *pfbPixel_row = &_fb1[y * _width + x];
    int16_t y_changed_min = _height;
    int16_t y_changed_max = -1;
    int16_t i_changed_min = _width;
    int16_t i_changed_max = -1;

    for (; h > 0; h--) {
      uint16_t *pfbPixel = pfbPixel_row;
      pcolors += x_clip_left;
      for (int i = 0; i < w; i++) {
        if (*pfbPixel != *pcolors) {
          // pixel changed
          *pfbPixel = *pcolors;
          if (y < y_changed_min) y_changed_min = y;
          if (y > y_changed_max) y_changed_max = y;
          if (i < i_changed_min) i_changed_min = i;
          if (i > i_changed_max) i_changed_max = i;

        }
        pfbPixel++;
        pcolors++;
      }
      pfbPixel_row += _width;
      pcolors += x_clip_right;
      y++;
    }
    // See if we found any chang
    // if any of the min/max values have default value we know that nothing changed.
    if (y_changed_max != -1) {
      updateChangedRange(x + i_changed_min , y_changed_min, 
        (i_changed_max - i_changed_min) + 1, (y_changed_max - y_changed_min) + 1);

      //if(Serial)Serial.printf("WRECT: %d - %d %d %d %d\n", x, i_changed_min, y_changed_min, i_changed_max, y_changed_max);
    }
    return;
  }

   	beginSPITransaction();
	setAddr(x, y, x+w-1, y+h-1);
	writecommand(ST7735_RAMWR);
	for(y=h; y>0; y--) {
		pcolors += x_clip_left;
		for(x=w; x>1; x--) {
			writedata16(*pcolors++);
		}
		writedata16_last(*pcolors++);
		pcolors += x_clip_right;
	}
	endSPITransaction();
}

FLASHMEM void ST7796Driver::writeSubImageRect(int16_t x, int16_t y, int16_t w, int16_t h, 
  int16_t image_offset_x, int16_t image_offset_y, int16_t image_width, int16_t image_height, const uint16_t *pcolors)
{
  if (x == CENTER) x = (_width - w) / 2;
  if (y == CENTER) y = (_height - h) / 2;
  x+=_originx;
  y+=_originy;
  // Rectangular clipping 

  // See if the whole thing out of bounds...
  if((x >= _displayclipx2) || (y >= _displayclipy2)) return;
  if (((x+w) <= _displayclipx1) || ((y+h) <= _displayclipy1)) return;

  // Now lets use or image offsets to get to the first pixels data
  pcolors += image_offset_y * image_width + image_offset_x;

  // In these cases you can not do simple clipping, as we need to synchronize the colors array with the
  // We can clip the height as when we get to the last visible we don't have to go any farther. 
  // also maybe starting y as we will advance the color array. 
  if(y < _displayclipy1) {
    int dy = (_displayclipy1 - y);
    h -= dy; 
    pcolors += (dy * image_width); // Advance color array by that number of rows in the image 
    y = _displayclipy1;   
  }

  if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;

  // For X see how many items in color array to skip at start of row and likewise end of row 
  if(x < _displayclipx1) {
    uint16_t x_clip_left = _displayclipx1-x; 
    w -= x_clip_left; 
    x = _displayclipx1;   
    pcolors += x_clip_left;  // pre index the colors array.
  }
  if((x + w - 1) >= _displayclipx2) {
    uint16_t x_clip_right = w;
    w = _displayclipx2  - x;
    x_clip_right -= w; 
  } 

  if (!skip_buffer) {
    uint16_t * pfbPixel_row = &_fb1[ y*_width + x];
    for (;h>0; h--) {
      const uint16_t *pcolors_row = pcolors; 
      uint16_t * pfbPixel = pfbPixel_row;
      for (int i = 0 ;i < w; i++) {
        *pfbPixel++ = *pcolors++;
      }
      pfbPixel_row += _width;
      pcolors = pcolors_row + image_width;
    }
    return; 
  }

  beginSPITransaction();
  setAddr(x, y, x+w-1, y+h-1);
  writecommand(ST7735_RAMWR);
  for(y=h; y>0; y--) {
    const uint16_t *pcolors_row = pcolors; 
    for(x=w; x>1; x--) {
      writedata16(*pcolors++);
    }
    writedata16_last(*pcolors++);
    pcolors = pcolors_row + image_width;
  }
  endSPITransaction();
}

FLASHMEM void ST7796Driver::writeSubImageRectBytesReversed(int16_t x, int16_t y, int16_t w, int16_t h, 
  int16_t image_offset_x, int16_t image_offset_y, int16_t image_width, int16_t image_height, const uint16_t *pcolors)
{
  if (x == CENTER) x = (_width - w) / 2;
  if (y == CENTER) y = (_height - h) / 2;
  x+=_originx;
  y+=_originy;
  // Rectangular clipping 

  // See if the whole thing out of bounds...
  if((x >= _displayclipx2) || (y >= _displayclipy2)) return;
  if (((x+w) <= _displayclipx1) || ((y+h) <= _displayclipy1)) return;

  // Now lets use or image offsets to get to the first pixels data
  pcolors += image_offset_y * image_width + image_offset_x;

  // In these cases you can not do simple clipping, as we need to synchronize the colors array with the
  // We can clip the height as when we get to the last visible we don't have to go any farther. 
  // also maybe starting y as we will advance the color array. 
  if(y < _displayclipy1) {
    int dy = (_displayclipy1 - y);
    h -= dy; 
    pcolors += (dy * image_width); // Advance color array by that number of rows in the image 
    y = _displayclipy1;   
  }

  if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;

  // For X see how many items in color array to skip at start of row and likewise end of row 
  if(x < _displayclipx1) {
    uint16_t x_clip_left = _displayclipx1-x; 
    w -= x_clip_left; 
    x = _displayclipx1;   
    pcolors += x_clip_left;  // pre index the colors array.
  }
  if((x + w - 1) >= _displayclipx2) {
    uint16_t x_clip_right = w;
    w = _displayclipx2  - x;
    x_clip_right -= w; 
  } 

  if (!skip_buffer) {
    uint16_t * pfbPixel_row = &_fb1[ y*_width + x];
    for (;h>0; h--) {
      const uint16_t *pcolors_row = pcolors; 
      uint16_t * pfbPixel = pfbPixel_row;
      for (int i = 0 ;i < w; i++) {
        *pfbPixel++ = *pcolors++;
      }
      pfbPixel_row += _width;
      pcolors = pcolors_row + image_width;
    }
    return; 
  }

  beginSPITransaction();
  setAddr(x, y, x+w-1, y+h-1);
  writecommand(ST7735_RAMWR);
  for(y=h; y>0; y--) {
    const uint16_t *pcolors_row = pcolors; 
    for(x=w; x>1; x--) {
      uint16_t color = *pcolors++;
      color = ((color & 0xff) << 8) + (color >> 8);
      writedata16(color);
    }
      uint16_t color = *pcolors;
      color = ((color & 0xff) << 8) + (color >> 8);
      writedata16_last(color);
    pcolors = pcolors_row + image_width;
  }
  endSPITransaction();
}

// writeRect8BPP - 	write 8 bit per pixel paletted bitmap
//					bitmap data in array at pixels, one byte per
//pixel
//					color palette data in array at palette
FLASHMEM void ST7796Driver::writeRect8BPP(int16_t x, int16_t y, int16_t w, int16_t h,
                                const uint8_t *pixels,
                                const uint16_t *palette) {
  // Serial.printf("\nWR8: %d %d %d %d %x\n", x, y, w, h, (uint32_t)pixels);
  x += _originx;
  y += _originy;

  uint16_t x_clip_left =
      0; // How many entries at start of colors to skip at start of row
  uint16_t x_clip_right =
      0; // how many color entries to skip at end of row for clipping
  // Rectangular clipping

  // See if the whole thing out of bounds...
  if ((x >= _displayclipx2) || (y >= _displayclipy2))
    return;
  if (((x + w) <= _displayclipx1) || ((y + h) <= _displayclipy1))
    return;

  // In these cases you can not do simple clipping, as we need to synchronize
  // the colors array with the
  // We can clip the height as when we get to the last visible we don't have to
  // go any farther.
  // also maybe starting y as we will advance the color array.
  if (y < _displayclipy1) {
    int dy = (_displayclipy1 - y);
    h -= dy;
    pixels += (dy * w); // Advance color array to
    y = _displayclipy1;
  }

  if ((y + h - 1) >= _displayclipy2)
    h = _displayclipy2 - y;

  // For X see how many items in color array to skip at start of row and
  // likewise end of row
  if (x < _displayclipx1) {
    x_clip_left = _displayclipx1 - x;
    w -= x_clip_left;
    x = _displayclipx1;
  }
  if ((x + w - 1) >= _displayclipx2) {
    x_clip_right = w;
    w = _displayclipx2 - x;
    x_clip_right -= w;
  }
// Serial.printf("WR8C: %d %d %d %d %x- %d %d\n", x, y, w, h, (uint32_t)pixels,
// x_clip_right, x_clip_left);
if (!skip_buffer) {
  	updateChangedRange(
        x, y, w, h); // update the range of the screen that has been changed;
      uint16_t *pfbPixel_row = &_fb1[y * _width + x];
    for (; h > 0; h--) {
      pixels += x_clip_left;
      uint16_t *pfbPixel = pfbPixel_row;
      for (int i = 0; i < w; i++) {
        *pfbPixel++ = palette[*pixels++];
      }
      pixels += x_clip_right;
      pfbPixel_row += _width;
    }
    return;
  }

  beginSPITransaction();
  setAddr(x, y, x + w - 1, y + h - 1);
  writecommand(ST7735_RAMWR);
  for (y = h; y > 0; y--) {
    pixels += x_clip_left;
    // Serial.printf("%x: ", (uint32_t)pixels);
    for (x = w; x > 1; x--) {
      // Serial.print(*pixels, DEC);
      writedata16(palette[*pixels++]);
    }
    // Serial.println(*pixels, DEC);
    writedata16_last(palette[*pixels++]);
    pixels += x_clip_right;
  }
  endSPITransaction();
}

// writeRect4BPP - 	write 4 bit per pixel paletted bitmap
//					bitmap data in array at pixels, 4 bits per
//pixel
//					color palette data in array at palette
//					width must be at least 2 pixels
FLASHMEM void ST7796Driver::writeRect4BPP(int16_t x, int16_t y, int16_t w, int16_t h,
                                const uint8_t *pixels,
                                const uint16_t *palette) {
  // Simply call through our helper
  writeRectNBPP(x, y, w, h, 4, pixels, palette);
}

// writeRect2BPP - 	write 2 bit per pixel paletted bitmap
//					bitmap data in array at pixels, 4 bits per
//pixel
//					color palette data in array at palette
//					width must be at least 4 pixels
FLASHMEM void ST7796Driver::writeRect2BPP(int16_t x, int16_t y, int16_t w, int16_t h,
                                const uint8_t *pixels,
                                const uint16_t *palette) {
  // Simply call through our helper
  writeRectNBPP(x, y, w, h, 2, pixels, palette);
}

///============================================================================
// writeRect1BPP - 	write 1 bit per pixel paletted bitmap
//					bitmap data in array at pixels, 4 bits per
//pixel
//					color palette data in array at palette
//					width must be at least 8 pixels
FLASHMEM void ST7796Driver::writeRect1BPP(int16_t x, int16_t y, int16_t w, int16_t h,
                                const uint8_t *pixels,
                                const uint16_t *palette) {
  // Simply call through our helper
  writeRectNBPP(x, y, w, h, 1, pixels, palette);
}

///============================================================================
// writeRectNBPP - 	write N(1, 2, 4, 8) bit per pixel paletted bitmap
//					bitmap data in array at pixels
//  Currently writeRect1BPP, writeRect2BPP, writeRect4BPP use this to do all of
//  the work.
FLASHMEM void ST7796Driver::writeRectNBPP(int16_t x, int16_t y, int16_t w, int16_t h,
                                uint8_t bits_per_pixel, const uint8_t *pixels,
                                const uint16_t *palette) {
  // Serial.printf("\nWR8: %d %d %d %d %x\n", x, y, w, h, (uint32_t)pixels);
  x += _originx;
  y += _originy;
  uint8_t pixels_per_byte = 8 / bits_per_pixel;
  uint16_t count_of_bytes_per_row =
      (w + pixels_per_byte - 1) /
      pixels_per_byte; // Round up to handle non multiples
  uint8_t row_shift_init =
      8 - bits_per_pixel; // We shift down 6 bits by default
  uint8_t pixel_bit_mask = (1 << bits_per_pixel) - 1; // get mask to use below
  // Rectangular clipping

  // See if the whole thing out of bounds...
  if ((x >= _displayclipx2) || (y >= _displayclipy2))
    return;
  if (((x + w) <= _displayclipx1) || ((y + h) <= _displayclipy1))
    return;

  // In these cases you can not do simple clipping, as we need to synchronize
  // the colors array with the
  // We can clip the height as when we get to the last visible we don't have to
  // go any farther.
  // also maybe starting y as we will advance the color array.
  // Again assume multiple of 8 for width
  if (y < _displayclipy1) {
    int dy = (_displayclipy1 - y);
    h -= dy;
    pixels += dy * count_of_bytes_per_row;
    y = _displayclipy1;
  }

  if ((y + h - 1) >= _displayclipy2)
    h = _displayclipy2 - y;

  // For X see how many items in color array to skip at start of row and
  // likewise end of row
  if (x < _displayclipx1) {
    uint16_t x_clip_left = _displayclipx1 - x;
    w -= x_clip_left;
    x = _displayclipx1;
    // Now lets update pixels to the rigth offset and mask
    uint8_t x_clip_left_bytes_incr = x_clip_left / pixels_per_byte;
    pixels += x_clip_left_bytes_incr;
    row_shift_init =
        8 -
        (x_clip_left - (x_clip_left_bytes_incr * pixels_per_byte) + 1) *
            bits_per_pixel;
  }

  if ((x + w - 1) >= _displayclipx2) {
    w = _displayclipx2 - x;
  }

  const uint8_t *pixels_row_start =
      pixels; // remember our starting position offset into row

if (!skip_buffer) {
  	updateChangedRange(
        x, y, w, h); // update the range of the screen that has been changed;
      uint16_t *pfbPixel_row = &_fb1[y * _width + x];
    for (; h > 0; h--) {
      uint16_t *pfbPixel = pfbPixel_row;
      pixels = pixels_row_start;            // setup for this row
      uint8_t pixel_shift = row_shift_init; // Setup mask

      for (int i = 0; i < w; i++) {
        *pfbPixel++ = palette[((*pixels) >> pixel_shift) & pixel_bit_mask];
        if (!pixel_shift) {
          pixel_shift = 8 - bits_per_pixel; // setup next mask
          pixels++;
        } else {
          pixel_shift -= bits_per_pixel;
        }
      }
      pfbPixel_row += _width;
      pixels_row_start += count_of_bytes_per_row;
    }
    return;
  }

  beginSPITransaction();
  setAddr(x, y, x + w - 1, y + h - 1);
  writecommand(ST7735_RAMWR);
  for (; h > 0; h--) {
    pixels = pixels_row_start;            // setup for this row
    uint8_t pixel_shift = row_shift_init; // Setup mask

    for (int i = 0; i < w; i++) {
      writedata16(palette[((*pixels) >> pixel_shift) & pixel_bit_mask]);
      if (!pixel_shift) {
        pixel_shift = 8 - bits_per_pixel; // setup next mask
        pixels++;
      } else {
        pixel_shift -= bits_per_pixel;
      }
    }
    pixels_row_start += count_of_bytes_per_row;
  }
  writecommand_last(ST7735_NOP);
  endSPITransaction();
}


/***************************************************************************************
** Function name:           setTextDatum
** Description:             Set the text position reference datum
***************************************************************************************/
FLASHMEM void ST7796Driver::setTextDatum(uint8_t d)
{
  textdatum = d;
}


FLASHMEM void ST7796Driver::scrollTextArea(uint8_t scrollSize){
	uint16_t awColors[scroll_width];
	for (int y=scroll_y+scrollSize; y < (scroll_y+scroll_height); y++) { 
		readRect(scroll_x, y, scroll_width, 1, awColors); 
		writeRect(scroll_x, y-scrollSize, scroll_width, 1, awColors);  
	}
	fillRect(scroll_x, (scroll_y+scroll_height)-scrollSize, scroll_width, scrollSize, scrollbgcolor);
}

FLASHMEM void ST7796Driver::setScrollTextArea(int16_t x, int16_t y, int16_t w, int16_t h){
	scroll_x = x; 
	scroll_y = y;
	scroll_width = w; 
	scroll_height = h;
}

FLASHMEM void ST7796Driver::setScrollBackgroundColor(uint16_t color){
	scrollbgcolor=color;
	fillRect(scroll_x,scroll_y,scroll_width,scroll_height,scrollbgcolor);
}

FLASHMEM void ST7796Driver::enableScroll(void){
	scrollEnable = true;
}

FLASHMEM void ST7796Driver::disableScroll(void){
	scrollEnable = false;
}

FLASHMEM void ST7796Driver::resetScrollBackgroundColor(uint16_t color){
	scrollbgcolor=color;
}	

uint32_t fetchbit(const uint8_t *p, uint32_t index)
{
	if (p[index >> 3] & (1 << (7 - (index & 7)))) return 1;
	return 0;
}

uint32_t fetchbits_unsigned(const uint8_t *p, uint32_t index, uint32_t required)
{
	uint32_t val = 0;
	do {
		uint8_t b = p[index >> 3];
		uint32_t avail = 8 - (index & 7);
		if (avail <= required) {
			val <<= avail;
			val |= b & ((1 << avail) - 1);
			index += avail;
			required -= avail;
		} else {
			b >>= avail - required;
			val <<= required;
			val |= b & ((1 << required) - 1);
			break;
		}
	} while (required);
	return val;
}

uint32_t fetchbits_signed(const uint8_t *p, uint32_t index, uint32_t required)
{
	uint32_t val = fetchbits_unsigned(p, index, required);
	if (val & (1 << (required - 1))) {
		return (int32_t)val - (1 << required);
	}
	return (int32_t)val;
}

FLASHMEM uint32_t ST7796Driver::fetchpixel(const uint8_t *p, uint32_t index, uint32_t x)
{
	// The byte
	uint8_t b = p[index >> 3];
	// Shift to LSB position and mask to get value
	uint8_t s = ((fontppb-(x % fontppb)-1)*fontbpp);
	// Mask and return
	return (b >> s) & fontbppmask;
}

FLASHMEM void ST7796Driver::dmaInterrupt(void) {
	if (_dmaActiveDisplay[0])  {
		_dmaActiveDisplay[0]->process_dma_interrupt();
	}
}
FLASHMEM void ST7796Driver::dmaInterrupt1(void) {
	if (_dmaActiveDisplay[1])  {
		_dmaActiveDisplay[1]->process_dma_interrupt();
	}
}
FLASHMEM void ST7796Driver::dmaInterrupt2(void) {
	if (_dmaActiveDisplay[2])  {
		_dmaActiveDisplay[2]->process_dma_interrupt();
	}
}

//=============================================================================
// Frame buffer support. 
//=============================================================================
#ifdef DEBUG_ASYNC_UPDATE
extern void dumpDMA_TCD(DMABaseClass *dmabc);
#endif

FLASHMEM void ST7796Driver::process_dma_interrupt(void) {
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_2, HIGH);
#endif
	// Serial.println(" ST7796Driver::process_dma_interrupt");
#if defined(__MK66FX1M0__) 
	// T3.6
	_dmatx.clearInterrupt();

	#ifdef DEBUG_ASYNC_UPDATE
	static uint8_t debug_count = 10;
  if (_frame_callback_on_HalfDone && debug_count) {
  	Serial.printf("DI %x %x %x\n", _dmatx.TCD->SADDR, _dmasettings[_spi_num][0].TCD->SADDR, _dmasettings[_spi_num][1].TCD->SADDR);
  	debug_count--;
  }
  #endif
  if (_frame_callback_on_HalfDone &&
      (_dmatx.TCD->SADDR >= _dmasettings[_spi_num][1].TCD->SADDR)) {
    _dma_sub_frame_count = 1; // set as partial frame.
  } else {
		_dma_frame_count++;
	  _dma_sub_frame_count = 0; // set back to zero

		// See if we are in continuous mode or not..
		if ((_dma_state & ST77XX_DMA_CONT) == 0) {
			// We are in single refresh mode or the user has called cancel so
			// Lets try to release the CS pin
			while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
			writecommand_last(ST7735_NOP);
			endSPITransaction();
			_dma_state &= ~ST77XX_DMA_ACTIVE;
			_dmaActiveDisplay[_spi_num] = 0;	// We don't have a display active any more... 
		}
	}
  if (_frame_complete_callback)
    (*_frame_complete_callback)();

#elif defined(__IMXRT1062__)  // Teensy 4.x
  _dma_data[_spi_num]._dmatx.clearInterrupt();
  if (_frame_callback_on_HalfDone &&
      (_dma_data[_spi_num]._dmatx.TCD->SADDR >= _dma_data[_spi_num]._dmasettings[1].TCD->SADDR)) {
    _dma_sub_frame_count = 1; // set as partial frame.
    if (_frame_complete_callback)
      (*_frame_complete_callback)();
    // Serial.print("-");
  } else {

    _dma_frame_count++;
    _dma_sub_frame_count = 0;
    // See if we are in continuous mode or not..
    if ((_dma_state & ST77XX_DMA_CONT) == 0) {
      // We are in single refresh mode or the user has called cancel so
      // Lets try to release the CS pin
      // Serial.printf("Before FSR wait: %x %x\n", _pimxrt_spi->FSR,
      // _pimxrt_spi->SR);
      while (_pimxrt_spi->FSR & 0x1f)
        ; // wait until this one is complete

      // Serial.printf("Before SR busy wait: %x\n", _pimxrt_spi->SR);
      while (_pimxrt_spi->SR & LPSPI_SR_MBF)
        ; // wait until this one is complete

      _dma_data[_spi_num]._dmatx.clearComplete();
      // Serial.println("Restore FCR");
      _pimxrt_spi->FCR = LPSPI_FCR_TXWATER(
          15);              // _spi_fcr_save;	// restore the FSR status...
      _pimxrt_spi->DER = 0; // DMA no longer doing TX (or RX)

      _pimxrt_spi->CR =
          LPSPI_CR_MEN | LPSPI_CR_RRF | LPSPI_CR_RTF; // actually clear both...
      _pimxrt_spi->SR = 0x3f00; // clear out all of the other status...

      maybeUpdateTCR(_tcr_dc_assert |
                     LPSPI_TCR_FRAMESZ(7)); // output Command with 8 bits
      // Serial.printf("Output NOP (SR %x CR %x FSR %x FCR %x %x TCR:%x)\n",
      // _pimxrt_spi->SR, _pimxrt_spi->CR, _pimxrt_spi->FSR,
      //	_pimxrt_spi->FCR, _spi_fcr_save, _pimxrt_spi->TCR);
      writecommand_last(ST7735_NOP);
      endSPITransaction();
      _dma_state &= ~ST77XX_DMA_ACTIVE;
      _dmaActiveDisplay[_spi_num] =
          0; // We don't have a display active any more...
    } else {
      // Lets try to flush out memory
      if (_frame_complete_callback)
        (*_frame_complete_callback)();
      else if ((uint32_t)_fb1 >= 0x20200000u)
        arm_dcache_flush(_fb1, _count_pixels*2);
    }
  }
  asm("dsb");
#endif
}

FLASHMEM void ST7796Driver::setFrameCompleteCB(void (*pcb)(), bool fCallAlsoHalfDone) {
  _frame_complete_callback = pcb;
  _frame_callback_on_HalfDone = pcb ? fCallAlsoHalfDone : false;

  noInterrupts();
  _dma_state &=
      ~ST77XX_DMA_INIT; // Lets setup  the call backs on next call out
  interrupts();
}

FLASHMEM void ST7796Driver::updateScreen(void)					// call to say update the screen now.
{
	// Not sure if better here to check flag or check existence of buffer.
	// Will go by buffer as maybe can do interesting things?
	if (!skip_buffer) {
		beginSPITransaction();
		if (_standard && !_updateChangedAreasOnly) {
			// Doing full window. 
			setAddr(0, 0, _width-1, _height-1);
			writecommand(ST7735_RAMWR);

			// BUGBUG doing as one shot.  Not sure if should or not or do like
			// main code and break up into transactions...
			uint16_t *pfbtft_end = &_fb1[(_count_pixels)-1];	// setup 
			uint16_t *pftbft = _fb1;

			// Quick write out the data;
			while (pftbft < pfbtft_end) {
				writedata16(*pftbft++);
			}
			writedata16_last(*pftbft);
		} else {
      // setup just to output the clip rectangle area anded with updated area if
      // enabled
      int16_t start_x = _displayclipx1;
      int16_t start_y = _displayclipy1;
      int16_t end_x = _displayclipx2 - 1;
      int16_t end_y = _displayclipy2 - 1;

      if (_updateChangedAreasOnly) {
        // maybe update range of values to update...
        if (_changed_min_x > start_x)
          start_x = _changed_min_x;
        if (_changed_min_y > start_y)
          start_y = _changed_min_y;
        if (_changed_max_x < end_x)
          end_x = _changed_max_x;
        if (_changed_max_y < end_y)
          end_y = _changed_max_y;
      }

      //if (Serial) Serial.printf("updateScreen: (%u %u) - (%u %u)\n", start_x, start_y, end_x, end_y);
      if ((start_x <= end_x) && (start_y <= end_y)) {
        setAddr(start_x, start_y, end_x, end_y);
        writecommand(ST7735_RAMWR);

        // BUGBUG doing as one shot.  Not sure if should or not or do like
        // main code and break up into transactions...
        uint16_t *pfbPixel_row = &_fb1[start_y * _width + start_x];
        for (uint16_t y = start_y; y <= end_y; y++) {
          uint16_t *pfbPixel = pfbPixel_row;
          for (uint16_t x = start_x; x < end_x; x++) {
            writedata16(*pfbPixel++);
          }
          if (y < (end_y))
            writedata16(*pfbPixel);
          else
            writedata16_last(*pfbPixel);
          pfbPixel_row += _width; // setup for the next row.
        }
      }

		}

		endSPITransaction();
	}
  clearChangedRange(); // make sure the dirty range is updated.
}			 

#ifdef DEBUG_ASYNC_UPDATE

FLASHMEM void dumpDMA_TCD(DMABaseClass *dmabc)
{
	Serial.printf("%x %x:", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

	Serial.printf("SA:%x SO:%d AT:%x NB:%x SL:%d DA:%x DO: %d CI:%x DL:%x CS:%x BI:%x\n", (uint32_t)dmabc->TCD->SADDR,
		dmabc->TCD->SOFF, dmabc->TCD->ATTR, dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR, 
		dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA, dmabc->TCD->CSR, dmabc->TCD->BITER);
}
#endif


//==============================================
#ifdef ENABLE_ST77XX_FRAMEBUFFER
FLASHMEM void	ST7796Driver::initDMASettings(void) 
{
	// Serial.printf("initDMASettings called %d\n", _dma_state);
  if (_dma_state & ST77XX_DMA_INIT) { // should test for init, but...
		return;	// we already init this. 
	}
#ifdef DEBUG_ASYNC_LEDS	
  pinMode(DEBUG_PIN_1, OUTPUT); digitalWrite(DEBUG_PIN_1, LOW);
  pinMode(DEBUG_PIN_2, OUTPUT); digitalWrite(DEBUG_PIN_2, LOW);
  pinMode(DEBUG_PIN_3, OUTPUT); digitalWrite(DEBUG_PIN_3, LOW);
#endif


	//Serial.println("InitDMASettings");
	uint8_t dmaTXevent = _spi_hardware->tx_dma_channel;
	_count_pixels = _width*_height;	// cache away the size of the display. 

//	Serial.printf("cbDisplay: %u COUNT_WORDS_WRITE:%d(%x) spi_num:%d\n", _count_pixels, COUNT_WORDS_WRITE, COUNT_WORDS_WRITE, _spi_num);
#if defined(__MK66FX1M0__) 
  uint8_t  cnt_dma_settings = 2;   // how many do we need for this display?
	uint32_t COUNT_WORDS_WRITE = (_count_pixels) / 2;
	// The 240x320 display requires us to expand to another DMA setting. 
	if (COUNT_WORDS_WRITE >= 32768) {
		COUNT_WORDS_WRITE = (_count_pixels) / 3;
		cnt_dma_settings = 3;
	}
	// T3.6
	//Serial.printf("CWW: %d %d %d\n", CBALLOC, SCREEN_DMA_NUM_SETTINGS, count_words_write);
	// Now lets setup DMA access to this memory... 
	_cnt_dma_settings = cnt_dma_settings;	// save away code that needs to update
	_dmasettings[_spi_num][0].sourceBuffer(&_fb1[1], (COUNT_WORDS_WRITE-1)*2);
	_dmasettings[_spi_num][0].destination(_pkinetisk_spi->PUSHR);

	// Hack to reset the destination to only output 2 bytes.
	_dmasettings[_spi_num][0].TCD->ATTR_DST = 1;
	_dmasettings[_spi_num][0].replaceSettingsOnCompletion(_dmasettings[_spi_num][1]);

	_dmasettings[_spi_num][1].sourceBuffer(&_fb1[COUNT_WORDS_WRITE], COUNT_WORDS_WRITE*2);
	_dmasettings[_spi_num][1].destination(_pkinetisk_spi->PUSHR);
	_dmasettings[_spi_num][1].TCD->ATTR_DST = 1;
	_dmasettings[_spi_num][1].replaceSettingsOnCompletion(_dmasettings[_spi_num][2]);

	if (cnt_dma_settings == 3) {
		_dmasettings[_spi_num][2].sourceBuffer(&_fb1[COUNT_WORDS_WRITE*2], COUNT_WORDS_WRITE*2);
		_dmasettings[_spi_num][2].destination(_pkinetisk_spi->PUSHR);
		_dmasettings[_spi_num][2].TCD->ATTR_DST = 1;
		_dmasettings[_spi_num][2].replaceSettingsOnCompletion(_dmasettings[_spi_num][3]);		
		// 3 in chain so half done is half of 1...		
	  if (_frame_callback_on_HalfDone)
	    _dmasettings[_spi_num][1].interruptAtHalf();
	  else
	    _dmasettings[_spi_num][1].TCD->CSR &= ~DMA_TCD_CSR_INTHALF;
	} else {
	  if (_frame_callback_on_HalfDone)
	    _dmasettings[_spi_num][0].interruptAtCompletion();
	  else
	    _dmasettings[_spi_num][0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);
	}
	// Sort of hack - but wrap around to output the first word again. 
	_dmasettings[_spi_num][cnt_dma_settings].sourceBuffer(_fb1, 2);
	_dmasettings[_spi_num][cnt_dma_settings].destination(_pkinetisk_spi->PUSHR);
	_dmasettings[_spi_num][cnt_dma_settings].TCD->ATTR_DST = 1;
	_dmasettings[_spi_num][cnt_dma_settings].replaceSettingsOnCompletion(_dmasettings[_spi_num][0]);



	// Setup DMA main object
	//Serial.println("Setup _dmatx");
	_dmatx.begin(true);
	_dmatx.triggerAtHardwareEvent(dmaTXevent);
	_dmatx = _dmasettings[_spi_num][0];
	// probably could use const table of functio_ns...
	if (_spi_num == 0) _dmatx.attachInterrupt(dmaInterrupt);
	else if (_spi_num == 1) _dmatx.attachInterrupt(dmaInterrupt1);
	else _dmatx.attachInterrupt(dmaInterrupt2);

#elif defined(__IMXRT1062__)  // Teensy 4.x
	_cnt_dma_settings = 2;   // how many do we need for this display?

	uint32_t COUNT_WORDS_WRITE =  (height() * width()) / 2;
	// The 240x320 display requires us to expand to another DMA setting. 
	if (COUNT_WORDS_WRITE >= 32768) {
		COUNT_WORDS_WRITE = (height() * width()) / 3;
		_cnt_dma_settings = 3;
	}

  // First time we init...
  _dma_data[_spi_num]._dmasettings[0].sourceBuffer(_fb1, (COUNT_WORDS_WRITE)*2);
  _dma_data[_spi_num]._dmasettings[0].destination(_pimxrt_spi->TDR);
  _dma_data[_spi_num]._dmasettings[0].TCD->ATTR_DST = 1;
  _dma_data[_spi_num]._dmasettings[0].replaceSettingsOnCompletion(_dma_data[_spi_num]._dmasettings[1]);

  _dma_data[_spi_num]._dmasettings[1].sourceBuffer(&_fb1[COUNT_WORDS_WRITE],
                               COUNT_WORDS_WRITE * 2);
  _dma_data[_spi_num]._dmasettings[1].destination(_pimxrt_spi->TDR);
  _dma_data[_spi_num]._dmasettings[1].TCD->ATTR_DST = 1;

	if (_cnt_dma_settings == 3) {
  	_dma_data[_spi_num]._dmasettings[1].replaceSettingsOnCompletion(_dma_data[_spi_num]._dmasettings[2]);
    _dma_data[_spi_num]._dmasettings[2].sourceBuffer(&_fb1[COUNT_WORDS_WRITE * 2],
                                 COUNT_WORDS_WRITE * 2);
    _dma_data[_spi_num]._dmasettings[2].destination(_pimxrt_spi->TDR);
    _dma_data[_spi_num]._dmasettings[2].TCD->ATTR_DST = 1;
    _dma_data[_spi_num]._dmasettings[2].replaceSettingsOnCompletion(_dma_data[_spi_num]._dmasettings[0]);
    _dma_data[_spi_num]._dmasettings[2].interruptAtCompletion();

		// 3 in chain so half done is half of 1...		
	  if (_frame_callback_on_HalfDone)
	    _dma_data[_spi_num]._dmasettings[1].interruptAtHalf();
	  else
	    _dma_data[_spi_num]._dmasettings[1].TCD->CSR &= ~DMA_TCD_CSR_INTHALF;
	} else {
    _dma_data[_spi_num]._dmasettings[1].replaceSettingsOnCompletion(_dma_data[_spi_num]._dmasettings[0]);
    _dma_data[_spi_num]._dmasettings[1].interruptAtCompletion();
	  if (_frame_callback_on_HalfDone)
	    _dma_data[_spi_num]._dmasettings[0].interruptAtCompletion();
	  else
	    _dma_data[_spi_num]._dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);
	}
	// Setup DMA main object
	//Serial.println("Setup _dmatx");
	// Serial.println("DMA initDMASettings - before dmatx");
	_dma_data[_spi_num]._dmatx.begin(true);
	_dma_data[_spi_num]._dmatx.triggerAtHardwareEvent(dmaTXevent);
	_dma_data[_spi_num]._dmatx = _dma_data[_spi_num]._dmasettings[0];
	// probably could use const table of functions...
	if (_spi_num == 0) _dma_data[_spi_num]._dmatx.attachInterrupt(dmaInterrupt);
	else if (_spi_num == 1) _dma_data[_spi_num]._dmatx.attachInterrupt(dmaInterrupt1);
	else _dma_data[_spi_num]._dmatx.attachInterrupt(dmaInterrupt2);
#else
	// T3.5
	// Lets setup the write size.  For SPI we can use up to 32767 so same size as we use on T3.6...
	// But SPI1 and SPI2 max of 511.  We will use 480 in that case as even divider...

	uint32_t COUNT_WORDS_WRITE = (_count_pixels) / 2;

	// The 240x320 display requires us to expand to another DMA interrupt...
	// Warning, hack, if interrupt at half, make it 4 interrupts...
	if (COUNT_WORDS_WRITE >= 32768) {
	  if (_frame_callback_on_HalfDone) COUNT_WORDS_WRITE = (_count_pixels) / 4;
	  else COUNT_WORDS_WRITE = (_count_pixels) / 3;
	}
	_dmarx.disable();
	_dmarx.source(_pkinetisk_spi->POPR);
	_dmarx.TCD->ATTR_SRC = 1;
	_dmarx.destination(_dma_dummy_rx);
	_dmarx.disableOnCompletion();
	_dmarx.triggerAtHardwareEvent(_spi_hardware->rx_dma_channel);
	// probably could use const table of functions...
	if (_spi_num == 0) _dmarx.attachInterrupt(dmaInterrupt);
	else if (_spi_num == 1) _dmarx.attachInterrupt(dmaInterrupt1);
	else _dmarx.attachInterrupt(dmaInterrupt2);

	_dmarx.interruptAtCompletion();

	// We may be using settings chain here so lets set it up. 
	// Now lets setup TX chain.  Note if trigger TX is not set
	// we need to have the RX do it for us.
	_dmatx.disable();
	_dmatx.destination(_pkinetisk_spi->PUSHR);
	_dmatx.TCD->ATTR_DST = 1;
	_dmatx.disableOnCompletion();
	// Current SPIN, has both RX/TX same for SPI1/2 so just know f
	if (_pspi == &SPI) {
		_dmatx.triggerAtHardwareEvent(dmaTXevent);
		_dma_write_size_words = COUNT_WORDS_WRITE;
	} else {
		_dma_write_size_words = 480;
	    _dmatx.triggerAtTransfersOf(_dmarx);
	}
	//Serial.printf("Init DMA Settings: TX:%d size:%d\n", dmaTXevent, _dma_write_size_words);

#endif
	_dma_state = ST77XX_DMA_INIT;  // Should be first thing set!
	// Serial.println("DMA initDMASettings - end");

}

FLASHMEM void ST7796Driver::dumpDMASettings() {
#ifdef DEBUG_ASYNC_UPDATE
#if defined(__MK66FX1M0__) 
	// T3.6
	Serial.printf("DMA dump TCDs %d\n", _dmatx.channel);
	dumpDMA_TCD(&_dmatx);
	dumpDMA_TCD(&_dmasettings[_spi_num][0]);
	dumpDMA_TCD(&_dmasettings[_spi_num][1]);
	dumpDMA_TCD(&_dmasettings[_spi_num][2]);
	dumpDMA_TCD(&_dmasettings[_spi_num][3]);
#elif defined(__IMXRT1062__)  // Teensy 4.x
	// Serial.printf("DMA dump TCDs %d\n", _dmatx.channel);
	dumpDMA_TCD(&_dma_data[_spi_num]._dmatx);
	dumpDMA_TCD(&_dma_data[_spi_num]._dmasettings[0]);
	dumpDMA_TCD(&_dma_data[_spi_num]._dmasettings[1]);
#else
	Serial.printf("DMA dump TX:%d RX:%d\n", _dmatx.channel, _dmarx.channel);
	dumpDMA_TCD(&_dmatx);
	dumpDMA_TCD(&_dmarx);
#endif	
#endif

}

FLASHMEM bool ST7796Driver::updateScreenAsync(bool update_cont)					// call to say update the screen now.
{
	// Not sure if better here to check flag or check existence of buffer.
	// Will go by buffer as maybe can do interesting things?
	// BUGBUG:: only handles full screen so bail on the rest of it...
	// Also bail if we are working with a hardware SPI port. 


	#if defined(__MK64FX512__) || defined(__MK20DX256__)  // If T3.5 only allow on SPI...
	// The T3.5 DMA to SPI has issues with preserving stuff like we want 16 bit mode
	// and we want CS to stay on... So hack it.  We will turn off using CS for the CS
	//	pin.
	if (!cspin && (_cs != 0xff)) {
		//Serial.println("***T3.5 CS Pin hack");
		pcs_data = 0;
		pcs_command = pcs_data | _pspi->setCS(_rs);
		pinMode(_cs, OUTPUT);
		cspin    = portOutputRegister(digitalPinToPort(_cs));
		*cspin = 1;
	}
	#endif

#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_1, HIGH);
#endif
	// Init DMA settings. 
	initDMASettings();

	// Don't start one if already active.
	if (_dma_state & ST77XX_DMA_ACTIVE) {
	#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_1, LOW);
	#endif
		return false;
	}

#if defined(__MK66FX1M0__) 
	//==========================================
	// T3.6
	//==========================================
	if (update_cont) {
		// Try to link in #3 into the chain (_cnt_dma_settings)
		_dmasettings[_spi_num][_cnt_dma_settings-1].replaceSettingsOnCompletion(_dmasettings[_spi_num][_cnt_dma_settings]);
		_dmasettings[_spi_num][_cnt_dma_settings-1].TCD->CSR &= ~(DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ);  // Don't interrupt on this 1one... 
		_dmasettings[_spi_num][_cnt_dma_settings].interruptAtCompletion();
		_dmasettings[_spi_num][_cnt_dma_settings].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one  
		_dma_state |= ST77XX_DMA_CONT;
	} else {
		// In this case we will only run through once...
		_dmasettings[_spi_num][_cnt_dma_settings-1].replaceSettingsOnCompletion(_dmasettings[_spi_num][0]);
		_dmasettings[_spi_num][_cnt_dma_settings-1].interruptAtCompletion();
		_dmasettings[_spi_num][_cnt_dma_settings-1].disableOnCompletion();
		_dma_state &= ~ST77XX_DMA_CONT;
	}


#ifdef DEBUG_ASYNC_UPDATE
	dumpDMASettings();
#endif
	beginSPITransaction();

	// Doing full window. 
	setAddr(0, 0, _width-1, _height-1);
	writecommand(ST7735_RAMWR);

	// Write the first Word out before enter DMA as to setup the proper CS/DC/Continue flaugs
	writedata16(*_fb1);
	// now lets start up the DMA
//	volatile uint16_t  biter = _dmatx.TCD->BITER;
	//DMA_CDNE_CDNE(_dmatx.channel);
//	_dmatx = _dmasettings[0];
//	_dmatx.TCD->BITER = biter;
	_dma_frame_count = 0;  // Set frame count back to zero. 
	_dmaActiveDisplay[_spi_num] = this;
	_dma_state |= ST77XX_DMA_ACTIVE;
	_pkinetisk_spi->RSER |= SPI_RSER_TFFF_DIRS |	 SPI_RSER_TFFF_RE;	 // Set DMA Interrupt Request Select and Enable register
	_pkinetisk_spi->MCR &= ~SPI_MCR_HALT;  //Start transfers.
	_dmatx.enable();
	//==========================================
	// T4
	//==========================================
#elif defined(__IMXRT1062__)  // Teensy 4.x
  /////////////////////////////
  // BUGBUG try first not worry about continueous or not.
  // Start off remove disable on completion from both...
  // it will be the ISR that disables it...
  if ((uint32_t)_fb1 >= 0x20200000u)
    arm_dcache_flush(_fb1, _count_pixels*2);

  _dma_data[_spi_num]._dmasettings[_cnt_dma_settings-1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);
  beginSPITransaction();
// Doing full window.

  setAddr(0, 0, _width - 1, _height - 1);
  writecommand_last(ST7735_RAMWR);

  // Update TCR to 16 bit mode. and output the first entry.
  _spi_fcr_save = _pimxrt_spi->FCR; // remember the FCR
  _pimxrt_spi->FCR = 0;             // clear water marks...
  maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15) |
                 LPSPI_TCR_RXMSK /*| LPSPI_TCR_CONT*/);
  _pimxrt_spi->DER = LPSPI_DER_TDDE;
  _pimxrt_spi->SR = 0x3f00; // clear out all of the other status...

  _dma_data[_spi_num]._dmatx.triggerAtHardwareEvent(_spi_hardware->tx_dma_channel);

  _dma_data[_spi_num]._dmatx =_dma_data[_spi_num]. _dmasettings[0];

  _dma_data[_spi_num]._dmatx.begin(false);
  _dma_data[_spi_num]._dmatx.enable();

  _dma_frame_count = 0; // Set frame count back to zero.
  _dmaActiveDisplay[_spi_num] = this;
  if (update_cont) {
    _dma_state |= ST77XX_DMA_CONT;
  } else {
    _dma_data[_spi_num]._dmasettings[_cnt_dma_settings-1].disableOnCompletion();
    _dma_state &= ~ST77XX_DMA_CONT;
  }

  _dma_state |= ST77XX_DMA_ACTIVE;
#ifdef DEBUG_ASYNC_UPDATE
  dumpDMASettings();
#endif
#else
	//==========================================
	// T3.5
	//==========================================

	// lets setup the initial pointers. 
	_dmatx.sourceBuffer(&_fb1[1], (_dma_write_size_words-1)*2);
	_dmatx.TCD->SLAST = 0;	// Finish with it pointing to next location
	_dmarx.transferCount(_dma_write_size_words);
	_dma_count_remaining = _count_pixels - _dma_write_size_words;	// how much more to transfer? 
	Serial.printf("updateScreenAsync:: - Pixels:%u Write Size:%u Remaining:%u DMA/Update: %u\n", _count_pixels, _dma_write_size_words, _dma_count_remaining, _count_pixels/_dma_write_size_words);

#ifdef DEBUG_ASYNC_UPDATE
	dumpDMASettings();
#endif

	beginSPITransaction();
	// Doing full window. 
	setAddr(0, 0, _width-1, _height-1);
	writecommand(ST7735_RAMWR);

	// Write the first Word out before enter DMA as to setup the proper CS/DC/Continue flaugs
	// On T3.5 DMA only appears to work with CTAR 0 so hack it up...
	_pkinetisk_spi->CTAR0 |= SPI_CTAR_FMSZ(8); 	// Hack convert from 8 bit to 16 bit...

	_pkinetisk_spi->MCR = SPI_MCR_MSTR | SPI_MCR_CLR_RXF | SPI_MCR_PCSIS(0x1F);

	_pkinetisk_spi->SR = 0xFF0F0000;

	// Lets try to output the first byte to make sure that we are in 16 bit mode...
	_pkinetisk_spi->PUSHR = *_fb1 | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;	

	if (_pspi == &SPI) {
		// SPI - has both TX and RX so use it
		_pkinetisk_spi->RSER =  SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS | SPI_RSER_TFFF_RE | SPI_RSER_TFFF_DIRS;

	    _dmarx.enable();
	    _dmatx.enable();
	} else {
		_pkinetisk_spi->RSER =  SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS ;
	    _dmatx.triggerAtTransfersOf(_dmarx);
	    _dmatx.enable();
	    _dmarx.enable();
	}

	_dma_frame_count = 0;  // Set frame count back to zero. 
	_dmaActiveDisplay[_spi_num] = this;
	if (update_cont) {
		_dma_state |= ST77XX_DMA_CONT;
	} else {
		_dma_state &= ~ST77XX_DMA_CONT;

	}

	_dma_state |= ST77XX_DMA_ACTIVE;
#endif	
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_1, LOW);
#endif
	return true;

}			 

FLASHMEM void ST7796Driver::endUpdateAsync() {
	// make sure it is on
	if (_dma_state & ST77XX_DMA_CONT) {
		_dma_state &= ~ST77XX_DMA_CONT; // Turn off the continueous mode
#if defined(__MK66FX1M0__)
		_dmasettings[_spi_num][_cnt_dma_settings].disableOnCompletion();
#endif
#if defined(__IMXRT1062__)
    _dma_data[_spi_num]._dmasettings[_cnt_dma_settings-1].disableOnCompletion();
#endif
	}
}
	
FLASHMEM void ST7796Driver::waitUpdateAsyncComplete(void) 
{
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_3, HIGH);
#endif

	while ((_dma_state & ST77XX_DMA_ACTIVE)) {
		// asm volatile("wfi");
	};
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_3, LOW);
#endif
}

#endif

void ST7796Driver::clearScreen(){
  fillScreen(0xFF);
  updateScreenWithDiffs();
  fillScreen(0x00);
  updateScreenWithDiffs();
}


