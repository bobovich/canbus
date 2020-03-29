/*
 * cc1101_conf.h
 *
 *  Created on: 27 ����. 2020 �.
 *      Author: ����
 */

#ifndef CC1101_CONF_H_
#define CC1101_CONF_H_


uint8_t rfSettings[] =
{
    0x29,  // IOCFG2             0x0000  GDO2 Output Pin Configuration
    0x2e,  // IOCFG1             0x0001  GDO1 Output Pin Configuration
    0x07,  // IOCFG0             0x0002  GDO0 Output Pin Configuration
    0x47,  // FIFOTHR            0x0003  RX FIFO and TX FIFO Thresholds
    0xD3,  // SYNC1              0x0004  Sync Word, High Byte
    0x91,  // SYNC0              0x0005  Sync Word, Low Byte
    0xFF,  // PKTLEN             0x0006  Packet Length
    0x0F,  // PKTCTRL1           0x0007  Packet Automation Control
    0x05,  // PKTCTRL0           0x0008  Packet Automation Control
    0x00,  // ADDR               0x0009  Device Address
    0x00,  // CHANNR             0x000A  Channel Number
    0x06,  // FSCTRL1            0x000B  Frequency Synthesizer Control
    0x00,  // FSCTRL0            0x000C  Frequency Synthesizer Control
    0x21,  // FREQ2              0x000D  Frequency Control Word, High Byte
    0x62,  // FREQ1              0x000E  Frequency Control Word, Middle Byte
    0x76,  // FREQ0              0x000F  Frequency Control Word, Low Byte
    0xC8,  // MDMCFG4            0x0010  Modem Configuration
    0x93,  // MDMCFG3            0x0011  Modem Configuration
    0x13,  // MDMCFG2            0x0012  Modem Configuration
    0x22,  // MDMCFG1            0x0013  Modem Configuration
    0xF8,  // MDMCFG0            0x0014  Modem Configuration
    0x34,  // DEVIATN            0x0015  Modem Deviation Setting
    0x07,  // MCSM2              0x0016  Main Radio Control State Machine Configuration
    0x3F,  // MCSM1              0x0017  Main Radio Control State Machine Configuration
    0x18,  // MCSM0              0x0018  Main Radio Control State Machine Configuration
    0x16,  // FOCCFG             0x0019  Frequency Offset Compensation Configuration
    0x6C,  // BSCFG              0x001A  Bit Synchronization Configuration
    0x43,  // AGCCTRL2           0x001B  AGC Control
    0x40,  // AGCCTRL1           0x001C  AGC Control
    0x91,  // AGCCTRL0           0x001D  AGC Control
    0x87,  // WOREVT1            0x001E  High Byte Event0 Timeout
    0x6B,  // WOREVT0            0x001F  Low Byte Event0 Timeout
    0xFB,  // WORCTRL            0x0020  Wake On Radio Control
    0x56,  // FREND1             0x0021  Front End RX Configuration
    0x10,  // FREND0             0x0022  Front End TX Configuration
    0xE9,  // FSCAL3             0x0023  Frequency Synthesizer Calibration
    0x2A,  // FSCAL2             0x0024  Frequency Synthesizer Calibration
    0x00,  // FSCAL1             0x0025  Frequency Synthesizer Calibration
    0x1F,  // FSCAL0             0x0026  Frequency Synthesizer Calibration
    0x41,  // RCCTRL1            0x0027  RC Oscillator Configuration
    0x00,  // RCCTRL0            0x0028  RC Oscillator Configuration
    0x59,  // FSTEST             0x0029  Frequency Synthesizer Calibration Control
    0x7F,  // PTEST              0x002A  Production Test
    0x3F,  // AGCTEST            0x002B  AGC Test
    0x81,  // TEST2              0x002C  Various Test Settings
    0x35,  // TEST1              0x002D  Various Test Settings
    0x09  // TEST0              0x002E  Various Test Settings
		   // PARTNUM            0x0030  Chip ID
      	   // VERSION            0x0031  Chip ID
      // FREQEST            0x0032  Frequency Offset Estimate from Demodulator
      // LQI                0x0033  Demodulator Estimate for Link Quality
      // RSSI               0x0034  Received Signal Strength Indication
      // MARCSTATE          0x0035  Main Radio Control State Machine State
      // WORTIME1           0x0036  High Byte of WOR Time
			// WORTIME0           0x0037  Low Byte of WOR Time
      // PKTSTATUS          0x0038  Current GDOx Status and Packet Status
	// VCO_VC_DAC         0x0039  Current Setting from PLL Calibration Module
      	  // TXBYTES            0x003A  Underflow and Number of Bytes
      	// RXBYTES            0x003B  Overflow and Number of Bytes
      	// RCCTRL1_STATUS     0x003C  Last RC Oscillator Calibration Result
		// RCCTRL0_STATUS     0x003D  Last RC Oscillator Calibration Result
};


uint8_t patable868[] =
{

};





#endif /* CC1101_CONF_H_ */
