/*
 * modbas_slave.c
 *
 *  Created on: 03/05/2017
 *      Author: andru
 */

#include <stdio.h>
#include <string.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbutils.h"

#include "main.h"
#include "modbus_slave.h"

static const uint16_t mb_bitrates[] = {2400, 4800, 9600, 19200, 38400};

/*------------------------Slave mode use these variables----------------------*/
//Slave mode:DiscreteInputs variables
#if S_DISCRETE_INPUT_NDISCRETES%8
UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8+1];
#else
UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8]  ;
#endif
//Slave mode:Coils variables
#if S_COIL_NCOILS%8
UCHAR    ucSCoilBuf[S_COIL_NCOILS/8+1]                ;
#else
UCHAR    ucSCoilBuf[S_COIL_NCOILS/8]                  ;
#endif
//Slave mode:InputRegister variables
SHORT   usSRegInBuf[S_REG_INPUT_NREGS]                ;
//Slave mode:HoldingRegister variables
SHORT   usSRegHoldBuf[S_REG_HOLDING_NREGS]            ;

// changed coils
#if S_COIL_NCOILS%8
UCHAR ucCoilBytes = S_COIL_NCOILS/8+1		  ;
UCHAR ucChangeCoilBuf[S_COIL_NCOILS/8+1]       ;
#else
UCHAR ucCoilBytes = S_COIL_NCOILS/8		      ;
UCHAR ucChangeCoilBuf[S_COIL_NCOILS/8]         ;
#endif

BOOL initModbus (UCHAR addr, mb_bitrate_t bitrate, mb_parity_t parity) {
  eMBErrorCode eStatus;

  eStatus = eMBInit(MB_MODE, addr, MB_PORT, mb_bitrates[bitrate], parity);
  if (eStatus != MB_ENOERR) {
    return FALSE;
  }

  eStatus = eMBEnable();
  if (eStatus != MB_ENOERR) {
    return FALSE;
  }

  return TRUE;
}

/**
 * Modbus slave discrete callback function.
 *
 * @param pucRegBuffer discrete buffer
 * @param usAddress discrete address
 * @param usNDiscrete discrete number
 *
 * @return result
 */
eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex , iRegBitIndex , iNReg;
    iNReg =  usNDiscrete / 8 + 1;

    /* watchdog set, if not called WDGTIMEOUT board will be reset */
    setwatchdog();

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= S_DISCRETE_INPUT_START)
            && (usAddress + usNDiscrete    <= S_DISCRETE_INPUT_START + S_DISCRETE_INPUT_NDISCRETES))
    {
        iRegIndex = (USHORT) (usAddress - S_DISCRETE_INPUT_START) / 8;
        iRegBitIndex = (USHORT) (usAddress - S_DISCRETE_INPUT_START) % 8;

        while (iNReg > 0)
        {
            *pucRegBuffer++ = xMBUtilGetBits(&ucSDiscInBuf[iRegIndex++], iRegBitIndex, 8);
            iNReg--;
        }
        pucRegBuffer--;
        /* last discrete */
        usNDiscrete = usNDiscrete % 8;
        /* filling zero to high bit */
        *pucRegBuffer = *pucRegBuffer << (8 - usNDiscrete);
        *pucRegBuffer = *pucRegBuffer >> (8 - usNDiscrete);
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/**
 * Modbus slave coils callback function.
 *
 * @param pucRegBuffer coils buffer
 * @param usAddress coils address
 * @param usNCoils coils number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex , iRegBitIndex , iNReg;
    UCHAR			prevCoil;

    /* watchdog set, if not called WDGTIMEOUT board will be reset */
    setwatchdog();

    memset(ucChangeCoilBuf, 0, ucCoilBytes);

    iNReg =  usNCoils / 8 + 1;

    /* it already plus one in modbus function method. */
    usAddress--;

    if( ( usAddress >= S_COIL_START ) &&
        ( usAddress + usNCoils <= S_COIL_START + S_COIL_NCOILS ) )
    {
        iRegIndex = (USHORT) (usAddress - S_COIL_START) / 8;
        iRegBitIndex = (USHORT) (usAddress - S_COIL_START) % 8;
        switch ( eMode )
        {
        /* read current coil values from the protocol stack. */
        case MB_REG_READ:
            while (iNReg > 0)
            {
                *pucRegBuffer++ = xMBUtilGetBits(&ucSCoilBuf[iRegIndex++], iRegBitIndex, 8);
                iNReg--;
            }
            pucRegBuffer--;
            /* last coils */
            usNCoils = usNCoils % 8;
            /* filling zero to high bit */
            *pucRegBuffer = *pucRegBuffer << (8 - usNCoils);
            *pucRegBuffer = *pucRegBuffer >> (8 - usNCoils);
            break;

            /* write current coil values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (iNReg > 1)
            {
            	// check register change
            	prevCoil = xMBUtilGetBits(&ucSCoilBuf[iRegIndex], iRegBitIndex, 8);
               	xMBUtilSetBits(&ucChangeCoilBuf[iRegIndex], iRegBitIndex, 8, *pucRegBuffer ^ prevCoil);

                xMBUtilSetBits(&ucSCoilBuf[iRegIndex++], iRegBitIndex, 8, *pucRegBuffer++);
                iNReg--;
            }
            /* last coils */
            usNCoils = usNCoils % 8;
            /* xMBUtilSetBits has bug when ucNBits is zero */
            if (usNCoils != 0)
            {
            	// check register change
            	prevCoil = xMBUtilGetBits(&ucSCoilBuf[iRegIndex], iRegBitIndex, usNCoils);
                xMBUtilSetBits(&ucChangeCoilBuf[iRegIndex], iRegBitIndex, usNCoils, *pucRegBuffer ^ prevCoil);

                xMBUtilSetBits(&ucSCoilBuf[iRegIndex++], iRegBitIndex, usNCoils, *pucRegBuffer++);
            }

            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}


/**
 * Modbus slave input register callback function.
 *
 * @param pucRegBuffer input register buffer
 * @param usAddress input register address
 * @param usNRegs input register number
 *
 * @return result
 */
eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex;

    /* it already plus one in modbus function method. */
    usAddress--;

    if( ( usAddress >= S_REG_INPUT_START )
        && ( usAddress + usNRegs <= S_REG_INPUT_START + S_REG_INPUT_NREGS ) )
    {
        iRegIndex = ( USHORT )( usAddress - S_REG_INPUT_START );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = (UCHAR)( usSRegInBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = (UCHAR)( usSRegInBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/**
 * Modbus slave holding register callback function.
 *
 * @param pucRegBuffer holding register buffer
 * @param usAddress holding register address
 * @param usNRegs holding register number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= S_REG_HOLDING_START)
            && (usAddress + usNRegs <= S_REG_HOLDING_START + S_REG_HOLDING_NREGS))
    {
        iRegIndex = usAddress - S_REG_HOLDING_START;
        switch (eMode)
        {
        /* read current register values from the protocol stack. */
        case MB_REG_READ:
            while (usNRegs > 0)
            {
                *pucRegBuffer++ = (UCHAR)( usSRegHoldBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = (UCHAR)( usSRegHoldBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

        /* write current register values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (usNRegs > 0)
            {
                usSRegHoldBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usSRegHoldBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
#if 0 // TODO: implementation
#endif
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

void setDiscreteBit(USHORT regAddr, UCHAR ucValue) {
    USHORT          iRegIndex , iRegBitIndex;

    if (( regAddr >= S_DISCRETE_INPUT_START )
    		&& ( regAddr < S_DISCRETE_INPUT_START + S_DISCRETE_INPUT_NDISCRETES ))
    {
	    iRegIndex = (USHORT) (regAddr - S_DISCRETE_INPUT_START) / 8;
	    iRegBitIndex = (USHORT) (regAddr - S_DISCRETE_INPUT_START) % 8;
	    xMBUtilSetBits(&ucSDiscInBuf[iRegIndex], iRegBitIndex, 1, ucValue);
    }
}

uint8_t getCoilBit(USHORT regAddr) {
    USHORT          iRegIndex , iRegBitIndex;

    if (( regAddr >= S_COIL_START )
    		&& ( regAddr < S_COIL_START + S_COIL_NCOILS ) )
    {
        iRegIndex = (USHORT) (regAddr - S_COIL_START) / 8;
        iRegBitIndex = (USHORT) (regAddr - S_COIL_START) % 8;
        return xMBUtilGetBits(&ucSCoilBuf[iRegIndex], iRegBitIndex, 1);
    }
    return 0xFF;
}

void setCoilBit(USHORT regAddr, UCHAR ucValue) {
    USHORT          iRegIndex , iRegBitIndex;

    if (( regAddr >= S_COIL_START )
    		&& ( regAddr < S_COIL_START + S_COIL_NCOILS ) )
    {
	    iRegIndex = (USHORT) (regAddr - S_COIL_START) / 8;
	    iRegBitIndex = (USHORT) (regAddr - S_COIL_START) % 8;
        xMBUtilSetBits(&ucSCoilBuf[iRegIndex], iRegBitIndex, 1, ucValue);
    }
}

void writeInput(USHORT regAddr, SHORT sValue) {
    if( ( regAddr >= S_REG_INPUT_START )
    		&& ( regAddr < S_REG_INPUT_START + S_REG_INPUT_NREGS ) )
    {
    	usSRegInBuf[regAddr] = sValue;
    }
}

SHORT readHolding(USHORT regAddr) {
    if ((regAddr >= S_REG_HOLDING_START)
            && (regAddr < S_REG_HOLDING_START + S_REG_HOLDING_NREGS))
    {
    	return usSRegHoldBuf[regAddr];
    }
    return 0;
}

void writeHolding(USHORT regAddr, SHORT sValue) {
    if ((regAddr >= S_REG_HOLDING_START)
            && (regAddr < S_REG_HOLDING_START + S_REG_HOLDING_NREGS))
    {
    	usSRegHoldBuf[regAddr] = sValue;
    }
}
