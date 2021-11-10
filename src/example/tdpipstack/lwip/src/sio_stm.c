/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 */

#include <lwip/include/opt.h>
#include <lwip/include/sys.h>
#include <lwip/include/sio.h>

#include <stdio.h>
#include <stdarg.h>

//#include <windows.h>

/** If SIO_USE_COMPORT==1, use COMx, if 0, use a pipe (default) */
#if SIO_USE_COMPORT
#define SIO_DEVICENAME "\\\\.\\COM"
#else
#define SIO_DEVICENAME "\\\\.\\pipe\\lwip"
#endif

static int sio_abort=0;

/**
 * SIO_DEBUG: Enable debugging for SIO.
 */
#ifndef SIO_DEBUG
#define SIO_DEBUG    LWIP_DBG_OFF
#endif
/*
#if SIO_USE_COMPORT
//When using a real COM port, set up the * serial line settings (baudrate etc.)
static BOOL
sio_setup(HANDLE fd)
{
  COMMTIMEOUTS cto;
  DCB dcb;
  memset(&cto, 0, sizeof(cto));

  if(!GetCommTimeouts(fd, &cto))
  {
    return FALSE;
  }
  // change read timeout, leave write timeout as it is
  cto.ReadIntervalTimeout = 1;
  cto.ReadTotalTimeoutMultiplier = 0;
  cto.ReadTotalTimeoutConstant = 100; // 10 ms
  if(!SetCommTimeouts(fd, &cto)) {
    return FALSE;
  }

  // set up baudrate and other communication settings
  memset(&dcb, 0, sizeof(dcb));
  // Obtain the DCB structure for the device
  if (!GetCommState(fd, &dcb)) {
    return FALSE;
  }

  // Set the new data
  dcb.BaudRate = 115200;
  dcb.ByteSize = 8;
  dcb.StopBits = 0; // ONESTOPBIT
  dcb.Parity   = 0; // NOPARITY
  dcb.fParity  = 0; // parity is not used
  // do not use flow control
  dcb.fOutxDsrFlow = dcb.fDtrControl = 0;
  dcb.fOutxCtsFlow = dcb.fRtsControl = 0;
  dcb.fErrorChar = dcb.fNull = 0;
  dcb.fInX = dcb.fOutX = 0;
  dcb.XonChar = dcb.XoffChar = 0;
  dcb.XonLim = dcb.XoffLim = 100;

  // Set the new DCB structure
  if (!SetCommState(fd, &dcb)) {
    return FALSE;
  }
  return TRUE;
}
#endif
*/
/// Opens a serial device for communication.
sio_fd_t sio_open(u8_t devnum)
{
}
/*
  //HANDLE fileHandle = INVALID_HANDLE_VALUE;
  //CHAR   fileName[256];
  LWIP_DEBUGF(SIO_DEBUG, ("sio_open(%lu)\n", (DWORD)devnum));
  //_snprintf(fileName, 255, SIO_DEVICENAME"%lu", (DWORD)(devnum));
  //fileHandle = CreateFile(fileName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
  //if (fileHandle != INVALID_HANDLE_VALUE) {
    sio_abort = 0;
    FlushFileBuffers(fileHandle);
#if SIO_USE_COMPORT
    if(!sio_setup(fileHandle)) {
      LWIP_DEBUGF(SIO_DEBUG, ("sio_open(%lu): sio_setup. GetLastError() returns %d\n",
                  (DWORD)devnum, GetLastError()));
      CloseHandle(fileHandle);
      return NULL;
    }
#endif
    LWIP_DEBUGF(SIO_DEBUG, ("sio_open: file \"%s\" successfully opened.\n", fileName));
    return (sio_fd_t)(fileHandle);
  }
  LWIP_DEBUGF(SIO_DEBUG, ("sio_open(%lu) failed. GetLastError() returns %d\n",
              (DWORD)devnum, GetLastError()));
  return NULL;
}

// Sends a single character to the serial device.
 // @note This function will block until the character can be sent.
/*
void sio_send(u8_t c, sio_fd_t fd)
{
  //DWORD dwNbBytesWritten = 0;
  LWIP_DEBUGF(SIO_DEBUG, ("sio_send(%lu)\n", (DWORD)c));
  //while ((!WriteFile((HANDLE)(fd), &c, 1, &dwNbBytesWritten, NULL)) || (dwNbBytesWritten < 1));
  return;
}

/// Receives a single character from the serial device.
 //note This function will block until a character is received.

u8_t sio_recv(sio_fd_t fd)
{
  //DWORD dwNbBytesReadden = 0;
  u8_t byte = 0;
  LWIP_DEBUGF(SIO_DEBUG, ("sio_recv()\n"));
  //while ((sio_abort == 0) && ((!ReadFile((HANDLE)(fd), &byte, 1, &dwNbBytesReadden, NULL)) || (dwNbBytesReadden < 1)));
  LWIP_DEBUGF(SIO_DEBUG, ("sio_recv()=%lu\n", (DWORD)byte));
  return byte;
}

// Reads from the serial device.
// * @param data pointer to data buffer for receiving
 // @param len maximum length (in bytes) of data to receive
 // @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 // * @note This function will block until data can be received. The blocking* can be cancelled by calling sio_read_abort().

u32_t sio_read(sio_fd_t fd, u8_t* data, u32_t len)
{
  BOOL ret;
  DWORD dwNbBytesReadden = 0;
  LWIP_DEBUGF(SIO_DEBUG, ("sio_read()...\n"));
  ret = ReadFile((HANDLE)(fd), data, len, &dwNbBytesReadden, NULL);
  LWIP_DEBUGF(SIO_DEBUG, ("sio_read()=%lu bytes -> \n", dwNbBytesReadden, ret));
  return dwNbBytesReadden;
}

// Writes to the serial device.
//@param data pointer to data to send
// @param len length (in bytes) of data to send
//@return number of bytes actually sent
 //@note This function will block until all data can be sent.

u32_t sio_write(sio_fd_t fd, u8_t* data, u32_t len)
{
  BOOL ret;
  DWORD dwNbBytesWritten = 0;
  LWIP_DEBUGF(SIO_DEBUG, ("sio_write()...\n"));
  ret = WriteFile((HANDLE)(fd), data, len, &dwNbBytesWritten, NULL);
  LWIP_DEBUGF(SIO_DEBUG, ("sio_write()=%lu bytes -> %d\n", dwNbBytesWritten, ret));
  return dwNbBytesWritten;
}
// Aborts a blocking sio_read() call. @todo: This currently ignores fd and aborts all reads
 oid sio_read_abort(sio_fd_t fd)
{
  LWIP_UNUSED_ARG(fd);
  LWIP_DEBUGF(SIO_DEBUG, ("sio_read_abort() !!!!!...\n"));
  sio_abort = 1;
  return;
}
 */

/*
 * //==========================================================================
//
//      sio.c
//
//      Serial operations for SLIP, PPP etc.
//
//==========================================================================
//####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 2008, 2009 Free Software Foundation
//
// eCos is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 or (at your option) any later version.
//
// eCos is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with eCos; if not, write to the Free Software Foundation, Inc.,
// 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
//
// As a special exception, if other files instantiate templates or use macros
// or inline functions from this file, or you compile this file and link it
// with other works to produce a work based on this file, this file does not
// by itself cause the resulting work to be covered by the GNU General Public
// License. However the source code for this file must still be made available
// in accordance with section (3) of the GNU General Public License.
//
// This exception does not invalidate any other reasons why a work based on
// this file might be covered by the GNU General Public License.
// -------------------------------------------
//####ECOSGPLCOPYRIGHTEND####
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    Simon Kallweit
// Contributors:
// Date:         2008-12-01
// Purpose:
// Description:  Serial operations for SLIP, PPP etc.
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/net_lwip.h>

#include "lwip.h"
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/netif.h"
#include "lwip/sio.h"

#include <cyg/error/codes.h>
#include <cyg/io/io.h>
#include <cyg/io/serialio.h>
#include <cyg/io/config_keys.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>

/*
sio_fd_t sio_open(u8_t devnum)
{
    Cyg_ErrNo ret;
    cyg_io_handle_t handle;
    char *dev;

#ifdef CYGDBG_LWIP_DEBUG_SIO
    diag_printf("sio_open(devnum=%d)\n", devnum);
#endif

    switch (devnum) {
#ifdef CYGPKG_LWIP_SLIP
    case SIO_DEV_SLIPIF:
        dev = CYGDAT_LWIP_SLIPIF_DEV;
        break;
#endif
#ifdef CYGFUN_LWIP_PPPOS_SUPPORT
    case SIO_DEV_PPPOS:
        dev = CYGDAT_LWIP_PPPOS_DEV;
        break;
#endif
    default:
        // Unknown serial io device
        return NULL;
        break;
    }

    ret = cyg_io_lookup(dev, &handle);
    if (ret != ENOERR)
        return NULL;

    return handle;
}


void sio_send(u8_t c, sio_fd_t fd)
{
    cyg_uint32 len = 1;

#ifdef CYGDBG_LWIP_DEBUG_SIO
    diag_printf("sio_send(c=0x%02x,fd=%p)\n", c, fd);
#endif

	cyg_io_write((cyg_io_handle_t) fd, &c, &len);
}


u8_t sio_recv(sio_fd_t fd)
{
    cyg_uint32 len = 1;
    char c;

#ifdef CYGDBG_LWIP_DEBUG_SIO
    diag_printf("sio_recv(fd=%p)\n", fd);
#endif

	cyg_io_read((cyg_io_handle_t) fd, &c, &len);

#ifdef CYGDBG_LWIP_DEBUG_SIO
    diag_printf("sio_recv: %02X\n", (cyg_uint8) c);
#endif

	return c;
}

// Reads from the serial device.

u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
    Cyg_ErrNo ret;

#ifdef CYGDBG_LWIP_DEBUG_SIO
    diag_printf("sio_read(fd=%p,data=%p,len=%lu:)\n", fd, data, len);
#endif

    ret = cyg_io_read((cyg_io_handle_t) fd, data, (cyg_uint32 *) &len);
    if (ret != ENOERR)
        return 0;

#ifdef CYGDBG_LWIP_DEBUG_SIO
    diag_printf("sio_read: ");
    diag_dump_buf(data, len);
#endif

    return len;
}

// * Tries to read from the serial device. Same as sio_read but returns immediately if no data is available and never blocks.
 u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
    Cyg_ErrNo ret;
    cyg_serial_buf_info_t info;
    cyg_uint32 l;

#ifdef CYGDBG_LWIP_DEBUG_SIO
    diag_printf("sio_tryread(fd=%p,data=%p,len=%lu:)\n", fd, data, len);
#endif

    // Check how many bytes there are to read
    l = sizeof(info);
    ret = cyg_io_get_config((cyg_io_handle_t) fd, CYG_IO_GET_CONFIG_SERIAL_BUFFER_INFO, &info, &l);
    if (ret != ENOERR)
        return 0;
    l = info.rx_count;
    if (l < 1)
        return 0;
    if (l > len)
        l = len;

    ret = cyg_io_read((cyg_io_handle_t) fd, data, &l);
    if (ret != ENOERR)
        return 0;

#ifdef CYGDBG_LWIP_DEBUG_SIO
   diag_printf("sio_tryread: ");
   diag_dump_buf(data, len);
#endif

   return l;
}


//Writes to the serial device.

u32_t sio_write(sio_fd_t fd, u8_t *data, u32_t len)
{
    Cyg_ErrNo ret;
    cyg_uint32 count = 0;
    cyg_uint32 chunk;

#ifdef CYGDBG_LWIP_DEBUG_SIO
    diag_printf("sio_write(fd=%p,data=%p,len=%lu:)\n", fd, data, len);
    diag_printf("sio_write: ");
    diag_dump_buf(data, len);
#endif

    while (count < len) {
        chunk = len - count;
        ret = cyg_io_write((cyg_io_handle_t) fd, data, &chunk);
        if (ret != ENOERR)
            break;
        data += chunk;
        count += chunk;
    }

	return count;
}

// Aborts a blocking sio_read() call.

void sio_read_abort(sio_fd_t fd)
{
    cyg_uint32 l = 0;

#ifdef CYGDBG_LWIP_DEBUG_SIO
    diag_printf("sio_read_abort(fd=%p)\n", fd);
#endif

   cyg_io_set_config((cyg_io_handle_t) fd, CYG_IO_GET_CONFIG_SERIAL_ABORT, NULL, &l);
}
*/

void UART1StdioInit(unsigned long ulPortNum);



/*  typedef struct siostruct_t */
/*  {  */
/*  	sio_status_t *sio; */
/*  } siostruct_t; */

/** array of ((siostruct*)netif->state)->sio structs */
static sio_status_t statusar[3];

/**
* Initiation of serial device
* @param device : string with the device name and path, eg. "/dev/ttyS0"
* @param netif  : netinterface struct, contains interface instance data
* @return file handle to serial dev.
*/
static int sio_init( char * device, int devnum, sio_status_t * siostat )
{
	UART1StdioInit(1);
}
