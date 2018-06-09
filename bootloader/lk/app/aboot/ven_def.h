/*************
 *
 * $Id$
 *
 * Filename:  ven_def.h
 *
 * Copyright: ?2012 wewins Wireless, Inc.
 *            All rights reserved
 *
 **************/
#ifndef __VEN_DEF_H__
#define __VEN_DEF_H__

#define private

#ifndef BYTE
typedef unsigned char       BYTE,  byte;
typedef unsigned short      WORD,  word;
typedef unsigned long       DWORD,  dword;
#endif

#ifndef UINT32
typedef unsigned char	      UINT8,  uint8;
typedef unsigned short	      UINT16, uint16;
typedef unsigned int	      UINT,   uint;
//typedef unsigned int          size_t;


//typedef unsigned long         UINT32, uint32;
#endif

#ifndef INT32
typedef signed char         int8,   INT8;
typedef signed short        int16,  INT16;
//typedef signed long         int32,  INT32;
#endif

/*#ifndef BOOL
typedef unsigned char       BOOL;
#define TRUE                1
#define FALSE               0

#define bool                BOOL
#define true                1
#define false               0
#endif*/

#ifndef NULL
#define	NULL	0
#endif

#endif
