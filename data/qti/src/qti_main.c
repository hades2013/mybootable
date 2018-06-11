/******************************************************************************

                        QTI_MAIN.C

******************************************************************************/

/******************************************************************************

  @file    qti_main.c
  @brief   Tethering Interface module

  DESCRIPTION
  Implementation of Tethering Interface module.

  ---------------------------------------------------------------------------
  Copyright (c) 2012-2014 Qualcomm Technologies, Inc.  All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.
  ---------------------------------------------------------------------------


******************************************************************************/


/******************************************************************************

                      EDIT HISTORY FOR FILE

  $Id:$

when       who        what, where, why
--------   ---        -------------------------------------------------------
01/22/14   sb         Added QTI for Fusion
12/9/13    sb         Add port mapper functionality. Modem interface file interaction
11/15/12   sb         Initial version

******************************************************************************/

/*===========================================================================
                              INCLUDE FILES
===========================================================================*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <signal.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/if.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include "qti_cmdq.h"
#include "qti.h"


/*===========================================================================
                              VARIABLE DECLARATIONS
===========================================================================*/

static qti_conf_t                qti_conf;
static qti_rmnet_param           rmnet_config_param;
static qti_dpl_param             dpl_config_param;
qcmap_sk_fd_set_info_t           sk_fdset;

/* Global Netlink Socket. variable */
qcmap_sk_info_t     sk_info;


/*===========================================================================
                              FUNCTION DEFINITIONS
===========================================================================*/

/*==========================================================================

FUNCTION QTI_PRINT_MSG()

DESCRIPTION

  This function is used to print QTI logs to a file

DEPENDENCIES
  None.

RETURN VALUE


SIDE EFFECTS
  None

==========================================================================*/

void qti_print_msg
(
  const char *funcName,
  uint32 lineNum,
  const char*fmt, ...
)
{
  va_list args;

  /*------------------------------------------------------------------------*/
  va_start (args, fmt);
  fprintf(fp_logs,"\n Func: %s Line: %d :", funcName, lineNum);
  vfprintf (fp_logs, fmt, args);
  va_end (args);
}

/*==========================================================================

FUNCTION MAIN()

DESCRIPTION

  The main function for QTI which is first called when QTI gets started on
  boot up.

DEPENDENCIES
  None.

RETURN VALUE
  0 on SUCCESS
  -1 on FAILURE

SIDE EFFECTS
  None

==========================================================================*/
int main(int argc, char ** argv)
{
  int                       ret_val,i;
  uint8_t                   embd_mode = FALSE;
  char command[QTI_MAX_COMMAND_STR_LEN];
  file_log_mode = FALSE;

/*------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------
 qti invoked in embedded mode, during this mode qti does not initialize rndis
 and ecm.
-------------------------------------------------------------------------*/

  for(i = 1; i < argc; i++)
  {
    if(0 == strncasecmp(argv[i],"em",strlen("em")))
    {
      system("echo QTI embd_mode > /dev/kmsg");
      embd_mode = TRUE;
    }

    if(0 == strncasecmp(argv[i],"-d",strlen("-d")))
    {
      file_log_mode = TRUE;

/*-------------------------------------------------------------------------
Delete the qti_logs.txt file if present
-------------------------------------------------------------------------*/
      snprintf(command, QTI_MAX_COMMAND_STR_LEN, "rm %s", QTI_LOG_FILE);
      ds_system_call(command, strlen(command));

      fp_logs = fopen( QTI_LOG_FILE, "w" );
      if(fp_logs == NULL)
      {
        printf(" Error in opening /data/qti_logs.txt\n");
        return QTI_FAILURE;
      }

      fprintf( fp_logs, "QTI LOGGING IN /data/qti_logs.txt \n" );

/*-------------------------------------------------------------------------
Fetch the file descriptor from file pointer
-------------------------------------------------------------------------*/
      qtilogfd = fileno(fp_logs);
      if( qtilogfd == QTI_FAILURE )
      {
        printf( " File /data/qti_logs.txt is not open, errno: %d\n",errno );
        fclose ( fp_logs );
        return QTI_FAILURE;
      }

      if(fcntl(qtilogfd, F_SETFD, FD_CLOEXEC) < 0)
      {
        printf( "Couldn't set Close on Exec, errno: %d", errno );
        fclose ( fp_logs );
        return QTI_FAILURE;
      }
    }
  }

/*-------------------------------------------------------------------------
  Initializing Diag for QXDM logs
-------------------------------------------------------------------------*/
  if (TRUE != Diag_LSM_Init(NULL))
  {
     printf("Diag_LSM_Init failed !!");
  }

  LOG_MSG_INFO1("Start QTI", 0, 0, 0);

/*-----------------------------------------------------------------------
    Initialize QTI variables
------------------------------------------------------------------------*/
  memset(&sk_fdset, 0, sizeof(qcmap_sk_fd_set_info_t));
  memset(&qti_conf, 0, sizeof(qti_conf_t));
  memset(&rmnet_config_param, 0, sizeof(qti_rmnet_param));
  memset(&dpl_config_param, 0, sizeof(qti_dpl_param));

/*-----------------------------------------------------------------------
    Identify target in use
------------------------------------------------------------------------*/
  rmnet_config_param.target = ds_get_target();

  if( DS_TARGET_FUSION4_5_PCIE == rmnet_config_param.target)
  {
    memcpy(rmnet_config_param.ph_iface[PH_DRIVER_TYPE_USB].ph_iface_device_file,
           RMNET_USB_DEV_FILE,
           strlen(RMNET_USB_DEV_FILE));
    memcpy(rmnet_config_param.ph_iface[PH_DRIVER_TYPE_USB].ph_data_iface_name,
           USB_DATA_INTERFACE,
           strlen(USB_DATA_INTERFACE));

    memcpy(rmnet_config_param.modem_iface_device_file,
           RMNET_MHI_DEV_FILE,
           strlen(RMNET_MHI_DEV_FILE));
    memcpy(rmnet_config_param.modem_data_iface_name,
           MHI_DATA_INTERFACE,
           strlen(MHI_DATA_INTERFACE));

    rmnet_config_param.qmux_conn_id = QMI_CONN_ID_RMNET_MHI_1;
  }
  else
  {
    memcpy(rmnet_config_param.ph_iface[PH_DRIVER_TYPE_USB].ph_iface_device_file,
           RMNET_USB_DEV_FILE,
           strlen(RMNET_USB_DEV_FILE));
    memcpy(rmnet_config_param.ph_iface[PH_DRIVER_TYPE_MHI].ph_iface_device_file,
           RMNET_MHI_PH_DEV_FILE,
           strlen(RMNET_MHI_PH_DEV_FILE));
    memcpy(rmnet_config_param.modem_iface_device_file,
           RMNET_SMD_DEV_FILE,
           strlen(RMNET_SMD_DEV_FILE));
    memcpy(dpl_config_param.dpl_iface_device_file,
           DPL_USB_DEV_FILE,
           strlen(DPL_USB_DEV_FILE));
    rmnet_config_param.qmux_conn_id = QMI_CONN_ID_RMNET_8;
  }
/*---------------------------------------------------------------------
  Initialize QTI peripheral interface
---------------------------------------------------------------------*/
  ret_val = qti_rmnet_ph_init(&rmnet_config_param,
                              &sk_fdset,
                              qti_rmnet_ph_recv_msg);
  if(ret_val != QTI_SUCCESS)
  {
    LOG_MSG_ERROR("Failed to initialize QTI peripheral interface",
                  0, 0, 0);
    return QTI_FAILURE;
  }


/*---------------------------------------------------------------------
  Initialize QTI modem interface
---------------------------------------------------------------------*/
   ret_val = qti_rmnet_modem_init(&rmnet_config_param,
                                  qti_rmnet_modem_recv_msg,
                                  &dpl_config_param);
  if(ret_val != QTI_SUCCESS)
  {
    LOG_MSG_ERROR("Failed to initialize QTI modem interface",
                  0, 0, 0);
    return QTI_FAILURE;
  }

/*---------------------------------------------------------------------
      Initialize QTI command queue
----------------------------------------------------------------------*/
  qti_cmdq_init();

/*---------------------------------------------------------------------
  Initialize DPM client
---------------------------------------------------------------------*/
  ret_val = qti_dpm_init(&rmnet_config_param,
                         &dpl_config_param);

  if(ret_val != QTI_SUCCESS)
  {
    LOG_MSG_ERROR("Failed to initialize DPM",
                  0, 0, 0);
    return QTI_FAILURE;
  }

/*---------------------------------------------------------------------
  Initialize QTI peripheral interface for DPL logging
---------------------------------------------------------------------*/
  ret_val = qti_dpl_ph_init(&dpl_config_param,
                            &sk_fdset,
                            qti_dpl_ph_recv_msg);
  if(ret_val != QTI_SUCCESS)
  {
    LOG_MSG_ERROR("Failed to initialize QTI peripheral interface for DPL logging",
                  0, 0, 0);
  }

#ifdef FEATURE_MDM_LE

  if( !embd_mode )
  {
    /*-----------------------------------------------------------------------
      Initialize QTI interfaces for netlink events
      ------------------------------------------------------------------------*/
    ret_val = qti_netlink_init(&qti_conf);

    if(ret_val != QTI_SUCCESS)
    {
      LOG_MSG_ERROR("Failed to initialize netlink interfaces for QTI",
          0, 0, 0);
      return QTI_FAILURE;
    }

    /*---------------------------------------------------------------------
      Call into the netlink listener init function which sets up QTI to
      listen to netlink events
      ---------------------------------------------------------------------*/
    if (qcmap_nl_listener_init(
                                 NETLINK_ROUTE,
                                 RTMGRP_LINK,
                                 &sk_fdset,
                                 qti_nl_recv_msg,
                                 &sk_info,
                                 MAX_NUM_OF_FD) == QCMAP_UTIL_SUCCESS)
    {
      LOG_MSG_INFO1("Open netlink socket succeeds",0,0,0);
    }
    else
    {
      LOG_MSG_ERROR("Netlink socket open failed",0,0,0);
      return QTI_FAILURE;
    }
    /*-----------------------------------------------------------------------
      Initialize QTI to be a client of QCMAP
      ------------------------------------------------------------------------*/
    ret_val = qti_qcmap_init(&qti_conf);
    if(ret_val != QTI_SUCCESS)
    {
      LOG_MSG_ERROR("Failed to post qcmap init command to cmd_queue",
          0, 0, 0);
      return QTI_FAILURE;
    }
  }

#endif //FEATURE_MDM_LE

  if( (DS_TARGET_FUSION4_5_PCIE == rmnet_config_param.target) ||
      (DS_TARGET_JOLOKIA == rmnet_config_param.target) )
  {
    ret_val = qti_rmnet_qmi_init(&rmnet_config_param);
    if(ret_val != QTI_SUCCESS)
    {
      LOG_MSG_ERROR("Failed to initialize QMI message handler",
            0, 0, 0);
      return QTI_FAILURE;
    }
  }


/*--------------------------------------------------------------------
  Start the listener which listens to netlink events and QMI packets
  coming on USB-Rmnet device file
---------------------------------------------------------------------*/
/*--------------------------------------------------------------------------
  Update QTI state
---------------------------------------------------------------------------*/
  qti_conf.state = QTI_LINK_UP_WAIT;

  ret_val = qcmap_listener_start(&sk_fdset);

  if(ret_val != QCMAP_UTIL_SUCCESS)
  {
    LOG_MSG_ERROR("Failed to start NL listener",
                   0, 0, 0);
  }
/*--------------------------------------------------------------------
  Wait for the QTI command queue to finish before exiting QTI
---------------------------------------------------------------------*/
#ifdef FEATURE_MDM_LE

  if(!embd_mode)
  {
    qti_qcmap_exit();
  }
#endif //FEATURE_MDM_LE

  qti_cmdq_wait();


  fclose(fp_logs);

  return QTI_SUCCESS;
}


