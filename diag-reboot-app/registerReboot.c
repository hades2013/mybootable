/*
 * Copyright (c) 2012 Qualcomm Technologies, Inc. All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 *
 */

#include "msg.h"
#include "diag_lsm.h"
#include "stdio.h"
#include "diagpkt.h"
#include "string.h"
#include <unistd.h>
#include <stdlib.h>
#include <qmi.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#define DEFAULT_SMD_PORT QMI_PORT_RMNET_0
#define LOGI(...) fprintf(stderr, "I:" __VA_ARGS__)
#define DIAG_CONTROL_F      41

/* string to send to reboot_daemon to reboot */
#define REBOOT_STR "REBOOT"
/* size to make write buffer */
#define MAX_BUF 64
/* name of pipe to write to */
#define FIFO_NAME "/dev/rebooterdev"

/* ID's for diag */
#define DIAG_UID 53
#define DIAG_GID 53
#define REBOOTERS_GID 1301
#define SDCARD_GID 1015

//enable the line below if you want to turn on debug messages
//#define DIAG_REBOOT_DEBUG

PACK(void *)le_diag_handler(PACK(void *)req_pkt, uint16 pkt_len);

static const diagpkt_user_table_entry_type le_table_1[] =
    {{DIAG_CONTROL_F, DIAG_CONTROL_F, le_diag_handler}};

void drop_privileges() {

    /* Start as root */
    /* Update primary group */
    setgid(DIAG_GID);

    /* Update secondary groups */
    gid_t *groups;
    int numGroups = 2;
    groups = (gid_t *) malloc(numGroups * sizeof(gid_t));
    if (groups){
        groups[0] = REBOOTERS_GID;
        groups[1] = SDCARD_GID;
    }
    setgroups(numGroups, groups);

    /* Update UID -- from root to diag */
    setuid(DIAG_UID);

    free(groups);
}

int main (void) {

    /* Drop privileges from root to diag after updating groups */
    drop_privileges();

    boolean bInit_Success = FALSE;
    bInit_Success = Diag_LSM_Init(NULL);

    if (!bInit_Success) {
        printf("Diag_LSM_Init() failed.\n");
        return -1;
    }

    DIAGPKT_DISPATCH_TABLE_REGISTER(DIAGPKT_NO_SUBSYS_ID, le_table_1);

    do {
        sleep (1);
    } while (1);

    Diag_LSM_DeInit();

    return 0;

}

void call_reboot_daemon() {

    int pipe_fd = -1;
    char buf[MAX_BUF];
    int write_count = -1;

    /* Open connection to file reboot-daemon.c. */
    pipe_fd = open(FIFO_NAME, O_WRONLY);
    if (pipe_fd == -1) {
        LOGI("open() failed: %s\n", strerror(errno));
        return;
    }

    /* Write REBOOT_STR to the file. */
    memset(buf, 0, MAX_BUF);
    strncpy(buf, REBOOT_STR, strlen(REBOOT_STR));
    strcat(buf, "\0");

    write_count = write(pipe_fd, buf, strlen(buf));
    if (write_count < strlen(buf)) {
        LOGI("write() failed: %s\n", strerror(errno));
        return;
    }

    /* Close connection to the file. */
    LOGI("Closing FIFO\n");
    close(pipe_fd);
}

PACK(void *) le_diag_handler(PACK(void *)req_pkt, uint16 pkt_len) {
#ifdef DIAG_REBOOT_DEBUG
    printf("\n le_diag_handler called!\n");
#endif
    void *rsp_pkt = diagpkt_alloc(DIAG_CONTROL_F, pkt_len);
    if (rsp_pkt) {
        memcpy(rsp_pkt, req_pkt, pkt_len);
    }

    pthread_t modem_thread;
    pthread_attr_t thrd_attr;

    (void)pthread_attr_init( &thrd_attr );
    (void)pthread_attr_setdetachstate( &thrd_attr, PTHREAD_CREATE_DETACHED);

#ifdef DIAG_REBOOT_DEBUG
    printf("about to call reboot\n");
#endif

    call_reboot_daemon();
    return rsp_pkt;
}
