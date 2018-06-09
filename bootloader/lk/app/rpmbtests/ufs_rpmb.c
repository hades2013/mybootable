/* Copyright (c) 2014 The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <debug.h>
#include <reg.h>
#include <ufs_hw.h>
#include <utp.h>
#include <upiu.h>
#include <uic.h>
#include <ucs.h>
#include <dme.h>
#include <rpmb.h>
#include <rand.h>
#include <string.h>
#include <stdlib.h>
#include <endian.h>
#include <target.h>
#include "ufs_rpmb.h"

void rpmb_run_test()
{
	bool result;
	uint16_t sector_address = 1, rpmb_num_blocks = 4;
	void *dev;
	dev = target_mmc_device();
	result = rpmb_test((struct ufs_dev *)dev, sector_address, rpmb_num_blocks);
	if (!result)
		dprintf(INFO, "RPMB test failed");
}

int verify_rpmb_frame(struct rpmb_frame *request_frame, struct rpmb_frame *result_frame, int type)
{
	int i = 0;
	if(result_frame->result[0] == MAXED_WR_COUNTER)
	{
		dprintf(INFO, "Max write counter value reached\n");
		return MAXED_WR_COUNTER;
	}
	if(result_frame->result[1] != OPERATION_OK)
	{
		dprintf(INFO, "RPMB operation error: 0x%x\n", result_frame->result[1]);
		return result_frame->result[1];
	}
	for(i = 0; i < 16; i++)
	{
		if(request_frame->nonce[i] != result_frame->nonce[i])
			return NONCE_MISMATCH;
	}
	return OPERATION_OK;
}

void dump_rpmb_data(struct rpmb_frame *result_frame)
{
	int data_counter;
	for (data_counter = 0; data_counter < 256; data_counter++)
	{
		printf("0x%x ", result_frame->data[data_counter]);
		if ((data_counter + 1) % 16 == 0)
			printf("\n");
	}
	printf("\n");
}

bool rpmb_test(struct ufs_dev *dev, uint16_t address, uint16_t rpmb_num_blocks)
{
	struct rpmb_frame data_frame, result_frame[rpmb_num_blocks];
	int i = 0, ret;
	uint32_t response_len = 0;
	// check if address + sectors requested for read do not exceed total size of rpmb
	if ((address + rpmb_num_blocks) > dev->rpmb_num_blocks)
	{
		dprintf(CRITICAL, "Invalid request to rpmb\n");
		return false;
	}
	memset(&data_frame, 0, sizeof(data_frame));
	memset(&result_frame, 0, sizeof(result_frame));
	data_frame.address = BE16(address);
	data_frame.blockcount = BE16(rpmb_num_blocks);
	data_frame.requestresponse = BE16(AUTH_READ);
	for(i = 0 ; i < 16; i++)
	{
		data_frame.nonce[i] = (rand() % 256) + 1;
	}
#ifdef DEBUG_UFS_RPMB
	dprintf(INFO, "Dumping RPMB Request frame\n");
	dprintf(INFO, "--------------------------\n");
	dump((void *) &data_frame, sizeof(struct rpmb_frame));
#endif
	ret = ucs_do_scsi_rpmb_read(dev, (uint32_t *) &data_frame, rpmb_num_blocks,
                               (uint32_t *) &result_frame, &response_len);
	if (ret)
	{
		dprintf(CRITICAL, "RPMB Read error\n");
		return false;
	}
	for (i = 0; i < rpmb_num_blocks; i++)
	{
		ret = verify_rpmb_frame(&data_frame, &result_frame[i], AUTH_READ);
		if(ret)
		{
			dprintf(CRITICAL, "Error in verifying RPMB frame\n");
			dump((void *) &result_frame[i], sizeof(struct rpmb_frame));
		}
	}
#ifdef DEBUG_UFS_RPMB
	dprintf(INFO, "Dumping RPMB Response frames\n");
	dprintf(INFO, "----------------------------\n");
	dump((void *) &result_frame, sizeof(struct rpmb_frame)*rpmb_num_blocks);
#endif
	dprintf(INFO, "Data dump for RPMB read request\n");
	printf("-------------------------------\n");

	for (i = 0; i < rpmb_num_blocks; i++)
		dump_rpmb_data((void *) &result_frame[i]);

	printf("-------------------------------\n");

	return true;
}
