/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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

#include <stdint.h>
#include <debug.h>
#include <reg.h>
#include <mmc.h>
#include <clock.h>
#include <platform/timer.h>
#include <platform/clock.h>
#include <platform/iomap.h>
#include <pm8x41.h>

void clock_init_mmc(uint32_t interface)
{
	char clk_name[64];
	int ret;

	snprintf(clk_name, sizeof(clk_name), "sdc%u_iface_clk", interface);

	/* enable interface clock */
	ret = clk_get_set_enable(clk_name, 0, true);
	if(ret)
	{
		dprintf(CRITICAL, "failed to set sdc%u_iface_clk ret = %d\n", interface, ret);
		ASSERT(0);
	}
}

/* Configure MMC clock */
void clock_config_mmc(uint32_t interface, uint32_t freq)
{
	int ret = 0;
	char clk_name[64];

	snprintf(clk_name, sizeof(clk_name), "sdc%u_core_clk", interface);

	if(freq == MMC_CLK_400KHZ)
	{
		ret = clk_get_set_enable(clk_name, 400000, true);
	}
	else if(freq == MMC_CLK_50MHZ)
	{
		ret = clk_get_set_enable(clk_name, 50000000, true);
	}
	else if(freq == MMC_CLK_96MHZ)
	{
		ret = clk_get_set_enable(clk_name, 100000000, true);
	}
	else if(freq == MMC_CLK_192MHZ)
	{
		ret = clk_get_set_enable(clk_name, 192000000, true);
	}
	else
	{
		dprintf(CRITICAL, "sdc frequency (%u) is not supported\n", freq);
		ASSERT(0);
	}

	if(ret)
	{
		dprintf(CRITICAL, "failed to set sdc%u_core_clk ret = %d\n", interface, ret);
		ASSERT(0);
	}
}

/* Configure UART clock based on the UART block id*/
void clock_config_uart_dm(uint8_t id)
{
	int ret;
	char iclk[64];
	char cclk[64];

	snprintf(iclk, sizeof(iclk), "uart%u_iface_clk", id);
	snprintf(cclk, sizeof(cclk), "uart%u_core_clk", id);

	ret = clk_get_set_enable(iclk, 0, true);
	if(ret)
	{
		dprintf(CRITICAL, "failed to set uart%u_iface_clk ret = %d\n", id, ret);
		ASSERT(0);
	}

	ret = clk_get_set_enable(cclk, 7372800, true);
	if(ret)
	{
		dprintf(CRITICAL, "failed to set uart%u_core_clk ret = %d\n", id, ret);
		ASSERT(0);
	}
}

/* Function to asynchronously reset CE (Crypto Engine).
 * Function assumes that all the CE clocks are off.
 */
static void ce_async_reset(uint8_t instance)
{
}

void clock_ce_enable(uint8_t instance)
{
}

void clock_ce_disable(uint8_t instance)
{
}

void clock_config_ce(uint8_t instance)
{
	/* Need to enable the clock before disabling since the clk_disable()
	 * has a check to default to nop when the clk_enable() is not called
	 * on that particular clock.
	 */
	clock_ce_enable(instance);

	clock_ce_disable(instance);

	ce_async_reset(instance);

	clock_ce_enable(instance);

}

void clock_usb30_gdsc_enable(void)
{
	uint32_t reg = readl(GCC_USB30_GDSCR);

	reg &= ~(0x1);

	writel(reg, GCC_USB30_GDSCR);
}

/* enables usb30 clocks */
void clock_usb30_init(void)
{
	int ret;

	ret = clk_get_set_enable("usb30_iface_clk", 0, true);
	if(ret)
	{
		dprintf(CRITICAL, "failed to set usb30_iface_clk. ret = %d\n", ret);
		ASSERT(0);
	}

	clock_usb30_gdsc_enable();

	ret = clk_get_set_enable("usb30_master_clk", 125000000, true);
	if(ret)
	{
		dprintf(CRITICAL, "failed to set usb30_master_clk. ret = %d\n", ret);
		ASSERT(0);
	}

	ret = clk_get_set_enable("usb30_phy_aux_clk", 1200000, true);
	if(ret)
	{
		dprintf(CRITICAL, "failed to set usb30_phy_aux_clk. ret = %d\n", ret);
		ASSERT(0);
	}

	ret = clk_get_set_enable("usb30_mock_utmi_clk", 60000000, true);
	if(ret)
	{
		dprintf(CRITICAL, "failed to set usb30_mock_utmi_clk ret = %d\n", ret);
		ASSERT(0);
	}

	ret = clk_get_set_enable("usb30_sleep_clk", 0, true);
	if(ret)
	{
		dprintf(CRITICAL, "failed to set usb30_sleep_clk ret = %d\n", ret);
		ASSERT(0);
	}

	ret = clk_get_set_enable("usb_phy_cfg_ahb2phy_clk", 0, true);
	if(ret)
	{
		dprintf(CRITICAL, "failed to enable usb_phy_cfg_ahb2phy_clk = %d\n", ret);
		ASSERT(0);
	}
}

void clock_bumpup_pipe3_clk()
{
	int ret = 0;

	ret = clk_get_set_enable("usb30_pipe_clk", 0, true);
	if(ret)
	{
		dprintf(CRITICAL, "failed to set usb30_pipe_clk. ret = %d\n", ret);
		ASSERT(0);
	}

	return;
}

void clock_reset_usb_phy()
{
	int ret;

	struct clk *phy_reset_clk = NULL;
	struct clk *pipe_reset_clk = NULL;

	/* Look if phy com clock is present */
	phy_reset_clk = clk_get("usb30_phy_reset");
	ASSERT(phy_reset_clk);

	pipe_reset_clk = clk_get("usb30_pipe_clk");
	ASSERT(pipe_reset_clk);

	/* ASSERT */
	ret = clk_reset(phy_reset_clk, CLK_RESET_ASSERT);

	if (ret)
	{
		dprintf(CRITICAL, "Failed to assert usb30_phy_reset clk\n");
		return;
	}

	ret = clk_reset(pipe_reset_clk, CLK_RESET_ASSERT);
	if (ret)
	{
		dprintf(CRITICAL, "Failed to assert usb30_pipe_clk\n");
		goto deassert_phy_clk;
	}

	udelay(100);

	/* DEASSERT */
	ret = clk_reset(pipe_reset_clk, CLK_RESET_DEASSERT);
	if (ret)
	{
		dprintf(CRITICAL, "Failed to deassert usb_pipe_clk\n");
		return;
	}

deassert_phy_clk:

	ret = clk_reset(phy_reset_clk, CLK_RESET_DEASSERT);
	if (ret)
	{
		dprintf(CRITICAL, "Failed to deassert usb30_phy_com_reset clk\n");
		return;
	}
}
