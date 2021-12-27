/*
 * drivers/mmc/host/sdhci-npcm750.c
 *
 * Support of SDHCI platform devices for npcm750 soc family
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * Inspired by sdhci-pltfm.c
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/highmem.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include "sdhci-pltfm.h"
#include <linux/of.h>
#include <linux/dma-mapping.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/io.h>
#include <linux/clk.h>
//#include "sdhci.h"

#if 0
/* done in the npcmx50_module_init file */
#include "mach/module_init.h"
#endif

struct npcm750_sdhci {
	struct clk *clk;
	int card_int_gpio;
};

unsigned int npcm7xx_clk_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	return clk_get_rate(pltfm_host->clk);
}

/* sdhci ops */
static const struct sdhci_ops sdhci_pltfm_ops = {
	/*.get_max_clock = npcm7xx_clk_get_max_clock,
	.get_timeout_clock = npcm7xx_clk_get_max_clock,*/
	.set_clock = sdhci_set_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        sdhci_module_init                                                                      */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs HW init                                                          */
/*---------------------------------------------------------------------------------------------------------*/
static void sdhci_module_init(struct platform_device *pdev, struct npcm750_sdhci *sdhci)
{
    int sd_num = pdev->id;

    
    /*-----------------------------------------------------------------------------------------------------*/
    /* Enable SDIO                                                                                         */
    /*-----------------------------------------------------------------------------------------------------*/
	dev_dbg(&pdev->dev, "sdhci_module_init: mmc %d\n", sd_num);

	#if 0	
	npcmx50_sdhci_probe_mux(sd_num);
	#endif

    /*-----------------------------------------------------------------------------------------------------*/
    /* Enable SDIO clock, CLKEN/SDIO=1                                                                     */
    /*-----------------------------------------------------------------------------------------------------*/
    // replaced: CLK_ConfigureSDClock(sd_num);
#ifdef CONFIG_OF
    if (sdhci->clk)
        clk_prepare_enable(sdhci->clk);
#endif    

    /*-----------------------------------------------------------------------------------------------------*/
    /* SD Card Interface - Set/clear Software Reset Control Bit                                            */
    /*-----------------------------------------------------------------------------------------------------*/
	#if 0
    npcmx50_sdhci_probe_rst(sd_num);
	#endif

}


static int sdhci_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct resource *iomem;
	struct npcm750_sdhci *sdhci;
	struct device *dev;
	int ret;

#ifdef CONFIG_OF
	struct device_node *np = pdev->dev.of_node;
	
	pdev->id = of_alias_get_id(np, "emmc");
	if (pdev->id < 0)
		pdev->id = 0;
#endif

	dev = &pdev->dev;
	host = sdhci_alloc_host(dev, sizeof(*sdhci));
	if (IS_ERR(host)) {
		ret = PTR_ERR(host);
		dev_dbg(&pdev->dev, "cannot allocate memory for sdhci\n");
		goto err;
	}

	
#ifdef CONFIG_OF
	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we have dma capability bindings this can go away.
	 */
	ret = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
	{
		goto err_host;
	}
#endif

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->ioaddr = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(host->ioaddr)) {
		ret = PTR_ERR(host->ioaddr);
		dev_dbg(&pdev->dev, "unable to map iomem: %d\n", ret);
		goto err_host;
	}

	host->hw_name = "sdhci";
	host->ops = &sdhci_pltfm_ops;
	host->irq = platform_get_irq(pdev, 0);
	//host->quirks = SDHCI_QUIRK_BROKEN_DMA;

	//host->quirks |= SDHCI_QUIRK_NO_BUSY_IRQ;
 
    /* This host supports the Auto CMD12 */
    //host->quirks |= SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12;
 
    /* Samsung SoCs need BROKEN_ADMA_ZEROLEN_DESC */
    //host->quirks |= SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC;

	sdhci = sdhci_priv(host);

#ifdef CONFIG_OF
	sdhci->clk = devm_clk_get(&pdev->dev, NULL);

	if (IS_ERR(sdhci->clk))
	{
	    ret = PTR_ERR(sdhci->clk);
	    goto err_host;
	}

	dev_dbg(&pdev->dev, "\tsdhci %d clock is %ld\n", pdev->id, clk_get_rate(sdhci->clk));     
#endif   

	sdhci_module_init(pdev, sdhci);

	ret = mmc_of_parse(host->mmc);
	if (ret) {
	        dev_err(&pdev->dev, "parsing dt failed (%d)\n", ret);
	}
	
#if 0
{
	unsigned long gpio_irq_type;
	int val;

	val = gpio_get_value(sdhci->card_int_gpio);

	if (sdhci->card_int_gpio >= 0) {
		ret = mmc_gpio_request_cd(host->mmc, sdhci->card_int_gpio, 0);
		if (ret < 0) {
			dev_dbg(&pdev->dev,
				"failed to request card-detect gpio%d\n",
				sdhci->card_int_gpio);
			goto disable_clk;
		}
	}
}
#endif
       
    if (pdev->id == 0)  // mmc0 emmc
    {
	    if (readl(host->ioaddr + SDHCI_CAPABILITIES) & SDHCI_CAN_DO_8BIT)
	    {
		host->mmc->caps |= MMC_CAP_8_BIT_DATA; 
	    }
	    /* Controller uses Auto CMD12 command to stop the transfer */
	    //host->quirks |= SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12;
	    /* Controller has unreliable card detection */
	    //host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;
	    /* The system physically doesn't support 1.8v, even if the host does */
	    //host->quirks2 |= SDHCI_QUIRK2_NO_1_8_V;
	    /* Controller has an unusable DMA engine */
	    //host->quirks |= SDHCI_QUIRK_BROKEN_DMA;
	    /* preserve card power during suspend */
	    //host->mmc->pm_caps |= MMC_PM_KEEP_POWER;
    }
    else      // SD Card only
    {
	    host->quirks |= SDHCI_QUIRK_DELAY_AFTER_POWER;		
	    host->quirks2 = SDHCI_QUIRK2_NO_1_8_V | SDHCI_QUIRK2_STOP_WITH_TC;
    }
	
	host->quirks |= SDHCI_QUIRK_NO_LED;		
	
	ret = sdhci_add_host(host);
	if (ret) {
		dev_dbg(&pdev->dev, "error adding host\n");
		goto err_host;
	}

	platform_set_drvdata(pdev, host);

	return 0;

err_host:
	sdhci_free_host(host);
err:
	dev_err(&pdev->dev, "spear-sdhci probe failed: %d\n", ret);
	return ret;
}

static int sdhci_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	/*struct npcm750_sdhci *sdhci = sdhci_priv(host);*/
	int dead = 0;
	u32 scratch;

	scratch = readl(host->ioaddr + SDHCI_INT_STATUS);
	if (scratch == (u32)-1)
		dead = 1;

	sdhci_remove_host(host, dead);
	// clk_disable_unprepare(sdhci->clk); // warning: disabling clock wa not tested on SD.
	sdhci_free_host(host);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sdhci_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct npcm750_sdhci *sdhci = sdhci_priv(host);
	int ret;

	ret = sdhci_suspend_host(host);
	if (!ret)
	{
	    dev_err(dev, "sdhci suspend failed %d\n", ret);
		clk_disable_unprepare(sdhci->clk);
	}
	return ret;
}

static int sdhci_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct npcm750_sdhci *sdhci = sdhci_priv(host);
	int ret;
	
	ret = sdhci_resume_host(host);
	if (!ret)
	{
	    dev_err(dev, "sdhci resume failed %d\n", ret);
		clk_disable_unprepare(sdhci->clk);
    }

    clk_prepare_enable(sdhci->clk);

	return ret;
}
#endif

static const struct of_device_id sdhci_nuvoton_of_match[] = {
	{ .compatible = "nuvoton,npcm750-sdhci" },
	{ .compatible = "nuvoton,npcm845-sdhci" },
	{ }
};
MODULE_DEVICE_TABLE(of, sdhci_nuvoton_of_match);

static SIMPLE_DEV_PM_OPS(sdhci_pm_ops, sdhci_suspend, sdhci_resume);

static struct platform_driver sdhci_driver = {
	.driver = {
		.name	= "sdhci-npcm750",
		.of_match_table = sdhci_nuvoton_of_match,
		.owner	= THIS_MODULE,
		.pm	= &sdhci_pm_ops,
	},
	.probe		= sdhci_probe,
	.remove		= sdhci_remove,
};

module_platform_driver(sdhci_driver);

MODULE_DESCRIPTION("npcm750 Secure Digital Host Controller Interface driver");
MODULE_AUTHOR("Viresh Kumar <viresh.kumar@st.com>");
MODULE_LICENSE("GPL v2");

