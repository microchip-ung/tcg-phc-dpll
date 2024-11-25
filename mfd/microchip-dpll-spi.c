// SPDX-License-Identifier: GPL-2.0+

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/of_platform.h>
#include <linux/mfd/microchip-dpll.h>

#define MICROCHIP_DPLL_PAGE_ADDR		0x007F
#define MICROCHIP_DPLL_HIGHER_ADDR_MASK		0xFF80
#define MICROCHIP_DPLL_LOWER_ADDR_MASK		0x007F

static const struct spi_device_id microchip_dpll_spi_id[] = {
	{ "zl80732-spi",  },
	{}
};
MODULE_DEVICE_TABLE(spi, microchip_dpll_spi_id);

static const struct of_device_id microchip_dpll_spi_of_match[] = {
	{ .compatible = "microchip,zl80732-spi" },
	{}
};
MODULE_DEVICE_TABLE(of, microchip_dpll_spi_of_match);

static int microchip_dpll_read_device(struct microchip_dpll_ddata *dpll,
				      u8 reg, u8 *buf, u16 bytes)
{
	struct spi_device *client = to_spi_device(dpll->dev);
	struct spi_transfer xfer = {0};
	struct spi_message msg;
	u8 cmd[256] = {0};
	u8 rsp[256] = {0};
	int ret;

	cmd[0] = reg | 0x80;
	xfer.rx_buf = rsp;
	xfer.len = bytes + 1;
	xfer.tx_buf = cmd;
	xfer.bits_per_word = client->bits_per_word;
	xfer.speed_hz = client->max_speed_hz;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(client, &msg);
	if (ret >= 0)
		memcpy(buf, &rsp[1], xfer.len-1);

	return ret;
}

static int microchip_dpll_write_device(struct microchip_dpll_ddata *dpll,
				       u8 reg, u8 *buf, u16 bytes)
{
	struct spi_device *client = to_spi_device(dpll->dev);
	struct spi_transfer xfer = {0};
	struct spi_message msg;
	u8 cmd[256] = {0};

	cmd[0] = reg;
	memcpy(&cmd[1], buf, bytes);

	xfer.len = bytes + 1;
	xfer.tx_buf = cmd;
	xfer.bits_per_word = client->bits_per_word;
	xfer.speed_hz = client->max_speed_hz;
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	return spi_sync(client, &msg);
}

static int microchip_dpll_write_page_register(struct microchip_dpll_ddata *dpll,
					      u16 reg)
{
	u8 page_reg;
	u16 bytes;
	u16 page;
	int err;
	u8 buf;

	page_reg = MICROCHIP_DPLL_PAGE_ADDR;
	page = reg & MICROCHIP_DPLL_HIGHER_ADDR_MASK;
	buf = (u8)((page >> 7) & 0xff);
	bytes = 1;

	/* Simply return if we are on the same page */
	if (dpll->page == page)
		return 0;

	err = microchip_dpll_write_device(dpll, page_reg, &buf, bytes);
	if (err)
		dev_err(dpll->dev, "Failed to set page offset 0x%x\n", page);
	else
		/* Remember the last page */
		dpll->page = page;

	return err;
}

static int microchip_dpll_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct microchip_dpll_ddata *dpll = spi_get_drvdata((struct spi_device *)context);
	u8 addr = (u8)(reg & MICROCHIP_DPLL_LOWER_ADDR_MASK);
	int err;

	err = microchip_dpll_write_page_register(dpll, reg);
	if (err)
		return err;

	err = microchip_dpll_read_device(dpll, addr, (u8 *)val, 1);
	if (err)
		dev_err(dpll->dev,
			"Failed to read offset address 0x%x\n", addr);

	return err;
}

static int microchip_dpll_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct microchip_dpll_ddata *dpll = spi_get_drvdata((struct spi_device *)context);
	u8 addr = (u8)(reg & MICROCHIP_DPLL_LOWER_ADDR_MASK);
	u8 data = (u8)val;
	int err;

	err = microchip_dpll_write_page_register(dpll, reg);
	if (err)
		return err;

	err = microchip_dpll_write_device(dpll, addr, &data, 1);
	if (err)
		dev_err(dpll->dev,
			"Failed to write offset address 0x%x\n", addr);

	return err;
}

static const struct regmap_config microchip_dpll_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x0780,
	.reg_read = microchip_dpll_reg_read,
	.reg_write = microchip_dpll_reg_write,
	.cache_type = REGCACHE_NONE,
};

static int microchip_dpll_spi_probe(struct spi_device *client)
{
	struct microchip_dpll_ddata *dpll;
	int ret;

	dpll = devm_kzalloc(&client->dev, sizeof(*dpll), GFP_KERNEL);
	if (!dpll)
		return -ENOMEM;

	spi_set_drvdata(client, dpll);

	dpll->dev = &client->dev;
	dpll->regmap = devm_regmap_init(&client->dev, NULL, client,
					&microchip_dpll_regmap_config);
	if (IS_ERR(dpll->regmap)) {
		ret = PTR_ERR(dpll->regmap);
		dev_err(dpll->dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}
	mutex_init(&dpll->lock);

	return of_platform_default_populate(dpll->dev->of_node, NULL, dpll->dev);
}

static void microchip_dpll_spi_remove(struct spi_device *client)
{
}

static struct spi_driver microchip_dpll_spi_driver = {
	.driver = {
		.name = "microchip-dpll-spi",
		.of_match_table = of_match_ptr(microchip_dpll_spi_of_match),
	},
	.probe = microchip_dpll_spi_probe,
	.remove	= microchip_dpll_spi_remove,
	.id_table = microchip_dpll_spi_id,
};

static int __init microchip_dpll_spi_init(void)
{
	return spi_register_driver(&microchip_dpll_spi_driver);
}
subsys_initcall(microchip_dpll_spi_init);

static void __exit microchip_dpll_spi_exit(void)
{
	spi_unregister_driver(&microchip_dpll_spi_driver);
}
module_exit(microchip_dpll_spi_exit);

MODULE_DESCRIPTION("Microchip DPLL SPI driver");
MODULE_LICENSE("GPL");
