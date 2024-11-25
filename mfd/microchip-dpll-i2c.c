// SPDX-License-Identifier: GPL-2.0+

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/mfd/microchip-dpll.h>

#define MICROCHIP_DPLL_PAGE_ADDR		0x007F
#define MICROCHIP_DPLL_HIGHER_ADDR_MASK		0xFF80
#define MICROCHIP_DPLL_LOWER_ADDR_MASK		0x007F

static const struct i2c_device_id microchip_dpll_i2c_id[] = {
	{ "zl80732-i2c",  },
	{}
};
MODULE_DEVICE_TABLE(i2c, microchip_dpll_i2c_id);

static const struct of_device_id microchip_dpll_i2c_of_match[] = {
	{ .compatible = "microchip,zl80732-i2c" },
	{}
};
MODULE_DEVICE_TABLE(of, microchip_dpll_i2c_of_match);

static int microchip_dpll_read_device(struct microchip_dpll_ddata *dpll,
				      u8 reg, u8 *buf, u16 bytes)
{
	struct i2c_client *client = to_i2c_client(dpll->dev);
	struct i2c_msg msg[2];
	int cnt;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = bytes;
	msg[1].buf = buf;

	cnt = i2c_transfer(client->adapter, msg, 2);

	if (cnt < 0) {
		dev_err(dpll->dev, "i2c_transfer failed at addr: %04x!", reg);
		return cnt;
	} else if (cnt != 2) {
		dev_err(dpll->dev,
			"i2c_transfer sent only %d of 2 messages", cnt);
		return -EIO;
	}

	return 0;
}

static int microchip_dpll_write_device(struct microchip_dpll_ddata *dpll,
				       u8 reg, u8 *buf, u16 bytes)
{
	struct i2c_client *client = to_i2c_client(dpll->dev);
	u8 msg[256];
	int cnt;

	msg[0] = reg;
	memcpy(&msg[1], buf, bytes);

	cnt = i2c_master_send(client, msg, bytes + 1);

	if (cnt < 0) {
		dev_err(&client->dev,
			"i2c_master_send failed at addr: %04x!", reg);
		return cnt;
	}

	return 0;
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
	struct microchip_dpll_ddata *dpll = i2c_get_clientdata((struct i2c_client *)context);
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
	struct microchip_dpll_ddata *dpll = i2c_get_clientdata((struct i2c_client *)context);
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

static int microchip_dpll_i2c_probe(struct i2c_client *client)
{
	struct microchip_dpll_ddata *dpll;
	int ret;

	dpll = devm_kzalloc(&client->dev, sizeof(*dpll), GFP_KERNEL);
	if (!dpll)
		return -ENOMEM;

	i2c_set_clientdata(client, dpll);

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

static void microchip_dpll_i2c_remove(struct i2c_client *client)
{
}

static struct i2c_driver microchip_dpll_i2c_driver = {
	.driver = {
		.name = "microchip-dpll-i2c",
		.of_match_table = of_match_ptr(microchip_dpll_i2c_of_match),
	},
	.probe = microchip_dpll_i2c_probe,
	.remove	= microchip_dpll_i2c_remove,
	.id_table = microchip_dpll_i2c_id,
};

static int __init microchip_dpll_i2c_init(void)
{
	return i2c_add_driver(&microchip_dpll_i2c_driver);
}
subsys_initcall(microchip_dpll_i2c_init);

static void __exit microchip_dpll_i2c_exit(void)
{
	i2c_del_driver(&microchip_dpll_i2c_driver);
}
module_exit(microchip_dpll_i2c_exit);

MODULE_DESCRIPTION("Microchip DPLL I2C driver");
MODULE_LICENSE("GPL");
