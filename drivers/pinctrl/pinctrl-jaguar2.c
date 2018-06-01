// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Microsemi SoCs pinctrl driver
 *
 * Author: <alexandre.belloni@free-electrons.com>
 * License: Dual MIT/GPL
 * Copyright (c) 2017 Microsemi Corporation
 */

#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "core.h"
#include "pinconf.h"
#include "pinmux.h"

#define OCELOT_GPIO_OUT_SET	0x0
#define OCELOT_GPIO_OUT_CLR	0x4
#define OCELOT_GPIO_OUT		0x8
#define OCELOT_GPIO_IN		0xc
#define OCELOT_GPIO_OE		0x10
#define OCELOT_GPIO_INTR	0x14
#define OCELOT_GPIO_INTR_ENA	0x18
#define OCELOT_GPIO_INTR_IDENT	0x1c
#define OCELOT_GPIO_ALT0	0x20
#define OCELOT_GPIO_ALT1	0x24
#define OCELOT_GPIO_SD_MAP	0x28

#define OCELOT_PINS		22
#define OCELOT_FUNC_PER_PIN	4

#define JAGUAR2_GPIO_OUT_SET	0x0
#define JAGUAR2_GPIO_OUT_CLR	0x8
#define JAGUAR2_GPIO_OUT	0x10
#define JAGUAR2_GPIO_IN		0x18
#define JAGUAR2_GPIO_OE		0x20
#define JAGUAR2_GPIO_INTR	0x28
#define JAGUAR2_GPIO_INTR_ENA	0x30
#define JAGUAR2_GPIO_INTR_IDENT	0x38
#define JAGUAR2_GPIO_ALT0	0x40
#define JAGUAR2_GPIO_ALT1	0x48
#define JAGUAR2_GPIO_SD_MAP	0x50

#define JAGUAR2_PINS		64
#define JAGUAR2_FUNC_PER_PIN	3

enum {
	FUNC_NONE,
	FUNC_GPIO,
	FUNC_IRQ0_IN,
	FUNC_IRQ0_OUT,
	FUNC_IRQ1_IN,
	FUNC_IRQ1_OUT,
	FUNC_MIIM1,
	FUNC_MIIM2,
	FUNC_PCI_WAKE,
	FUNC_PTP0,
	FUNC_PTP1,
	FUNC_PTP2,
	FUNC_PTP3,
	FUNC_PWM,
	FUNC_RECO_CLK0,
	FUNC_RECO_CLK1,
	FUNC_SFP0,
	FUNC_SFP1,
	FUNC_SFP2,
	FUNC_SFP3,
	FUNC_SFP4,
	FUNC_SFP5,
	FUNC_SFP6,
	FUNC_SFP7,
	FUNC_SFP8,
	FUNC_SFP9,
	FUNC_SFP10,
	FUNC_SFP11,
	FUNC_SFP12,
	FUNC_SFP13,
	FUNC_SFP14,
	FUNC_SFP15,
	FUNC_SG0,
	FUNC_SG1,
	FUNC_SG2,
	FUNC_SI,
	FUNC_TACHO,
	FUNC_TWI,
	FUNC_TWI2,
	FUNC_TWI_SCL_M,
	FUNC_UART,
	FUNC_UART2,
	FUNC_MAX
};

static const char *const jaguar2_function_names[] = {
	[FUNC_NONE]		= "none",
	[FUNC_GPIO]		= "gpio",
	[FUNC_IRQ0_IN]		= "irq0_in",
	[FUNC_IRQ0_OUT]		= "irq0_out",
	[FUNC_IRQ1_IN]		= "irq1_in",
	[FUNC_IRQ1_OUT]		= "irq1_out",
	[FUNC_MIIM1]		= "miim1",
	[FUNC_MIIM2]		= "miim2",
	[FUNC_PCI_WAKE]		= "pci_wake",
	[FUNC_PTP0]		= "ptp0",
	[FUNC_PTP1]		= "ptp1",
	[FUNC_PTP2]		= "ptp2",
	[FUNC_PTP3]		= "ptp3",
	[FUNC_PWM]		= "pwm",
	[FUNC_RECO_CLK0]	= "reco_clk0",
	[FUNC_RECO_CLK1]	= "reco_clk1",
	[FUNC_SFP0]		= "sfp0",
	[FUNC_SFP1]		= "sfp1",
	[FUNC_SFP2]		= "sfp2",
	[FUNC_SFP3]		= "sfp3",
	[FUNC_SFP4]		= "sfp4",
	[FUNC_SFP5]		= "sfp5",
	[FUNC_SFP6]		= "sfp6",
	[FUNC_SFP7]		= "sfp7",
	[FUNC_SFP8]		= "sfp8",
	[FUNC_SFP9]		= "sfp9",
	[FUNC_SFP10]		= "sfp10",
	[FUNC_SFP11]		= "sfp11",
	[FUNC_SFP12]		= "sfp12",
	[FUNC_SFP13]		= "sfp13",
	[FUNC_SFP14]		= "sfp14",
	[FUNC_SFP15]		= "sfp15",
	[FUNC_SG0]		= "sg0",
	[FUNC_SG1]		= "sg1",
	[FUNC_SG2]		= "sg2",
	[FUNC_SI]		= "si",
	[FUNC_TACHO]		= "tacho",
	[FUNC_TWI]		= "twi",
	[FUNC_TWI2]		= "twi2",
	[FUNC_TWI_SCL_M]	= "twi_scl_m",
	[FUNC_UART]		= "uart",
	[FUNC_UART2]		= "uart2",
};

struct jaguar2_pmx_func {
	const char **groups;
	unsigned int ngroups;
};

struct jaguar2_pin_caps {
	unsigned int pin;
	unsigned char functions[JAGUAR2_FUNC_PER_PIN];
};

struct jaguar2_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctl;
	struct gpio_chip gpio_chip;
	struct regmap *map;
	struct jaguar2_pmx_func func[FUNC_MAX];
};

#define JAGUAR2_P(p, f0, f1)						\
static struct jaguar2_pin_caps jaguar2_pin_##p = {			\
	.pin = p,							\
	.functions = {							\
			FUNC_GPIO, FUNC_##f0, FUNC_##f1,		\
	},								\
}

JAGUAR2_P(0,  SG0,       NONE      );
JAGUAR2_P(1,  SG0,       NONE      );
JAGUAR2_P(2,  SG0,       NONE      );
JAGUAR2_P(3,  SG0,       NONE      );
JAGUAR2_P(4,  SG1,       NONE      );
JAGUAR2_P(5,  SG1,       NONE      );
JAGUAR2_P(6,  IRQ0_IN,   IRQ0_OUT  );
JAGUAR2_P(7,  IRQ1_IN,   IRQ1_OUT  );
JAGUAR2_P(8,  PTP0,      NONE      );
JAGUAR2_P(9,  PTP1,      NONE      );
JAGUAR2_P(10, UART,      NONE      );
JAGUAR2_P(11, UART,      NONE      );
JAGUAR2_P(12, SG1,       NONE      );
JAGUAR2_P(13, SG1,       NONE      );
JAGUAR2_P(14, TWI,       TWI_SCL_M );
JAGUAR2_P(15, TWI,       NONE      );
JAGUAR2_P(16, SI,        TWI_SCL_M );
JAGUAR2_P(17, SI,        TWI_SCL_M );
JAGUAR2_P(18, SI,        TWI_SCL_M );
JAGUAR2_P(19, PCI_WAKE,  NONE      );
JAGUAR2_P(20, IRQ0_OUT,  TWI_SCL_M );
JAGUAR2_P(21, IRQ1_OUT,  TWI_SCL_M );
JAGUAR2_P(22, TACHO,     NONE      );
JAGUAR2_P(23, PWM,       NONE      );
JAGUAR2_P(24, UART2,     NONE      );
JAGUAR2_P(25, UART2,     SI        );
JAGUAR2_P(26, PTP2,      SI        );
JAGUAR2_P(27, PTP3,      SI        );
JAGUAR2_P(28, TWI2,      SI        );
JAGUAR2_P(29, TWI2,      SI        );
JAGUAR2_P(30, SG2,       SI        );
JAGUAR2_P(31, SG2,       SI        );
JAGUAR2_P(32, SG2,       SI        );
JAGUAR2_P(33, SG2,       SI        );
JAGUAR2_P(34, NONE,      TWI_SCL_M );
JAGUAR2_P(35, NONE,      TWI_SCL_M );
JAGUAR2_P(36, NONE,      TWI_SCL_M );
JAGUAR2_P(37, NONE,      TWI_SCL_M );
JAGUAR2_P(38, NONE,      TWI_SCL_M );
JAGUAR2_P(39, NONE,      TWI_SCL_M );
JAGUAR2_P(40, NONE,      TWI_SCL_M );
JAGUAR2_P(41, NONE,      TWI_SCL_M );
JAGUAR2_P(42, NONE,      TWI_SCL_M );
JAGUAR2_P(43, NONE,      TWI_SCL_M );
JAGUAR2_P(44, NONE,      SFP8      );
JAGUAR2_P(45, NONE,      SFP9      );
JAGUAR2_P(46, NONE,      SFP10     );
JAGUAR2_P(47, NONE,      SFP11     );
JAGUAR2_P(48, SFP0,      NONE      );
JAGUAR2_P(49, SFP1,      SI        );
JAGUAR2_P(50, SFP2,      SI        );
JAGUAR2_P(51, SFP3,      SI        );
JAGUAR2_P(52, SFP4,      NONE      );
JAGUAR2_P(53, SFP5,      NONE      );
JAGUAR2_P(54, SFP6,      NONE      );
JAGUAR2_P(55, SFP7,      NONE      );
JAGUAR2_P(56, MIIM1,     SFP12     );
JAGUAR2_P(57, MIIM1,     SFP13     );
JAGUAR2_P(58, MIIM2,     SFP14     );
JAGUAR2_P(59, MIIM2,     SFP15     );
JAGUAR2_P(60, NONE,      NONE      );
JAGUAR2_P(61, NONE,      NONE      );
JAGUAR2_P(62, NONE,      NONE      );
JAGUAR2_P(63, NONE,      NONE      );
JAGUAR2_P(64, NONE,      NONE      );

#define JAGUAR2_PIN(n) {						\
	.number = n,						\
	.name = "GPIO_"#n,					\
	.drv_data = &jaguar2_pin_##n				\
}

static const struct pinctrl_pin_desc jaguar2_pins[] = {
	JAGUAR2_PIN(0),
	JAGUAR2_PIN(1),
	JAGUAR2_PIN(2),
	JAGUAR2_PIN(3),
	JAGUAR2_PIN(4),
	JAGUAR2_PIN(5),
	JAGUAR2_PIN(6),
	JAGUAR2_PIN(7),
	JAGUAR2_PIN(8),
	JAGUAR2_PIN(9),
	JAGUAR2_PIN(10),
	JAGUAR2_PIN(11),
	JAGUAR2_PIN(12),
	JAGUAR2_PIN(13),
	JAGUAR2_PIN(14),
	JAGUAR2_PIN(15),
	JAGUAR2_PIN(16),
	JAGUAR2_PIN(17),
	JAGUAR2_PIN(18),
	JAGUAR2_PIN(19),
	JAGUAR2_PIN(20),
	JAGUAR2_PIN(21),
	JAGUAR2_PIN(22),
	JAGUAR2_PIN(23),
	JAGUAR2_PIN(24),
	JAGUAR2_PIN(25),
	JAGUAR2_PIN(26),
	JAGUAR2_PIN(27),
	JAGUAR2_PIN(28),
	JAGUAR2_PIN(29),
	JAGUAR2_PIN(30),
	JAGUAR2_PIN(31),
	JAGUAR2_PIN(32),
	JAGUAR2_PIN(33),
	JAGUAR2_PIN(34),
	JAGUAR2_PIN(35),
	JAGUAR2_PIN(36),
	JAGUAR2_PIN(37),
	JAGUAR2_PIN(38),
	JAGUAR2_PIN(39),
	JAGUAR2_PIN(40),
	JAGUAR2_PIN(41),
	JAGUAR2_PIN(42),
	JAGUAR2_PIN(43),
	JAGUAR2_PIN(44),
	JAGUAR2_PIN(45),
	JAGUAR2_PIN(46),
	JAGUAR2_PIN(47),
	JAGUAR2_PIN(48),
	JAGUAR2_PIN(49),
	JAGUAR2_PIN(50),
	JAGUAR2_PIN(51),
	JAGUAR2_PIN(52),
	JAGUAR2_PIN(53),
	JAGUAR2_PIN(54),
	JAGUAR2_PIN(55),
	JAGUAR2_PIN(56),
	JAGUAR2_PIN(57),
	JAGUAR2_PIN(58),
	JAGUAR2_PIN(59),
	JAGUAR2_PIN(60),
	JAGUAR2_PIN(61),
	JAGUAR2_PIN(62),
	JAGUAR2_PIN(63),
	JAGUAR2_PIN(64),
};

static int jaguar2_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(jaguar2_function_names);
}

static const char *jaguar2_get_function_name(struct pinctrl_dev *pctldev,
					    unsigned int function)
{
	return jaguar2_function_names[function];
}

static int jaguar2_get_function_groups(struct pinctrl_dev *pctldev,
				      unsigned int function,
				      const char *const **groups,
				      unsigned *const num_groups)
{
	struct jaguar2_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	*groups  = info->func[function].groups;
	*num_groups = info->func[function].ngroups;

	return 0;
}

static int jaguar2_pin_function_idx(unsigned int pin, unsigned int function)
{
	struct jaguar2_pin_caps *p = jaguar2_pins[pin].drv_data;
	int i;

	for (i = 0; i < JAGUAR2_FUNC_PER_PIN; i++) {
		if (function == p->functions[i])
			return i;
	}

	return -1;
}

static int jaguar2_pinmux_set_mux(struct pinctrl_dev *pctldev,
				 unsigned int selector, unsigned int group)
{
	struct jaguar2_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);
	struct jaguar2_pin_caps *pin = jaguar2_pins[group].drv_data;
	int f;

	f = jaguar2_pin_function_idx(group, selector);
	if (f < 0)
		return -EINVAL;

	/*
	 * f is encoded on two bits.
	 * bit 0 of f goes in BIT(pin) of ALT0, bit 1 of f goes in BIT(pin) of
	 * ALT1
	 * This is racy because both registers can't be updated at the same time
	 * but it doesn't matter much for now.
	 */
	regmap_update_bits(info->map, JAGUAR2_GPIO_ALT0, BIT(pin->pin),
			   f << pin->pin);
	regmap_update_bits(info->map, JAGUAR2_GPIO_ALT1, BIT(pin->pin),
			   f << (pin->pin - 1));

	return 0;
}

static int jaguar2_gpio_set_direction(struct pinctrl_dev *pctldev,
				     struct pinctrl_gpio_range *range,
				     unsigned int pin, bool input)
{
	struct jaguar2_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	regmap_update_bits(info->map, JAGUAR2_GPIO_OE, BIT(pin),
			   input ? 0 : BIT(pin));

	return 0;
}

static int jaguar2_gpio_request_enable(struct pinctrl_dev *pctldev,
				      struct pinctrl_gpio_range *range,
				      unsigned int offset)
{
	struct jaguar2_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	regmap_update_bits(info->map, JAGUAR2_GPIO_ALT0, BIT(offset), 0);
	regmap_update_bits(info->map, JAGUAR2_GPIO_ALT1, BIT(offset), 0);

	return 0;
}

static const struct pinmux_ops jaguar2_pmx_ops = {
	.get_functions_count = jaguar2_get_functions_count,
	.get_function_name = jaguar2_get_function_name,
	.get_function_groups = jaguar2_get_function_groups,
	.set_mux = jaguar2_pinmux_set_mux,
	.gpio_set_direction = jaguar2_gpio_set_direction,
	.gpio_request_enable = jaguar2_gpio_request_enable,
};

static int jaguar2_pctl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(jaguar2_pins);
}

static const char *jaguar2_pctl_get_group_name(struct pinctrl_dev *pctldev,
					      unsigned int group)
{
	return jaguar2_pins[group].name;
}

static int jaguar2_pctl_get_group_pins(struct pinctrl_dev *pctldev,
				      unsigned int group,
				      const unsigned int **pins,
				      unsigned int *num_pins)
{
	*pins = &jaguar2_pins[group].number;
	*num_pins = 1;

	return 0;
}

static const struct pinctrl_ops jaguar2_pctl_ops = {
	.get_groups_count = jaguar2_pctl_get_groups_count,
	.get_group_name = jaguar2_pctl_get_group_name,
	.get_group_pins = jaguar2_pctl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static struct pinctrl_desc jaguar2_desc = {
	.name = "jaguar2-pinctrl",
	.pins = jaguar2_pins,
	.npins = ARRAY_SIZE(jaguar2_pins),
	.pctlops = &jaguar2_pctl_ops,
	.pmxops = &jaguar2_pmx_ops,
	.owner = THIS_MODULE,
};

static int jaguar2_create_group_func_map(struct device *dev,
					struct jaguar2_pinctrl *info)
{
	u16 pins[ARRAY_SIZE(jaguar2_pins)];
	int f, npins, i;

	for (f = 0; f < FUNC_MAX; f++) {
		for (npins = 0, i = 0; i < ARRAY_SIZE(jaguar2_pins); i++) {
			if (jaguar2_pin_function_idx(i, f) >= 0)
				pins[npins++] = i;
		}

		info->func[f].ngroups = npins;
		info->func[f].groups = devm_kzalloc(dev, npins *
							 sizeof(char *),
							 GFP_KERNEL);
		if (!info->func[f].groups)
			return -ENOMEM;

		for (i = 0; i < npins; i++)
			info->func[f].groups[i] = jaguar2_pins[pins[i]].name;
	}

	return 0;
}

static int jaguar2_pinctrl_register(struct platform_device *pdev,
				   struct jaguar2_pinctrl *info)
{
	int ret;

	ret = jaguar2_create_group_func_map(&pdev->dev, info);
	if (ret) {
		dev_err(&pdev->dev, "Unable to create group func map.\n");
		return ret;
	}

	info->pctl = devm_pinctrl_register(&pdev->dev, &jaguar2_desc, info);
	if (IS_ERR(info->pctl)) {
		dev_err(&pdev->dev, "Failed to register pinctrl\n");
		return PTR_ERR(info->pctl);
	}

	return 0;
}

static int jaguar2_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct jaguar2_pinctrl *info = gpiochip_get_data(chip);
	unsigned int val;

	regmap_read(info->map, JAGUAR2_GPIO_IN, &val);

	return !!(val & BIT(offset));
}

static void jaguar2_gpio_set(struct gpio_chip *chip, unsigned int offset,
			    int value)
{
	struct jaguar2_pinctrl *info = gpiochip_get_data(chip);

	if (value)
		regmap_write(info->map, JAGUAR2_GPIO_OUT_SET, BIT(offset));
	else
		regmap_write(info->map, JAGUAR2_GPIO_OUT_CLR, BIT(offset));
}

static int jaguar2_gpio_get_direction(struct gpio_chip *chip,
				     unsigned int offset)
{
	struct jaguar2_pinctrl *info = gpiochip_get_data(chip);
	unsigned int val;

	regmap_read(info->map, JAGUAR2_GPIO_OE, &val);

	return !(val & BIT(offset));
}

static int jaguar2_gpio_direction_input(struct gpio_chip *chip,
				       unsigned int offset)
{
	return pinctrl_gpio_direction_input(chip->base + offset);
}

static int jaguar2_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset, int value)
{
	struct jaguar2_pinctrl *info = gpiochip_get_data(chip);
	unsigned int pin = BIT(offset);

	if (value)
		regmap_write(info->map, JAGUAR2_GPIO_OUT_SET, pin);
	else
		regmap_write(info->map, JAGUAR2_GPIO_OUT_CLR, pin);

	return pinctrl_gpio_direction_output(chip->base + offset);
}

static const struct gpio_chip jaguar2_gpiolib_chip = {
	.request = gpiochip_generic_request,
	.free = gpiochip_generic_free,
	.set = jaguar2_gpio_set,
	.get = jaguar2_gpio_get,
	.get_direction = jaguar2_gpio_get_direction,
	.direction_input = jaguar2_gpio_direction_input,
	.direction_output = jaguar2_gpio_direction_output,
	.owner = THIS_MODULE,
};

static int jaguar2_gpiochip_register(struct platform_device *pdev,
				    struct jaguar2_pinctrl *info)
{
	struct gpio_chip *gc;
	int ret;

	info->gpio_chip = jaguar2_gpiolib_chip;

	gc = &info->gpio_chip;
	gc->ngpio = JAGUAR2_PINS;
	gc->parent = &pdev->dev;
	gc->base = 0;
	gc->of_node = info->dev->of_node;
	gc->label = "jaguar2-gpio";

	ret = devm_gpiochip_add_data(&pdev->dev, gc, info);
	if (ret)
		return ret;

	/* TODO: this can be used as an irqchip but no board is using that */

	return 0;
}

static const struct regmap_config jaguar2_pinctrl_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x8c,
};

static const struct of_device_id jaguar2_pinctrl_of_match[] = {
	{ .compatible = "mscc,jaguar2-pinctrl", },
	{},
};

static int jaguar2_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct jaguar2_pinctrl *info;
	void __iomem *base;
	int ret;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	base = devm_ioremap_resource(dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(base)) {
		dev_err(dev, "Failed to ioremap registers\n");
		return PTR_ERR(base);
	}

	info->map = devm_regmap_init_mmio(dev, base,
					  &jaguar2_pinctrl_regmap_config);
	if (IS_ERR(info->map)) {
		dev_err(dev, "Failed to create regmap\n");
		return PTR_ERR(info->map);
	}
	dev_set_drvdata(dev, info->map);
	info->dev = dev;

	ret = jaguar2_pinctrl_register(pdev, info);
	if (ret)
		return ret;

	ret = jaguar2_gpiochip_register(pdev, info);
	if (ret)
		return ret;

	return 0;
}

static struct platform_driver jaguar2_pinctrl_driver = {
	.driver = {
		.name = "pinctrl-jaguar2",
		.of_match_table = of_match_ptr(jaguar2_pinctrl_of_match),
		.suppress_bind_attrs = true,
	},
	.probe = jaguar2_pinctrl_probe,
};
builtin_platform_driver(jaguar2_pinctrl_driver);
