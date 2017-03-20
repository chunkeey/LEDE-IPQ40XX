/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/phy.h>
#include <linux/reset.h>
#include <linux/of_device.h>
#include <linux/qcom_scm.h>

struct qca_uni_ss_phy {
	struct usb_phy phy;
	struct device *dev;

	void __iomem *base;

	struct reset_control *por_rst;
};

#define phy_to_dw_phy(x) container_of((x), struct qca_uni_ss_phy, phy)

static void qca_uni_ss_phy_shutdown(struct usb_phy *x)
{
	struct qca_uni_ss_phy *phy = phy_to_dw_phy(x);

	/* assert SS PHY POR reset */
	reset_control_assert(phy->por_rst);
}

#define USB_CALIBRATION_CMD	0x10
#define USB3PHY_SPARE_1		0x7FC
#define RX_LOS_1		0x7C8
#define MISC_SOURCE_REG		0x21c
#define CDR_CONTROL_REG_1	0x80
#define PCS_INTERNAL_CONTROL14	0x364
#define MMD1_REG_REG_MASK	(0x7F << 8)
#define OTP_MASK		(0x7F << 5)
#define MMD1_REG_AUTOLOAD_MASK	(0x1 << 7)
#define SPARE_1_BIT14_MASK	(0x1 << 14)

int qca_uni_ss_phy_usb_los_calibration(void __iomem *base)
{
	int err;
	uint32_t data, otp_val = 0;

	/* Get OTP value */
	err = qcom_scm_fuse(USB_CALIBRATION_CMD, &otp_val);
	if (err < 0 || !(otp_val & OTP_MASK)) {
		pr_err("USB Calibration Failed with error %d %d\n", err, otp_val);
		return 0;
	}

	pr_info("Raw USB3 Calibration value %x\n", otp_val);

	/*
	 * Read the USB3PHY_SPARE_1 register and
	 * set bit 14 to 0
	 */
	data = readl_relaxed(base + USB3PHY_SPARE_1);
	data = data & (~SPARE_1_BIT14_MASK);
	writel(data, base + USB3PHY_SPARE_1);
	udelay(100);

	/*
	 * Get bit 11:5 value, add with 0x14 and set to the
	 * register USB3PHY_RX_LOS_1 bit MMD1_REG_REG
	 */
	data = readl_relaxed(base + RX_LOS_1);
	otp_val = ((otp_val & OTP_MASK) >> 5) + 0x14;
	otp_val = otp_val << 8;
	data = data & (~MMD1_REG_REG_MASK);
	data = data | otp_val;
	writel(data, base + RX_LOS_1);
	udelay(100);

	/*
	 * Set bit MMD1_REG_AUTOLOAD_SEL_RX_LOS_THRES in
	 * USB3PHY_RX_LOS_1 to 1
	 */
	data = readl_relaxed(base + RX_LOS_1);
	data = data | MMD1_REG_AUTOLOAD_MASK;
	writel(data, base + RX_LOS_1);
	udelay(100);

	writel(0x4000, base + PCS_INTERNAL_CONTROL14);
	udelay(100);
	writel(0xaa0a, base + MISC_SOURCE_REG);
	udelay(100);
	writel(0x0202, base + CDR_CONTROL_REG_1);
	udelay(100);

	return 0;
}

static int qca_uni_ss_phy_init(struct usb_phy *x)
{
	struct qca_uni_ss_phy *phy = phy_to_dw_phy(x);

	/* assert SS PHY POR reset */
	reset_control_assert(phy->por_rst);

	msleep(20);

	/* deassert SS PHY POR reset */
	reset_control_deassert(phy->por_rst);

	/* USB LOS Calibration */
	return qca_uni_ss_phy_usb_los_calibration(phy->base);
}

static int qca_uni_ss_get_resources(struct platform_device *pdev,
		struct qca_uni_ss_phy *phy)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	phy->base = devm_ioremap_resource(phy->dev, res);
	if (IS_ERR(phy->base))
		return PTR_ERR(phy->base);

	phy->por_rst = devm_reset_control_get(phy->dev, "por_rst");
	if (IS_ERR(phy->por_rst))
		return PTR_ERR(phy->por_rst);

	return 0;
}

static int qca_uni_ss_remove(struct platform_device *pdev)
{
	struct qca_uni_ss_phy *phy = platform_get_drvdata(pdev);

	usb_remove_phy(&phy->phy);
	return 0;
}

static const struct of_device_id qca_uni_ss_id_table[] = {
	{ .compatible = "qca,uni-ssphy" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, qca_uni_ss_id_table);

static int qca_uni_ss_probe(struct platform_device *pdev)
{
	struct qca_uni_ss_phy *phy;
	int ret;

	phy = devm_kzalloc(&pdev->dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	platform_set_drvdata(pdev, phy);
	phy->dev = &pdev->dev;

	ret = qca_uni_ss_get_resources(pdev, phy);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request resources: %d\n", ret);
		return ret;
	}

	phy->phy.dev = phy->dev;
	phy->phy.label = "qca-uni-ssphy";
	phy->phy.init = qca_uni_ss_phy_init;
	phy->phy.shutdown = qca_uni_ss_phy_shutdown;
	phy->phy.type = USB_PHY_TYPE_USB3;

	ret = usb_add_phy_dev(&phy->phy);
	return ret;
}

static struct platform_driver qca_uni_ss_driver = {
	.probe = qca_uni_ss_probe,
	.remove	= qca_uni_ss_remove,
	.driver = {
		.name = "qca-uni-ssphy",
		.owner = THIS_MODULE,
		.of_match_table = qca_uni_ss_id_table,
	},
};

module_platform_driver(qca_uni_ss_driver);

MODULE_ALIAS("platform:qca-uni-ssphy");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("USB3 QCA UNI SSPHY driver");
