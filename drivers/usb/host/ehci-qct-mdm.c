/*
 * ehci-qct-mdm.c
 *
 */

#include <linux/platform_device.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/irq.h>
#include <linux/usb/otg.h>
#include <linux/clk.h>
#include <linux/wakelock.h>

#include <mach/usb_phy.h>
#include <mach/iomap.h>
#include <mach/clk.h>

#include <asm/mach-types.h>

static struct wake_lock ehci_qct_mdm_wlock;

void ehci_qct_mdm_init(void)
{
	printk(KERN_INFO"%s\n",__func__);
	wake_lock_init(&ehci_qct_mdm_wlock, WAKE_LOCK_SUSPEND, "ehci_qct_mdm_wlock");
}
EXPORT_SYMBOL_GPL(ehci_qct_mdm_init);

void ehci_qct_mdm_resume_suspend_recover(void)
{
	extern struct usb_hcd *mdm_hsic_usb_hcd;
	struct ehci_hcd *ehci = hcd_to_ehci (mdm_hsic_usb_hcd);
	u32 __iomem	*status_reg = &ehci->regs->port_status[0];
	//unsigned int val;

	wake_lock(&ehci_qct_mdm_wlock);
	printk(KERN_INFO"%s+\n",__func__);

	printk(KERN_INFO"%s tegra_ehci_bus_suspend\n",__func__);
	tegra_ehci_bus_suspend(mdm_hsic_usb_hcd);
	msleep(50);

	printk(KERN_INFO"%s tegra_ehci_bus_resume\n",__func__);
	tegra_ehci_bus_resume(mdm_hsic_usb_hcd);

	printk(KERN_INFO"%s  USBCMD: %x, PORTSC: %x, USBSTS: %x\n",
		__func__,
		ehci_readl(ehci, &ehci->regs->command),
		ehci_readl(ehci, status_reg),
		ehci_readl(ehci, &ehci->regs->status));

#if 0
	/* CMD_RESET */
	printk(KERN_INFO"%s(%d) CMD_RESET\n",__func__, __LINE__);
	ehci->command |= CMD_RESET;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	if (handshake(ehci, &ehci->regs->command, CMD_RESET, 0, 2500)) {
		pr_err("%s: timeout waiting for CMD_RESET clr\n", __func__);
	}

	printk(KERN_INFO"  USBCMD: %x, PORTSC: %x, USBSTS: %x\n",
		ehci_readl(ehci, &ehci->regs->command),
		ehci_readl(ehci, status_reg),
		ehci_readl(ehci, &ehci->regs->status));

	/* Enable Port Power */
	val = ehci_readl(ehci, status_reg);
	val |= PORT_POWER;
	ehci_writel(ehci, ehci->command, status_reg);
	udelay(10);

	/* Poll until PORT_CONNECT is enabled */
	if (handshake(ehci, status_reg, PORT_CONNECT, PORT_CONNECT, 2000)) {
		pr_err("%s: timeout waiting for PORT_CONNECT\n", __func__);
	}

	/* Poll until PORT_PE is enabled */
	if (handshake(ehci, status_reg, PORT_PE, PORT_PE, 2000)) {
		pr_err("%s: timeout waiting for PORT_PE\n", __func__);
	}
#endif

#if 0
	/* Read CMD_RUN */
	ehci->command = ehci_readl(ehci, &ehci->regs->command);

	/* CLR CMD_RUN */
	printk(KERN_INFO"%s(%d) ~CMD_RUN\n",__func__, __LINE__);
	ehci->command &= ~CMD_RUN;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	msleep(5);
	if (handshake(ehci, &ehci->regs->status, STS_HALT, STS_HALT, 2000)) {
		pr_err("%s: timeout waiting for STS_HALT\n", __func__);
	}

	printk(KERN_INFO"  USBCMD: %x, PORTSC: %x, USBSTS: %x\n",
		ehci_readl(ehci, &ehci->regs->command),
		ehci_readl(ehci, status_reg),
		ehci_readl(ehci, &ehci->regs->status));

	/* Enable CMD_RUN */
	printk(KERN_INFO"%s(%d) CMD_RUN\n",__func__, __LINE__);
	ehci->command |= CMD_RUN;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	printk(KERN_INFO"%s(%d)\n",__func__, __LINE__);
	msleep(5);
	printk(KERN_INFO"%s(%d)\n",__func__, __LINE__);
	if (handshake(ehci, &ehci->regs->command, CMD_RUN, CMD_RUN, 2000)) {
		printk(KERN_INFO"%s: timeout waiting for CMD_RUN to clear\n",__func__);
	}
	printk(KERN_INFO"  USBCMD: %x, PORTSC: %x, USBSTS: %x\n",
		ehci_readl(ehci, &ehci->regs->command),
		ehci_readl(ehci, status_reg),
		ehci_readl(ehci, &ehci->regs->status));
#endif

#if 0
	/* Set USB_PORT_FEAT_SUSPEND */
	printk(KERN_INFO"%s(%d) SUSPEND\n",__func__, __LINE__);
	ehci_hub_control(mdm_hsic_usb_hcd, SetPortFeature, USB_PORT_FEAT_SUSPEND, 1, 0, 0);
	msleep(5);
	if (handshake(ehci, status_reg, PORT_SUSPEND, PORT_SUSPEND, 2000)) {
		printk(KERN_INFO"%s: timeout waiting for PORT_SUSPEND\n",__func__);
	}
#endif

#if 0
	/* Clear USB_PORT_FEAT_SUSPEND */
	printk(KERN_INFO"%s(%d) RESUME\n",__func__, __LINE__);
	ehci_hub_control(mdm_hsic_usb_hcd, ClearPortFeature, USB_PORT_FEAT_SUSPEND, 1, 0, 0);

	/* Poll until (PORT_SUSPEND | PORT_RESUME) is clear */
	if (handshake(ehci, status_reg, (PORT_SUSPEND | PORT_RESUME), 0, 25000)) {
		printk(KERN_INFO"%s: timeout waiting for PORT_RESUME to clear\n",__func__);
	}
	msleep(5);
#endif

#if 0
	/* Clear USB_PORT_FEAT_C_SUSPEND */
	printk(KERN_INFO"%s(%d)\n",__func__, __LINE__);
	ehci_hub_control(mdm_hsic_usb_hcd, ClearPortFeature, USB_PORT_FEAT_C_SUSPEND, 1, 0, 0);
#endif

	printk(KERN_INFO"%s-\n",__func__);
	wake_unlock(&ehci_qct_mdm_wlock);
}
EXPORT_SYMBOL_GPL(ehci_qct_mdm_resume_suspend_recover);

