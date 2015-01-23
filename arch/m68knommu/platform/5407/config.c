/***************************************************************************/

/*
 *	linux/arch/m68knommu/platform/5407/config.c
 *
 *	Copyright (C) 1999-2002, Greg Ungerer (gerg@snapgear.com)
 *	Copyright (C) 2000, Lineo (www.lineo.com)
 */

/***************************************************************************/

#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <asm/machdep.h>
#include <asm/coldfire.h>
#include <asm/mcfsim.h>
#include <asm/mcfuart.h>

/***************************************************************************/

<<<<<<< HEAD
void coldfire_reset(void);

extern unsigned int mcf_timervector;
extern unsigned int mcf_profilevector;
extern unsigned int mcf_timerlevel;

/***************************************************************************/

||||||| merged common ancestors
extern unsigned int mcf_timervector;
extern unsigned int mcf_profilevector;
extern unsigned int mcf_timerlevel;

/***************************************************************************/

=======
>>>>>>> de55a89
static struct mcf_platform_uart m5407_uart_platform[] = {
	{
		.mapbase	= MCF_MBAR + MCFUART_BASE1,
		.irq		= 73,
	},
	{
		.mapbase 	= MCF_MBAR + MCFUART_BASE2,
		.irq		= 74,
	},
	{ },
};

static struct platform_device m5407_uart = {
	.name			= "mcfuart",
	.id			= 0,
	.dev.platform_data	= m5407_uart_platform,
};

static struct platform_device *m5407_devices[] __initdata = {
	&m5407_uart,
};

/***************************************************************************/

static void __init m5407_uart_init_line(int line, int irq)
{
	if (line == 0) {
		writeb(MCFSIM_ICR_LEVEL6 | MCFSIM_ICR_PRI1, MCF_MBAR + MCFSIM_UART1ICR);
		writeb(irq, MCF_MBAR + MCFUART_BASE1 + MCFUART_UIVR);
		mcf_mapirq2imr(irq, MCFINTC_UART0);
	} else if (line == 1) {
		writeb(MCFSIM_ICR_LEVEL6 | MCFSIM_ICR_PRI2, MCF_MBAR + MCFSIM_UART2ICR);
		writeb(irq, MCF_MBAR + MCFUART_BASE2 + MCFUART_UIVR);
		mcf_mapirq2imr(irq, MCFINTC_UART1);
	}
}

static void __init m5407_uarts_init(void)
{
	const int nrlines = ARRAY_SIZE(m5407_uart_platform);
	int line;

	for (line = 0; (line < nrlines); line++)
		m5407_uart_init_line(line, m5407_uart_platform[line].irq);
}

/***************************************************************************/

static void __init m5407_timers_init(void)
{
	/* Timer1 is always used as system timer */
	writeb(MCFSIM_ICR_AUTOVEC | MCFSIM_ICR_LEVEL6 | MCFSIM_ICR_PRI3,
		MCF_MBAR + MCFSIM_TIMER1ICR);
	mcf_mapirq2imr(MCF_IRQ_TIMER, MCFINTC_TIMER1);

#ifdef CONFIG_HIGHPROFILE
	/* Timer2 is to be used as a high speed profile timer  */
	writeb(MCFSIM_ICR_AUTOVEC | MCFSIM_ICR_LEVEL7 | MCFSIM_ICR_PRI3,
		MCF_MBAR + MCFSIM_TIMER2ICR);
	mcf_mapirq2imr(MCF_IRQ_PROFILER, MCFINTC_TIMER2);
#endif
}

/***************************************************************************/

void __init config_BSP(char *commandp, int size)
{
<<<<<<< HEAD
	mcf_setimr(MCFSIM_IMR_MASKALL);

#if defined(CONFIG_CLEOPATRA)
	/* Different timer setup - to prevent device clash */
	mcf_timervector = 30;
	mcf_profilevector = 31;
	mcf_timerlevel = 6;
#endif

	mach_reset = coldfire_reset;
}

/***************************************************************************/

static int __init init_BSP(void)
{
||||||| merged common ancestors
	mcf_setimr(MCFSIM_IMR_MASKALL);

#if defined(CONFIG_CLEOPATRA)
	/* Different timer setup - to prevent device clash */
	mcf_timervector = 30;
	mcf_profilevector = 31;
	mcf_timerlevel = 6;
#endif

	mach_reset = m5407_cpu_reset;
}

/***************************************************************************/

static int __init init_BSP(void)
{
=======
	mach_reset = m5407_cpu_reset;
	m5407_timers_init();
>>>>>>> de55a89
	m5407_uarts_init();

	/* Only support the external interrupts on their primary level */
	mcf_mapirq2imr(25, MCFINTC_EINT1);
	mcf_mapirq2imr(27, MCFINTC_EINT3);
	mcf_mapirq2imr(29, MCFINTC_EINT5);
	mcf_mapirq2imr(31, MCFINTC_EINT7);
}

/***************************************************************************/

static int __init init_BSP(void)
{
	platform_add_devices(m5407_devices, ARRAY_SIZE(m5407_devices));
	return 0;
}

arch_initcall(init_BSP);

/***************************************************************************/
