#ifndef _UART_PXA_H_
#define _UART_PXA_H_

struct pxa_uart_mach_info {
        int (*uart_pin_switch)(int enable);
};

extern void pxa_set_ffuart_info(struct pxa_uart_mach_info *info);
extern void pxa_set_stuart_info(struct pxa_uart_mach_info *info);
extern void pxa_set_btuart_info(struct pxa_uart_mach_info *info);
extern void pxa_set_hwuart_info(struct pxa_uart_mach_info *info);
#endif

