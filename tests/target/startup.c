// Minimal Cortex-M4F startup for running tests in QEMU (machine mps2-an386)
// with semihosting. FPU settings mirror the sensor firmware.
#include <stdint.h>

extern uint32_t _estack;
extern void _start( void); // newlib crt0: clears .bss, runs constructors, calls main, exit()

void Reset_Handler( void)
{
  *(volatile uint32_t *)0xE000ED88 |= (0xFu << 20); // CPACR: enable CP10/CP11 (FPU)
  __asm volatile ("dsb\n isb");

  uint32_t fpscr; // flush-to-zero, like SET_FPU_FLUSH_TO_ZERO in sw_sensor
  __asm volatile ("vmrs %0, fpscr" : "=r"(fpscr));
  fpscr |= (1u << 24);
  __asm volatile ("vmsr fpscr, %0" : : "r"(fpscr));

  _start();
  for(;;);
}

__attribute__((section(".vectors"), used)) void (* const vectors[])( void) =
{
  (void (*)( void))&_estack,
  Reset_Handler
};
