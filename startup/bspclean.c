#include <bsp.h>
#include <rtems/bspIo.h>

void
bsp_cleanup(void)
{
void rtemsReboot();
printk("RTEMS terminated; rebooting...\n");
rtemsReboot();
}
