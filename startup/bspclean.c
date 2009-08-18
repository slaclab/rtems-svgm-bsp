#include <bsp.h>
#include <rtems/bspIo.h>

void
bsp_cleanup(void)
{
printk("RTEMS terminated; rebooting...\n");
bsp_reset();
}
