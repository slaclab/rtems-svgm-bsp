void
bsp_cleanup(void)
{
void rtemsReboot();
printk("RTEMS terminated; rebooting...\n");
rtemsReboot();
}
