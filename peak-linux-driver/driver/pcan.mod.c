#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x2005612d, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x6aeb2956, __VMLINUX_SYMBOL_STR(pcmcia_dev_present) },
	{ 0x6bc3fbc0, __VMLINUX_SYMBOL_STR(__unregister_chrdev) },
	{ 0x2d3385d3, __VMLINUX_SYMBOL_STR(system_wq) },
	{ 0x1fedf0f4, __VMLINUX_SYMBOL_STR(__request_region) },
	{ 0xb275076b, __VMLINUX_SYMBOL_STR(class_remove_file_ns) },
	{ 0xffc92130, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0x1ed8b599, __VMLINUX_SYMBOL_STR(__x86_indirect_thunk_r8) },
	{ 0x9dd43850, __VMLINUX_SYMBOL_STR(pcmcia_enable_device) },
	{ 0x457b15f3, __VMLINUX_SYMBOL_STR(single_open) },
	{ 0x69a358a6, __VMLINUX_SYMBOL_STR(iomem_resource) },
	{ 0xbc8a3732, __VMLINUX_SYMBOL_STR(pcmcia_register_driver) },
	{ 0xba1093c8, __VMLINUX_SYMBOL_STR(usb_init_urb) },
	{ 0x4860fe4f, __VMLINUX_SYMBOL_STR(single_release) },
	{ 0xe8b45412, __VMLINUX_SYMBOL_STR(usb_reset_endpoint) },
	{ 0x3d5383d0, __VMLINUX_SYMBOL_STR(pci_disable_device) },
	{ 0xb968e7fd, __VMLINUX_SYMBOL_STR(i2c_transfer) },
	{ 0x20000329, __VMLINUX_SYMBOL_STR(simple_strtoul) },
	{ 0x6b06fdce, __VMLINUX_SYMBOL_STR(delayed_work_timer_fn) },
	{ 0x30fe3b7b, __VMLINUX_SYMBOL_STR(seq_printf) },
	{ 0xc87c1f84, __VMLINUX_SYMBOL_STR(ktime_get) },
	{ 0xa6364f5c, __VMLINUX_SYMBOL_STR(usb_kill_urb) },
	{ 0xea40aa27, __VMLINUX_SYMBOL_STR(remove_proc_entry) },
	{ 0x4c2d2ecd, __VMLINUX_SYMBOL_STR(parport_find_base) },
	{ 0xcbe14b99, __VMLINUX_SYMBOL_STR(__register_chrdev) },
	{ 0xc7cd6018, __VMLINUX_SYMBOL_STR(driver_for_each_device) },
	{ 0xeae3dfd6, __VMLINUX_SYMBOL_STR(__const_udelay) },
	{ 0x22e5a4e3, __VMLINUX_SYMBOL_STR(pci_release_regions) },
	{ 0x9580deb, __VMLINUX_SYMBOL_STR(init_timer_key) },
	{ 0xa57863e, __VMLINUX_SYMBOL_STR(cancel_delayed_work_sync) },
	{ 0x47256491, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x27b6110a, __VMLINUX_SYMBOL_STR(pci_bus_write_config_word) },
	{ 0xd2f24eff, __VMLINUX_SYMBOL_STR(device_create_with_groups) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x8b24d273, __VMLINUX_SYMBOL_STR(pcmcia_request_io) },
	{ 0xfff54354, __VMLINUX_SYMBOL_STR(seq_read) },
	{ 0x15ba50a6, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0xdb96137, __VMLINUX_SYMBOL_STR(mutex_trylock) },
	{ 0x9e88526, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0xccc3d30f, __VMLINUX_SYMBOL_STR(dma_get_required_mask) },
	{ 0x4f8b5ddb, __VMLINUX_SYMBOL_STR(_copy_to_user) },
	{ 0x4488ae17, __VMLINUX_SYMBOL_STR(param_ops_charp) },
	{ 0xb8f4fc9d, __VMLINUX_SYMBOL_STR(pci_set_master) },
	{ 0x706d051c, __VMLINUX_SYMBOL_STR(del_timer_sync) },
	{ 0xfb578fc5, __VMLINUX_SYMBOL_STR(memset) },
	{ 0xff7559e4, __VMLINUX_SYMBOL_STR(ioport_resource) },
	{ 0xb558270c, __VMLINUX_SYMBOL_STR(device_del) },
	{ 0x322643b7, __VMLINUX_SYMBOL_STR(pci_iounmap) },
	{ 0x8180b3bd, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x1916e38c, __VMLINUX_SYMBOL_STR(_raw_spin_unlock_irqrestore) },
	{ 0xbf2b6129, __VMLINUX_SYMBOL_STR(current_task) },
	{ 0x37befc70, __VMLINUX_SYMBOL_STR(jiffies_to_msecs) },
	{ 0x4e96000d, __VMLINUX_SYMBOL_STR(usb_deregister) },
	{ 0x14f2edb, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x7a01d993, __VMLINUX_SYMBOL_STR(sysfs_remove_file_from_group) },
	{ 0x2b57213e, __VMLINUX_SYMBOL_STR(parport_unregister_device) },
	{ 0x86037272, __VMLINUX_SYMBOL_STR(usb_set_interface) },
	{ 0xe7b00dfb, __VMLINUX_SYMBOL_STR(__x86_indirect_thunk_r13) },
	{ 0xa1c76e0a, __VMLINUX_SYMBOL_STR(_cond_resched) },
	{ 0x9166fada, __VMLINUX_SYMBOL_STR(strncpy) },
	{ 0xfbb5dbad, __VMLINUX_SYMBOL_STR(usb_control_msg) },
	{ 0x16305289, __VMLINUX_SYMBOL_STR(warn_slowpath_null) },
	{ 0xbfe8ed5b, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0x1bb31047, __VMLINUX_SYMBOL_STR(add_timer) },
	{ 0x9be6b93a, __VMLINUX_SYMBOL_STR(parport_claim) },
	{ 0x2072ee9b, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0xa73325fc, __VMLINUX_SYMBOL_STR(pcmcia_loop_config) },
	{ 0xfb42566a, __VMLINUX_SYMBOL_STR(parport_release) },
	{ 0x8af46adc, __VMLINUX_SYMBOL_STR(arch_dma_alloc_attrs) },
	{ 0x11fcb1e5, __VMLINUX_SYMBOL_STR(i2c_del_adapter) },
	{ 0x3ca614ca, __VMLINUX_SYMBOL_STR(usb_submit_urb) },
	{ 0x78764f4e, __VMLINUX_SYMBOL_STR(pv_irq_ops) },
	{ 0xb601be4c, __VMLINUX_SYMBOL_STR(__x86_indirect_thunk_rdx) },
	{ 0x42c8de35, __VMLINUX_SYMBOL_STR(ioremap_nocache) },
	{ 0xee1e8c21, __VMLINUX_SYMBOL_STR(pci_bus_read_config_word) },
	{ 0x70cd1f, __VMLINUX_SYMBOL_STR(queue_delayed_work_on) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x4e9c4f77, __VMLINUX_SYMBOL_STR(usb_reset_device) },
	{ 0xb298621d, __VMLINUX_SYMBOL_STR(parport_register_device) },
	{ 0xd62c833f, __VMLINUX_SYMBOL_STR(schedule_timeout) },
	{ 0x1000e51, __VMLINUX_SYMBOL_STR(schedule) },
	{ 0x3bfb4858, __VMLINUX_SYMBOL_STR(usb_clear_halt) },
	{ 0x2ea2c95c, __VMLINUX_SYMBOL_STR(__x86_indirect_thunk_rax) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0x71a1bfe8, __VMLINUX_SYMBOL_STR(class_create_file_ns) },
	{ 0x7c61340c, __VMLINUX_SYMBOL_STR(__release_region) },
	{ 0xa5ad3953, __VMLINUX_SYMBOL_STR(pci_enable_msi_range) },
	{ 0x18883aa8, __VMLINUX_SYMBOL_STR(pci_unregister_driver) },
	{ 0x44221035, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x680ec266, __VMLINUX_SYMBOL_STR(_raw_spin_lock_irqsave) },
	{ 0x2229b9d9, __VMLINUX_SYMBOL_STR(param_ops_byte) },
	{ 0xa6bbd805, __VMLINUX_SYMBOL_STR(__wake_up) },
	{ 0xf6ebc03b, __VMLINUX_SYMBOL_STR(net_ratelimit) },
	{ 0x2207a57f, __VMLINUX_SYMBOL_STR(prepare_to_wait_event) },
	{ 0x4f68e5c9, __VMLINUX_SYMBOL_STR(do_gettimeofday) },
	{ 0xf1b9f044, __VMLINUX_SYMBOL_STR(proc_create_data) },
	{ 0xeeb9e940, __VMLINUX_SYMBOL_STR(seq_lseek) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x69acdf38, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0x8ca13db2, __VMLINUX_SYMBOL_STR(pci_request_regions) },
	{ 0x397bdd46, __VMLINUX_SYMBOL_STR(param_array_ops) },
	{ 0x379581ce, __VMLINUX_SYMBOL_STR(pci_disable_msi) },
	{ 0x28e805b1, __VMLINUX_SYMBOL_STR(dma_supported) },
	{ 0xedc03953, __VMLINUX_SYMBOL_STR(iounmap) },
	{ 0x78e739aa, __VMLINUX_SYMBOL_STR(up) },
	{ 0x7cce103a, __VMLINUX_SYMBOL_STR(pcmcia_unregister_driver) },
	{ 0xa03f14d2, __VMLINUX_SYMBOL_STR(__pci_register_driver) },
	{ 0x88793eb8, __VMLINUX_SYMBOL_STR(usb_register_driver) },
	{ 0x8b8dd7ac, __VMLINUX_SYMBOL_STR(class_destroy) },
	{ 0xf08242c2, __VMLINUX_SYMBOL_STR(finish_wait) },
	{ 0x2f16d718, __VMLINUX_SYMBOL_STR(dev_warn) },
	{ 0xd54d4ae3, __VMLINUX_SYMBOL_STR(sysfs_add_file_to_group) },
	{ 0x8c2721f, __VMLINUX_SYMBOL_STR(i2c_bit_add_bus) },
	{ 0x28318305, __VMLINUX_SYMBOL_STR(snprintf) },
	{ 0xb0e602eb, __VMLINUX_SYMBOL_STR(memmove) },
	{ 0xe637e9c8, __VMLINUX_SYMBOL_STR(pci_iomap) },
	{ 0x75d4388d, __VMLINUX_SYMBOL_STR(param_ops_ushort) },
	{ 0xe2135f2e, __VMLINUX_SYMBOL_STR(pcmcia_disable_device) },
	{ 0xd60ea379, __VMLINUX_SYMBOL_STR(pci_enable_device) },
	{ 0x4f6b400b, __VMLINUX_SYMBOL_STR(_copy_from_user) },
	{ 0xe298a062, __VMLINUX_SYMBOL_STR(param_ops_uint) },
	{ 0xf6185bb8, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0x2a7eba5e, __VMLINUX_SYMBOL_STR(dma_ops) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=pcmcia,parport,i2c-algo-bit";

MODULE_ALIAS("pcmcia:m0377c0001f*fn*pfn*pa*pb*pc*pd*");
MODULE_ALIAS("pci:v0000001Cd00000001sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000003sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000004sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000005sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000006sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000007sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000008sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000002sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000000Asv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000009sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000010sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000013sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000014sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000017sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000018sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000019sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000001Asv*sd*bc*sc*i*");
MODULE_ALIAS("usb:v0C72p000Cd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p000Dd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0012d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0011d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0013d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0014d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "93F511CC9D0884CDA73CEFB");
