bug记录：
1、发送传送文件，固件升级完之后，返回的命令忘了增加0x60	   (bug已修复)
2、升级到一半之后，断开连接，此时舵机不能分配ID，所以就不能升级了


bootloader放置在aprom，从0x0000开始，到0x2000结束，并设置新唐MCU从aprom启动(with iap)
