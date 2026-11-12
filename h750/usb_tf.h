#pragma once
void usb_tf_start(void);
int usb_tf_host_active(void);
/* 采集过程中拒绝电脑读卡；等文件关闭后再把卡交给 USB。 */
void usb_tf_set_busy(unsigned busy);
int usb_tf_connected(void);
