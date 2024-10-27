# 基于GEC6818开发板（s5p6818）移植64位linux系统-uboot部分

[原版官方uboot项目 READE文件](./uboot_README)

基于 https://github.com/friendlyarm/u-boot.git 工程，nanopi2-v2016.01分支，23ef2cc提交进行开发

提交说明：nanopi2: increase IO drive strength for HD700, S702


## 项目介绍：

[基于GEC6818开发板（s5p6818）移植64位linux系统_linux gec6818 3.4.39-gec #3 smp preempt tue jun 4 -CSDN博客](https://blog.csdn.net/B_X_Z/article/details/136825225#comments_32116831)

[基于GEC6818开发板（s5p6818）移植64位linux系统-烧录篇（一）-CSDN博客](https://blog.csdn.net/B_X_Z/article/details/141439554)



## 编译命令

```
make s5p6818_gec6818_config
make CROSS_COMPILE=aarch64-linux-
```

编译成功结束后在当前目前可获得fip-nonsecure.img


## 微信讨论、交流群-针对gec6818开发板移植交流

<img src="doc/微信扫码加群.png" alt="微信扫码加群" style="zoom: 50%;" />
