# ch37x Linux Host Controller driver for USB

## Description

CH374/CH370 is a chip implementing a USB 2.0 Full-/Low-Speed host controller on a SPI bus. CH374 can expand 3 USB host while CH370 can only expand 1 USB host for Linux/Android system.

## Driver Operating Overview

First of all, you should compile the driver and integrate it to kernel.

Before the driver works, you should make sure that the hardware is connected and is working properly, the SPI and interrupt signals connection is especially important.

More details please refer to the other files.

## Note

Any question, you can send feedback to mail: [tech@wch.cn](mailto:tech@wch.cn)