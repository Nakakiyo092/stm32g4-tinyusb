This repository is an example of creating a USB device with an STM32G431 board. The used USB stack is TinyUSB, instead of the ST's own USB middleware.
The steps to integrate the TinyUSB to the STM32CubeIde project is mainly based on the following repository.

https://github.com/ejaaskel/stm32_tinyusb

The example code is from TinyUSB, from the CDC Dual Ports device example.

https://github.com/hathach/tinyusb/tree/master/examples/device/cdc_dual_ports

According to the following discussion, some interrupts have to be manually added for STM32G4.
In my case adding USB wakeup interrupt is crucial to start USB communication.
Without the interrupt, the USB communication does not start correctry saying "unknown descriptor".

https://github.com/hathach/tinyusb/discussions/3097

The final CRS step is added following the discussion in the issue post below.
It is said to be necessary to utilize HSI48, but it was not crucial in my case.

https://github.com/hathach/tinyusb/issues/1014
