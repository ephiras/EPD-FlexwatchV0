# Flexwatch

A simple, low stress smartwatch built around a 2.9 inch e ink display, a custom PCB, and a lot of improvised problem solving. This repository contains the Arduino code and the Eagle hardware files for the current working version.

The goal is a comfortable watch that is easy on the eyes, lasts a long time on a charge, and does not behave like a tiny phone.
![20260219_114707](https://github.com/user-attachments/assets/0e3a4dc1-aa7c-49bf-b5ea-8cf761fa90fe)

---

## What is in this repository

### code
Arduino sketches for the current version.  
Bluetooth and microphone support are not implemented yet, but the hardware is ready.

### hardware
Eagle schematic and PCB layout for the custom flex board used in the build.
<img width="3319" height="3044" alt="tmp_00848f6a-d5dd-4d53-946b-520c7fa0a136" src="https://github.com/user-attachments/assets/91cd6423-b4a5-41f8-b39c-44028cfb5ca6" />
---

## Current features


- 2.9 inch e ink display  
- Seeed Studio nRF52840 Sense  
- IMU support  
- Basic watch interface  
- Low power modes  
- Custom flex PCB  
- Ruler based rigid frame  

---

## Planned features

- Bluetooth  
- Microphone  
- GPS (new layout needed)  
- Heart rate sensor  
- UV sensor  
- Rigid flex PCB  
- Improved sleep modes  
- RTC backup battery  

---

## Notes

This is a work in progress.  
The hardware files match the version shown in the Instructables build.  
Expect changes as the design evolves.

## The Zephyr Migration
I’ve started migrating the firmware for the e‑ink smartwatch from the Arduino environment to Zephyr RTOS. This is a significant step forward for the project and sets up a much more robust foundation for future development.

#Why the migration
-Better power management: Zephyr gives fine‑grained control over sleep states, peripherals, and scheduling, which is essential for a low‑power wearable running on a small battery.
-Native nRF52840 support: Zephyr’s Nordic integration is far deeper than Arduino’s, with first‑class drivers, BLE stack support, and stable APIs.
-Modular architecture: The RTOS structure makes it easier to separate display handling, sensors, BLE, and UI logic into clean, maintainable modules.
-Scalability: As the watch grows beyond a simple prototype, Arduino’s abstractions become limiting. Zephyr provides a long‑term path for more complex features.

---

## License

MIT License.  
Feel free to use, modify, and build on this project.

Please reach out or credit me if you do use any of my schematics/ pcb layout. I would really appreciate it! would also be great to see what you come up with.
