# Automated LEGO Sorter

An embedded LEGO sorting system built around three ESP32-based nodes. The system detects a LEGO piece, captures a still image, performs lightweight computer vision on the main ESP32, selects a destination, and indexes a five-position sorting carousel.

All processing and control run locally on the embedded system without a PC or cloud service.

## Table of Contents
- [Demonstrations & Videos](#demonstrations--videos)
- [Performance](#performance)
- [Repository Structure](#repository-structure)
- [System Architecture](#system-architecture)
- [Sorting Cycle](#sorting-cycle)
- [Computer Vision](#computer-vision)
- [Sorting Modes](#sorting-modes)
- [Hardware](#hardware)
- [Software](#software)
- [Limitations](#limitations)
- [Acknowledgments & Software References](#acknowledgments--software-references)

## Demonstrations & Videos

| LEGO Sorter 3D Model | LEGO Sorter Demo Video |
| :---: | :---: |
| [![LEGO Sorter 3D Model](https://img.youtube.com/vi/RlbAfFvu32E/hqdefault.jpg)](https://www.youtube.com/watch?v=RlbAfFvu32E) | [![LEGO Sorter Demo Video](https://img.youtube.com/vi/in9Zvj4_v-k/hqdefault.jpg)](https://www.youtube.com/watch?v=in9Zvj4_v-k) |
| [3D Model on YouTube](https://www.youtube.com/watch?v=RlbAfFvu32E) | [Demo Video on YouTube](https://www.youtube.com/watch?v=in9Zvj4_v-k) |

## Performance

| Metric | Result |
| :--- | :--- |
| Sorting accuracy | **~95%** under controlled prototype conditions |
| Typical cycle time | **~3 s / piece** |
| Processing resolution | **160 × 120** |
| Carousel | **5 positions / 72° spacing** |
| Conveyor speed | **250 steps/s** |
| Main links | **HTTP/TCP** to camera, **UDP :8888** to UI |

## Repository Structure

```text
LEGO-Sorter/
├── README.md
└── src/              # Main ESP32 firmware
```

The mechanical system was designed in **Autodesk Fusion 360**. **DXF** files were used for laser-cuting parts and **STL** files for 3D-printing components. 

## System Architecture

```text




                           +------------------+
                           |   ESP32 CYD      |
                           | UI / Settings    |
                           +--------+---------+
                                    |
                                UDP :8888
                                    |
                                    v
+------------------+   +----------------------+       +------------------+
|   ESP32-CAM      |<--|     Main ESP32       |->GPIO |  4 × Active-LOW  |
|   /capture       |   |  Vision + Control    |       |  IR sensors      |
+------------------+   +----------+-----------+       +------------------+
                                  |
                         +--------+--------+
                         |                 |
                      STEP/DIR           STEP/DIR
                         |                 |
                         v                 v
                +----------------+  +----------------+
                |  NEMA 17 #2    |  | NEMA 17 #1     |
                |  5-position    |  | conveyer       |
                |  carousel      |  |                |
                +----------------+  +----------------+
```

### Nodes

- **Main ESP32:** object detection, conveyor control, image acquisition, embedded vision, bucket selection, carousel control, and communication.
- **ESP32-CAM:** provides individual JPEG images through `/capture`.
- **ESP32 CYD:** touchscreen interface for COLOR, SHAPE, and BOTH sorting modes and result display.

## Sorting Cycle

```text
IR detected → stop conveyor →  /capture → decode →  largest component → color + shape → select bucket → index carousel → restart conveyor → repeat

```

## Computer Vision

The main ESP32 processes a reduced **160 × 120** image using `TJpg_Decoder`.

1. **Segmentation** — dark background threshold (`R+G+B <= 60`) plus saturation/value filtering.
2. **Connected components** — 8-neighbor flood-fill keeps the largest foreground component.
3. **Geometry extraction** — area, bounding box, centroid, and extreme points are calculated.
4. **Shape classification** — contour corners provide the primary decision; area comparison is used as a special disambiguation rule for the exact four-corner case.
5. **Color classification** — average object RGB is converted to a lightweight HSV-like hue representation and mapped to broad UI color groups.

The shape classifier is deterministic and rule-based.

## Sorting Modes

- **COLOR:** match detected color to a configured bucket.
- **SHAPE:** match detected shape to a configured bucket.
- **BOTH:** require both color and shape to match the same bucket.
- **Fallback:** unmatched pieces are assigned to bucket `0`.


## Hardware

- 1 × ESP32 development board
- 1 × ESP32-CAM 
- 1 × ESP32 CYD touchscreen
- 2 × NEMA 17 stepper motors
- 2 × A4988 drivers
- 4 × active-LOW IR sensors
- Laser-cut plywood structure
- Custom 3D-printed mechanical parts
- 12 V DC power supply + buck regulation for logic

## Software

- **PlatformIO / VS Code:** 
- **Arduino IDE:** 
- **C/C++**
- `TJpg_Decoder` — JPEG decoding
- `AccelStepper` — carousel control
- `Preferences` — persistent ESP32 configuration
- `WiFi` / `WiFiUDP` — networking
- `TFT_eSPI` / `XPT2046_Touchscreen` — CYD interface

## Limitations 

The current prototype is sensitive to lighting, reflections, object orientation, overlap, and the limited detail available at 160 × 120 resolution. Its deterministic vision pipeline is also less flexible than a learned model.

## Acknowledgments & Software References
  * Team & Collaboration: Co-developed as a two-person engineering project.
  * ESP32-CAM Web Server:** Built and adapted from the official Arduino IDE `CameraWebServer` example code (`ESP32` $\rightarrow$ `Camera` $\rightarrow$ `CameraWebServer`).
  * ESP32-CAM Model: [T3do on Autodesk Gallery](https://www.autodesk.com/community/gallery/project/176252/esp-32-cam)
  * NEMA 17 Stepper Motor Model: [gshoykhet on Autodesk Gallery](https://www.autodesk.com/community/gallery/project/88386/nema-17-stepper-motor-1)
# LEGO-Sorter
