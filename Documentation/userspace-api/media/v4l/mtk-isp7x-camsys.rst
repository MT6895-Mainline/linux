==========================
MediaTek ISP7x Camera System
==========================

Overview
========

The MediaTek ISP7x camera system is designed to provide a flexible, high-performance image capture pipeline for SoCs, leveraging the Linux V4L2 framework. The system is composed of multiple hardware modules (Sensor, SENINF, RAW ISP, etc.) and a firmware backend, with a clear separation of control and data flow. This document describes the hardware architecture from the perspective of the interface exposed to the firmware (backend), and how it integrates with the V4L2 CAMSYS framework.

V4L2 CAMSYS Framework Overview
==============================

The V4L2 CAMSYS framework models the camera pipeline as a set of interconnected devices and subdevices:

- **Video Device**: Exposes buffer queue/dequeue (enque/deque) operations to user space, allowing applications to stream image data.
- **Subdevice**: Represents a hardware block (e.g., sensor, SENINF, RAW ISP) and handles control/configuration operations.
- **Pads and Links**: Each subdevice has pads (input/output endpoints). Media links connect pads, representing the physical and logical data flow between hardware blocks.

The topology and link state are established at probe time and can be dynamically enabled/disabled by user space, depending on the desired pipeline configuration.

Hardware Pipeline
=================

A typical pipeline consists of:

.. code-block::

    Sensor --> SENINF --> RAW ISP --> (optional: YUV ISP) --> Memory

- **Sensor**: Provides MIPI CSI-2 data.
- **SENINF**: Receives and demultiplexes sensor data, supports virtual channel (VC) and data type (DT) routing.
- **RAW ISP**: Main image signal processor, handles demosaicing, noise reduction, and other image enhancements.
- **YUV ISP** (optional): Further processing for YUV output.
- **CAMSYS**: System controller, manages pipeline state, buffer flow, and synchronization.

Firmware Interface
==================

The firmware (backend, e.g., CCD/SCP) is responsible for:

- Calculating hardware settings (3A, tuning, DMA, FBC, Twin, etc.) for each frame.
- Generating command queues (CQ) and descriptors to program the hardware.
- Abstracting hardware-specific details from the kernel driver.

The kernel driver communicates with the firmware via an inter-processor interface (IPI), typically using RPMSG. The main interactions are:

1. **Session Management**: The kernel establishes a session with the firmware at stream-on, and destroys it at stream-off.
2. **Per-frame Request Scheduling**: For each user request (frame), the kernel sends a request to the firmware, including sensor settings, buffer addresses, and control parameters.
3. **CQ Generation**: The firmware generates the necessary command queues and returns them to the kernel.
4. **Event Notification**: The firmware notifies the kernel of CQ completion, errors, or other events.

Request Scheduling and Buffer Flow
==================================

The request scheduling flow is as follows:

1. **User space** submits a request (with buffer and control settings) via V4L2.
2. **PipeMgr** (user space) collects requests, groups them by file descriptor, and sends them to the kernel CAMSYS driver.
3. **CAMSYS** schedules requests, manages buffer flow, and coordinates with the firmware for hardware programming.
4. **Firmware** processes each request, generates CQ, and signals completion.
5. **Kernel** dequeues the buffer and notifies user space.

The system supports per-frame control, allowing sensor settings and ISP parameters to be updated on a frame-by-frame basis.

Media Controller Topology
=========================

The media controller graph models the hardware pipeline as a set of entities (video devices, subdevices) connected by links. Each entity exposes pads, and links are established at probe time based on the hardware configuration and device tree.

.. code-block::

    +---------+     +--------+     +--------+     +------+
    | Sensor  | --> | SENINF | --> | RAW ISP| --> | Video|
    +---------+     +--------+     +--------+     +------+

- Solid lines: active links (enabled)
- Dashed lines: inactive links (disabled)

User space can query and configure the topology using standard V4L2 and media controller APIs.

Backend Abstraction
===================

The backend firmware abstracts hardware-specific programming, allowing the kernel driver to remain generic and maintainable. All hardware register programming, tuning, and advanced features are handled in the firmware, with the kernel acting as a control and data flow manager.

This separation ensures that proprietary hardware details remain in the firmware, while the kernel driver exposes a standard V4L2 interface.

Summary
=======

- The MediaTek ISP7x camera system is built on the V4L2 media controller framework, with a modular pipeline of video devices and subdevices.
- The kernel driver manages pipeline state, buffer flow, and per-frame control, while delegating hardware programming to the backend firmware via IPI.
- The firmware interface is session-based, supports per-frame CQ generation, and abstracts all hardware-specific details from the kernel.
- The media controller graph models the hardware pipeline, allowing flexible configuration and dynamic topology management.