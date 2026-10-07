==========================================
MediaTek ISP Camsys User Space Interface
==========================================

Overview
--------

The MediaTek ISP camsys driver provides a set of custom V4L2 controls for user space
applications to configure and query the ISP hardware. These controls allow
fine-grained management of resource allocation, feature enablement, and
hardware-specific settings.

Typical Usage Flow
------------------

1. Open the video device node (e.g., ``/dev/video0``).
2. Set required V4L2 controls using ``VIDIOC_S_CTRL`` or ``VIDIOC_S_EXT_CTRLS``.
3. Set up video format and buffers (``VIDIOC_S_FMT``, ``VIDIOC_REQBUFS``, etc.).
4. Start streaming with ``VIDIOC_STREAMON``.
5. Queue and dequeue buffers for image processing (``VIDIOC_QBUF``, ``VIDIOC_DQBUF``).
6. Stop streaming with ``VIDIOC_STREAMOFF``.

Control List and Documentation
-----------------------------

Below is a list of custom V4L2 controls provided by the camsys driver, with detailed descriptions.

.. list-table::
   :header-rows: 1

   * - Control Name
     - Type
     - Description
     - Valid Values / Range
     - When to Set

   * - ``V4L2_CID_MTK_CAM_USED_ENGINE_LIMIT``
     - Integer
     - Limits the maximum number of RAW engines (hardware pipelines) that can be used for this stream.
     - 1 ~ 2 (default: 2)
     - Before streaming

   * - ``V4L2_CID_MTK_CAM_BIN_LIMIT``
     - Integer
     - Sets the maximum binning ratio allowed for the stream.
     - 0 ~ 0xFFF (default: 0)
     - Before streaming

   * - ``V4L2_CID_MTK_CAM_FRZ_LIMIT``
     - Integer
     - Sets the maximum resizer (FRZ) ratio allowed (as a percentage).
     - 70 ~ 100 (default: 100)
     - Before streaming

   * - ``V4L2_CID_MTK_CAM_RESOURCE_PLAN_POLICY``
     - Integer (Enum)
     - Selects the resource planning policy for the stream.
     - 0: Default, 1: Power-saving, 2: Performance, ... (0~10)
     - Before streaming

   * - ``V4L2_CID_MTK_CAM_USED_ENGINE``
     - Integer
     - Reports or sets the number of RAW engines actually used for this stream after negotiation.
     - 1 ~ 2 (default: 2)
     - After resource negotiation

   * - ``V4L2_CID_MTK_CAM_BIN``
     - Boolean
     - Enables or disables binning for the stream.
     - 0 (disable), 1 (enable, default)
     - Before streaming

   * - ``V4L2_CID_MTK_CAM_FRZ``
     - Integer
     - Sets the resizer (FRZ) ratio (as a percentage) for the stream.
     - 70 ~ 100 (default: 100)
     - Before streaming

   * - ``V4L2_CID_MTK_CAM_USED_ENGINE_TRY``
     - Integer
     - Reports the number of RAW engines that would be used for the current "try" format negotiation.
     - 1 ~ 2 (default: 2)
     - After VIDIOC_TRY_FMT

   * - ``V4L2_CID_MTK_CAM_BIN_TRY``
     - Boolean
     - Reports whether binning would be enabled for the current "try" format negotiation.
     - 0, 1 (default: 1)
     - After VIDIOC_TRY_FMT

   * - ``V4L2_CID_MTK_CAM_FRZ_TRY``
     - Integer
     - Reports the resizer ratio that would be used for the current "try" format negotiation.
     - 70 ~ 100 (default: 100)
     - After VIDIOC_TRY_FMT

   * - ``V4L2_CID_MTK_CAM_PIXEL_RATE``
     - Integer64
     - Sets the pixel rate (in pixels per second) of the sensor for resource calculation.
     - 0 ~ 0xFFFFFFFF
     - Before streaming

   * - ``V4L2_CID_MTK_CAM_FEATURE``
     - Bitmask (Integer64)
     - Bitmask of enabled features for the stream (e.g., HDR, 3A, etc.). See driver header for details.
     - Bitmask (0 ~ RAW_FUNCTION_END)
     - Before streaming or per-frame

   * - ``V4L2_CID_MTK_CAM_SYNC_ID``
     - Integer64
     - Frame synchronization ID for multi-sensor or multi-pipeline synchronization.
     - -1 ~ 0x7FFFFFFF (default: -1)
     - Before streaming or per-frame

   * - ``V4L2_CID_MTK_CAM_RAW_PATH_SELECT``
     - Integer
     - Selects the RAW path (e.g., main or secondary) for the stream.
     - 0, 1 (default: 1)
     - Before streaming

   * - ``V4L2_CID_MTK_CAM_HSF_EN``
     - Integer
     - Enables or disables hardware secure flow (HSF) for the stream.
     - 0 (disable), 1 (enable)
     - Before streaming

   * - ``V4L2_CID_MTK_CAM_PDE_INFO``
     - Integer (struct mtk_cam_pde_info)
     - Provides or sets phase detection (PDE) information for PDAF/AF features.
     - See struct definition
     - Before streaming or per-frame

   * - ``V4L2_CID_MTK_CAM_MSTREAM_EXPOSURE``
     - Integer/Integer64
     - Controls multi-stream exposure settings for advanced HDR or multi-exposure modes.
     - See driver header
     - Before streaming or per-frame

   * - ``V4L2_CID_MTK_CAM_RAW_RESOURCE_CALC``
     - Compound (struct mtk_cam_resource)
     - Triggers resource calculation for the current format and settings.
     - See struct definition
     - During format negotiation

   * - ``V4L2_CID_MTK_CAM_TG_FLASH_CFG``
     - Integer/struct
     - Configures timing generator (TG) and flash synchronization.
     - See driver header
     - Before streaming or per-frame

   * - ``V4L2_CID_MTK_CAM_RAW_RESOURCE_UPDATE``
     - Integer
     - Notifies the driver of a sensor mode update (e.g., seamless switch).
     - 0 ~ 0xF (default: 0)
     - When sensor mode changes

   * - ``V4L2_CID_MTK_CAM_CAMSYS_HW_MODE``
     - Integer64
     - Selects the hardware mode for the CAMSYS pipeline (e.g., normal, secure, etc.).
     - 0 ~ 0x7FFFFFFF
     - Before streaming

Control Usage Patterns
---------------------

- Controls with ``_TRY`` suffix are used to query the result of a "try" format negotiation (``VIDIOC_TRY_FMT``), without committing changes.
- Controls without ``_TRY`` are used to set or get the actual configuration (``VIDIOC_S_FMT``, ``VIDIOC_G_CTRL``).
- Use ``V4L2_CID_MTK_CAM_RAW_RESOURCE_CALC`` to check if the requested format and settings are feasible, and to get recommended values.
- Use ``V4L2_CID_MTK_CAM_RAW_RESOURCE_UPDATE`` to notify the driver of a sensor mode change (e.g., seamless switch).
- Use ``V4L2_CID_MTK_CAM_SYNC_ID`` to synchronize frames across multiple pipelines.
- Use ``V4L2_CID_MTK_CAM_FEATURE`` to enable/disable advanced features (bitmask).
- Use ``V4L2_CID_MTK_CAM_PDE_INFO`` to set/get phase detection information for PDAF/AF.
- Use ``V4L2_CID_MTK_CAM_PIXEL_RATE`` to inform the driver of the sensor's pixel rate for bandwidth/resource calculation.

Example: Setting a Control
--------------------------

.. code-block:: c

   int fd = open("/dev/video0", O_RDWR);
   struct v4l2_control ctrl = {
       .id = V4L2_CID_MTK_CAM_BIN_LIMIT,
       .value = 2,
   };
   if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0) {
       perror("Failed to set BIN_LIMIT");
   }

Notes
-----

- All controls should be set before starting the video stream unless otherwise specified.
- Refer to the driver header file for detailed struct definitions and bitmask values.
- The driver will reject invalid values or unsupported configurations with -EINVAL.

References
----------

- V4L2 documentation: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2.html