.. SPDX-License-Identifier: GFDL-1.1-no-invariants-or-later

.. _pixfmt-meson-deep-color:

*********************************************
Amlogic MESON GE2D deep-color vendor formats
*********************************************

The Amlogic GE2D 2D graphics engine on G12A and newer SoCs (G12B, SM1)
has a "deep color" pipeline mode (enabled via GEN_CTRL2 bit 2) that
processes pixels with 10-bit or 12-bit internal precision. The memory
representation, however, stores only the top 8 bits of each component --
the extra precision exists only within the hardware's arithmetic
pipeline (CSC, scaler, blender) and is truncated when the result is
written back to memory.

These are vendor-specific formats with no equivalent in standard V4L2
(not compatible with V4L2_PIX_FMT_P010 or V4L2_PIX_FMT_Y210, which use
16-bit containers with 10 bits of data and 6 padding bits). They are
accepted only on hardware where the driver's ``has_deep_color`` flag
is set (G12A+); older GXL returns ``-EINVAL``.

Formats
=======

``V4L2_PIX_FMT_MESON_Y10P444`` ('AM4A')
---------------------------------------

Packed YUV 4:4:4, 3 bytes per pixel, byte order ``{Cr, Cb, Y}``. Each
byte is the top 8 bits of a 10-bit component processed by the deep
color pipeline.

.. flat-table:: MESON_Y10P444 byte layout (1 pixel)
    :header-rows: 1

    * - Byte 0
      - Byte 1
      - Byte 2
    * - Cr[9:2]
      - Cb[9:2]
      - Y[9:2]

``V4L2_PIX_FMT_MESON_Y10P422`` ('AM2A') and ``V4L2_PIX_FMT_MESON_Y12P422`` ('AM2C')
-----------------------------------------------------------------------------------

Packed YUV 4:2:2, 2 bytes per pixel. The ``Y10P422`` variant stores
top 8 bits of 10-bit components; ``Y12P422`` stores top 8 bits of
12-bit components. Internal pipeline precision differs but the memory
layout is identical.

``V4L2_PIX_FMT_MESON_R10P`` ('AMRA')
------------------------------------

Packed RGB, 3 bytes per pixel, byte order ``{R, G, B}``. Each byte is
the top 8 bits of a 10-bit component.

.. flat-table:: MESON_R10P byte layout (1 pixel)
    :header-rows: 1

    * - Byte 0
      - Byte 1
      - Byte 2
    * - R[9:2]
      - G[9:2]
      - B[9:2]

Characteristics
===============

Color space conversion
----------------------

RGB source formats convert to YUV deep-color formats with standard
BT.601 limited-range coefficients (Y in [16..235], Cb/Cr in
[16..240]). Probed on G12B 2026-04-23 with 14 controlled RGB inputs;
every output matched BT.601 limited-range predictions within +/-1
rounding.

Stride alignment
----------------

``bytesperline`` is often larger than ``width * bytes_per_pixel``.
The GE2D hardware aligns row stride to a minimum boundary (observed
minimum of 32 bytes on G12B); any unused bytes past the valid pixel
data are stride padding and must be preserved across the full
``bytesperline`` when passing buffers between components.

Use cases
=========

Suitable for:

- Internal video pipelines where the source or consumer understands
  the Amlogic-specific byte order ({Cr, Cb, Y} for Y10P444).
- Subsampled storage of the top 8 bits of a 10/12-bit frame when a
  lossy 8-bit representation is acceptable and the saved bandwidth
  is worthwhile.

NOT suitable for:

- Interop with standard V4L2 10-bit pipelines (P010, Y210 use
  different layouts with 16-bit containers).
- Zero-copy DMABUF sharing with video encoders/decoders that expect
  standard 16-bit-container 10-bit layouts.
