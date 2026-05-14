# Analysis: HCODEC H.264 Encode on GXL (S905X)

## Current Slot: 3 (HCODEC H.264 encode)
## Board: aml-s905x-cc-v2-2gb (gxl), 192.168.0.157
## Status: BLOCKED -- vermagic mismatch causes kernel crashes at 640x480+

## Key Findings

- (x5) Per-frame PSNR varies wildly for IDR-only encoding at 640x480+: frames range from 17 dB to 46 dB within a single session. All frames are IDR (GOP=1) so should be identical quality.
- (x3) Determinism fails consistently -- encoding same input twice produces different bitstreams at all resolutions.
- (x3) Board crashes (kernel panic + SSH disconnect) when encoding 640x480 with vermagic-mismatched module. 320x240 works. The force-loaded module with wrong vermagic is causing struct layout corruption.
- (x2) 320x240 encodes perfectly: 42 dB PSNR, consistent per-frame quality.
- (x2) Constant-color frames (Y=128) encode perfectly at all resolutions.
- (x1) QP changes (min_qp 26 vs 35 vs 45) do NOT change output size (~59KB for 640x480x5). Firmware may be ignoring QP register or CBR tables override it.
- (x1) With 4 v4l2 output buffers, some frames encode STALE DATA. 2 buffers fixes stale data but not quality.
- (x1) Firmware DOES read QPPICTURE at init (confirmed via disassembly: mfcr r8, 0x0acb -> sta [0x1]).
- (x1) Firmware reads ENCODER_STATUS to determine IDR vs P-frame command.
- (x1) CBR rate control works at 1-2 Mbps but misses at 4 Mbps (55%).
- (x1) Frame count is always correct (30/30).
- (x1) Bitstream buffer (1MB) is NOT overflowing -- per-frame sizes are ~11KB at 640x480.

## Checked Locations

- BSP encoder.c per-frame flow -- matches our register sequence
- BSP buffer layout -- identical (DCT 8MB, dec0/dec1 3MB, assist 0.75MB, bitstream 1MB)
- DMA coherent allocation -- confirmed
- IRQ vs polling race -- fixed (polling-only now)
- Full DOS reset + firmware reload -- implemented
- Buffer zeroing -- tried, causes crashes (20MB memset on DMA coherent too slow)
- Firmware disassembly (gxl_h264_enc.bin.dasm) -- confirmed QPPICTURE read at startup
- QP control path in venc.c -- ctx->qp IS set from v4l2 control

## Dead Ends

- Full DOS reset (0xffffffff) + firmware reload: Does not fix PSNR or determinism
- Buffer zeroing (memset 0): Causes kernel crashes (too much overhead on DMA coherent)
- Polling-only completion: Fixes stale-buffer issue, not quality
- Different QP values: Output size unchanged -- firmware ignoring?

## Blocker

The module has vermagic `6.12.75-g0695a2ee190f` but the running kernel is `6.12.75-gb467231ae45c`. The module loads with --force but struct layout differences cause crashes at 640x480+. To proceed:

1. Need to build a full kernel DEB from the gxl-enc worktree and deploy it
2. OR need to cherry-pick the encoder changes back to the linux-6.12.y-lc branch and rebuild from there

The builder's deploy.sh cannot handle worktree branches (it tries to checkout linux-6.12.y-lc which is locked by the main worktree). Manual kernel build via `make` fails because the output Makefile references the wrong source tree. The fix is to either:
- Use `make bindeb-pkg` after fixing the META_SUFFIX issue
- Manually update the output dir's Makefile and build a full kernel image
- Cherry-pick to linux-6.12.y-lc and use the normal builder workflow

## Commits Made

1. `0695a2ee190f` -- hcodec: add per-frame firmware reload with full DOS reset
2. `655f3a8af14f` -- hcodec: fix encode completion race and add DOS reset per frame

## Test Results (with vermagic mismatch, may not be reliable)

| Resolution | Avg PSNR (Y) | Min PSNR | Status |
|-----------|-------------|----------|--------|
| 320x240   | 42.7 dB     | 42.5 dB  | PASS (Round 1) |
| 640x480   | 24-28 dB    | 18-20 dB | CRASH / FAIL |
| 1280x720  | 17 dB       | 15 dB    | CRASH / FAIL |
| 1920x1080 | 17 dB       | 14 dB    | CRASH / FAIL |

## Next Steps (priority order)

1. **Deploy matching kernel** -- cherry-pick encoder commits to linux-6.12.y-lc and rebuild
2. **Re-run tests without vermagic mismatch** -- crashes may be entirely due to mismatch
3. **If still failing**: investigate firmware QDCT pipeline timing at higher MB counts
4. **If QP still ignored**: check if CBR firmware override is active by default

## Session State

- Worktree: /home/dxue/git/linux-worktree/gxl-enc
- Branch: topic/gxl/hcodec-h264-encode
- Board: 192.168.0.157 (recovers from crashes via watchdog)
- Running kernel: 6.12.75-gb467231ae45c
- Module vermagic: 6.12.75-g0695a2ee190f (MISMATCH)
- Last test: 2026-03-19 ~11:36 UTC
