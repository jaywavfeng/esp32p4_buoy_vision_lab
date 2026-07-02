# ESP32-P4 COCO Image Validation Path

This project now has a flash-only image validation path for the period when the TF card is unavailable.

## Selected Model

- Backend: Espressif `espressif/coco_detect` 0.3.2.
- Model: `coco_detect_yolo11n_320_s8_v3.espdl`.
- Target: ESP32-P4.
- Input: `320 x 320 x 3`.
- Classes: COCO 80 classes.
- Model bytes on board: `2,860,704`.
- Official P4 accuracy from the component README: COCO val2017 `mAP50-95 = 0.275`.
- Official P4 latency from the component README: preprocess `5.6 ms`, model `600.0 ms`, postprocess `5.8 ms`.

This is the first deployed model in this project that meets the requirement of at least one predicted frame per second on ESP32-P4. The older self-trained Coke/Sprite YOLO11/YOLO26 models are still kept for comparison, but their measured board-side latency is about `14-16 s` per frame.

## Board-Side Measurement

Measured on the ESP32-P4 board through serial self-test after embedding the COCO val2017 `demo_01..demo_04` images.
The self-test runs each image once as `warmup` and once as `measure`; the acceptance metric is `analysis_ms`, which includes JPEG decode, ESP-DL preprocessing, model inference, and postprocessing.

```text
CPU:                360 MHz
L2 cache:           256 KB, 128-byte line
Model:              coco-yolo11n-320-s8-v3-p4
Model bytes:        2,860,704
Input:              320 x 320
demo_01 measure:   inference 645 ms, analysis 697 ms, detections 8, top person 78%
demo_02 measure:   inference 597 ms, analysis 650 ms, detections 8, top person 82%
demo_03 measure:   inference 612 ms, analysis 666 ms, detections 8, top chair 78%
demo_04 measure:   inference 605 ms, analysis 660 ms, detections 7, top person 82%
Measure average:   inference 614.75 ms, analysis 668.25 ms
Latency target:    PASS, all measure analysis_ms < 1000
```

Evidence:

- Serial log: `reports/coco_video/board_coco_validation.log`
- Parsed summary: `reports/coco_video/board_coco_validation_summary.json`

`queue_total_ms` is larger during boot because it also includes queue wait and scheduling time, so the acceptance metric for steady single-frame prediction is `analysis_ms`.

The 512 KB L2 cache experiment did not boot on this board because the system could not reserve the 32 KB internal DMA pool. Keep `256 KB / 128 B` as the current fast and stable cache setting. The board revision used here also failed at 400 MHz, so CPU frequency is kept at 360 MHz.

## PC Video Validation

The PC-side video preview uses a public Intel IoT DevKit sample video and the local `yolo11n.pt` model at the same `320` input size used by the board model family.

Command:

```powershell
.\.venv_yolo\Scripts\python.exe tools\run_coco_video_validation.py --video data\coco_video\person-bicycle-car-detection.mp4 --weights yolo11n.pt --imgsz 320 --conf 0.25 --frame-stride 1 --top-k 4 --board-width 512
```

Artifacts:

- Input video: `data/coco_video/person-bicycle-car-detection.mp4`
- Annotated video: `reports/coco_video/person_bicycle_car_yolo11n_320_annotated.mp4`
- Report: `reports/coco_video/prediction_summary.json`
- Selected reference frames: `reports/coco_video/selected_frames/video_01.jpg` through `video_04.jpg`

Latest PC run:

```text
Source:           768 x 432, 647 frames, 12 FPS
Processed frames: 647
PC inference:     avg 62.969 ms, min 47.310 ms, max 379.771 ms
Classes observed: person, car, bus, cell phone, plus some small-object false positives
```

The video output is for visual preview of the end-to-end "input video, output annotated video" requirement. Board validation images are prepared from COCO val2017 instead of this video, because COCO still images provide denser and cleaner multi-object scenes.

## Firmware Video Demo

The phone validation page now uses a separate real-video sequence named
`coco_video_demo`. It embeds 16 consecutive frames from Intel's
`store-aisle-detection.mp4`, covering source frames 2400 through 3300 at a
60-frame interval. The images are 448 pixels wide, JPEG quality 78, and total
about 475 KB.

The board runs every frame through the same COCO inference queue used by the
camera. `/api/dataset/run/status` publishes the latest completed overlay while
the run is active. After all frames finish, the browser preloads the SVG
overlays and replays them at 1 FPS. Source and CC BY 4.0 attribution are stored
in `test_assets/coco_video_demo/SOURCE.md`.

## COCO Classic Board Samples

The current board-embedded demo images are selected from COCO val2017, not from the old Coke/Sprite or soda-bottle data.

Command:

```powershell
.\.venv_yolo\Scripts\python.exe tools\prepare_coco_classic_samples.py --weights yolo11n.pt --imgsz 320 --conf 0.25 --candidate-count 48 --top-k 4 --board-width 512
```

Artifacts:

- Selection report: `reports/coco_video/coco_classic_samples.json`
- PC annotated contact sheet: `reports/coco_video/coco_classic_contact_sheet.jpg`
- Full reference images: `test_assets/coco_classic/images/coco_01.jpg` through `coco_04.jpg`
- PC annotated references: `test_assets/coco_classic/images/coco_01_pc_annotated.jpg` through `coco_04_pc_annotated.jpg`
- Firmware embed paths: `test_assets/video_frames_320/images/demo_01.jpg` through `demo_04.jpg`

Selected COCO val2017 images:

```text
demo_01: 000000018380.jpg, COCO annotations 38 objects / 9 classes, PC detected 24 objects / 5 classes
demo_02: 000000189475.jpg, COCO annotations 32 objects / 10 classes, PC detected 20 objects / 5 classes
demo_03: 000000416104.jpg, COCO annotations 30 objects / 6 classes, PC detected 19 objects / 5 classes
demo_04: 000000275198.jpg, COCO annotations 26 objects / 8 classes, PC detected 15 objects / 5 classes
```

## Firmware Behavior

- Default recognition method is `coco`.
- Boot standby is disabled, so the camera starts after boot.
- TF card storage and history are disabled in defaults while the card is broken.
- Inference interval default is `0 ms`, so the firmware submits a new inference job whenever the depth-1 async queue can accept one.
- The queue is non-blocking. If inference is busy, camera capture and streaming continue and the frame is dropped instead of queued behind old frames.
- COCO inference uses ESP-DL multi-core runtime on ESP32-P4.

## Validation Images

The firmware embeds these current validation images in flash:

- `test_assets/video_frames_320/images/demo_01.jpg` from COCO val2017 `000000018380.jpg`
- `test_assets/video_frames_320/images/demo_02.jpg` from COCO val2017 `000000189475.jpg`
- `test_assets/video_frames_320/images/demo_03.jpg` from COCO val2017 `000000416104.jpg`
- `test_assets/video_frames_320/images/demo_04.jpg` from COCO val2017 `000000275198.jpg`

Use `method=coco&sample=demo_01..demo_04` for the current COCO board validation path. TinyCNN uses the separate `test_assets/tinycls_marine_demo` assets selected by board-side Top-1 checks.

## HTTP Entry Points

These routes are available once ESP-Hosted Wi-Fi is working:

- `/validate`: phone-friendly validation UI.
- `/api/validate/run?method=coco&sample=demo_01&box_min_score=50`: run board-side inference.
- `/api/validate/overlay.svg?id=<id>`: return the embedded JPEG with detected boxes.
- `/api/dataset/run/start?dataset=coco_video_demo&limit=16&stride=1`: start the embedded video run.
- `/api/dataset/frame.svg?run_id=<run>&dataset=coco_video_demo&index=<n>`: return one annotated video frame.
- `/api/status`: includes `model_info`, `coco_available`, model size, input size, detections, and measured `vision.inference_ms`.

The June 12, 2026 COM3 board test completed over the real ESP-Hosted/C6 Wi-Fi
link. Both 16-frame runs finished with `16/16` successful frames and only
`person` detections. The measured maximum `analysis_ms` values were 584 ms and
588 ms, and all 16 overlay routes returned valid annotated SVG images.

## TF Card Product Path

The replacement TF card path now keeps the same inference task and detection result structure, and adds storage around it:

1. Live camera JPEG frames are sampled into `/sdcard/esp32p4/recordings/*.mjpg`.
2. Positive detections are written to `history.jsonl` and optional JPEG snapshots.
3. Each MJPEG segment writes one `recordings.jsonl` entry and one `summaries.jsonl` periodic recognition summary.
4. The embedded `coco_video_demo` provides a storage-independent 16-frame phone demonstration.
5. Longer COCO video data can still be prepared with `tools/prepare_coco_tf_dataset.py` and copied to `/sdcard/esp32p4/datasets/coco_video`.
