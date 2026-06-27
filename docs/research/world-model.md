# World Model Research Brief

**For:** Rebuilt-2026 autonomy "orchestration brain" — the shared, read-only world model (blackboard / state estimate).
**Date:** 2026-06-26. **Bias:** simple, open-weight, permissively licensed, deployable on a small coprocessor (Orange Pi 5 / Jetson Orin Nano / mini-PC).
**Scope:** research only — no robot code here. Every claim carries a URL.

---

## Executive summary

Build the world model as **one flat, immutable Java `record` rebuilt once per ~20 ms loop** (the WPILib `robotPeriodic`/`periodic` cadence): sensing writes it, decisions only read it, and every consumer sees the *same* snapshot for the whole tick. This is the classic **blackboard** pattern (Hearsay-II / Nii) realized the way BehaviorTree.CPP and ROS `robot_localization` do it. Wire the fields you already have **today** (pose, velocity, mechanism states, match/alliance, vision targets + confidence) and leave the parts you cannot yet sense — **opponent positions, field game-piece positions, and possession** — as explicit **null sentinels / empty lists**, because a guessed value is worse than an absent one. When you do add opponent/game-piece sensing, the realistic FRC stack is a **PhotonVision (Orange Pi 5 + RK3588 NPU) or Limelight+Coral YOLO detector** for the 2D box, turned into a field position by the **classic pinhole / ground-plane homography** math you already own from AprilTag calibration (skip neural monocular depth — it is slow on edge hardware and not metric). Possession can start **sensorless** via intake **stator-current spike + velocity dip**, behind a clean `hasPiece()` seam so a beam-break drops in later. Finally, turn on the AprilTag pose fusion you already built via **`SwerveDrivePoseEstimator` + gated `addVisionMeasurement()`** — trust gyro for heading, vision for translation, reject high-ambiguity / fast-motion frames.

---

## 1. Architecture — the blackboard / world-model shell

### What the research says

- **Blackboard pattern, origin.** First practical system was Hearsay-II speech understanding (1971–76): many independent "knowledge source" experts read/refine hypotheses on one shared structure with no fixed control flow. [Wikipedia: Blackboard design pattern](https://en.wikipedia.org/wiki/Blackboard_(design_pattern)). The canonical survey is **H. Penny Nii, "Blackboard Systems," AI Magazine, 1986** — blackboard = central shared store + independent knowledge sources + a control component that decides ordering. [AI Magazine 1986](https://onlinelibrary.wiley.com/doi/abs/10.1609/aimag.v7i2.537), [Stanford TR CS-TR-86-1123](http://i.stanford.edu/pub/cstr/reports/cs/tr/86/1123/CS-TR-86-1123.pdf). Control-as-a-first-class-concern comes from **Hayes-Roth, "A blackboard architecture for control," Artificial Intelligence, 1985** (cited in the Wikipedia article above).
- **Behavior Trees (the decision layer that reads the blackboard).** Canonical reference: **Colledanchise & Ögren, "Behavior Trees in Robotics and AI: An Introduction"** — arXiv 1709.00084 (2017, rev. 2022), CRC Press book 2018. [arXiv 1709.00084](https://arxiv.org/abs/1709.00084). The most-used implementation, **BehaviorTree.CPP** (basis of ROS 2 Nav2), makes the **blackboard a typed key-value store**: nodes declare input ports (read) and output ports (write); a global blackboard is reachable everywhere. [GitHub](https://github.com/BehaviorTree/BehaviorTree.CPP), [ports tutorial](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_02_basic_ports/).
- **Utility AI (an alternative/complement to BTs for action selection).** **Dave Mark, *Behavioral Mathematics for Game AI* (2009)** and **Mark & Dill, "Improving AI Decision Modeling Through Utility Theory," GDC 2010** ([slides PDF](https://media.gdcvault.com/gdc10/slides/MarkDill_ImprovingAIUtilityTheory.pdf)); summary at [Wikipedia: Utility system](https://en.wikipedia.org/wiki/Utility_system). A utility scorer reads the world-model snapshot (distance, possession, match time) and numerically scores candidate actions — exactly the decision layer that sits on top of a blackboard.
- **ROS single-fused-state pattern.** `robot_localization` fuses an arbitrary number of sensors (odometry, IMU, pose, twist) into one 15-DOF state via EKF/UKF. [package](https://index.ros.org/p/robot_localization/), [ROSCon 2015 tutorial](https://roscon.ros.org/2015/presentations/robot_localization.pdf). The FRC analog is WPILib's `SwerveDrivePoseEstimator` (see §5).
- **FRC-specific prior art.** **Team 254's `RobotState`** is the most-cited FRC "shared world model": a central, time-indexed pose store (interpolating tree-map keyed by timestamp) so vision and aiming query "where were we when this frame was captured." [Team254 RobotState.java](https://github.com/Team254/FRC-2017-Public/blob/master/src/com/team254/frc2017/RobotState.java), [CD explainer](https://www.chiefdelphi.com/t/understanding-254s-robot-field-position/344534). The natural refresh hook is the **WPILib command scheduler loop (~20 ms, all modes)** that calls every subsystem's `periodic()`. [scheduler docs](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html), [subsystem docs](https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html). Note: a *strictly immutable rebuilt-per-loop* FRC write-up is thin in public sources — most teams use a mutable 254-style singleton; the immutable discipline below is the blackboard/BT literature applied to the WPILib loop.

### What we'd actually use

A single immutable Java `record WorldModel`, rebuilt once per loop and published behind one `volatile`/`AtomicReference`. Readers grab the reference once at the start of their logic, so there is no mid-loop tearing. Use `List.copyOf(...)` for collections (unmodifiable). Recommended fields, flat and small:

| Group | Fields | Source today |
|---|---|---|
| Timing/match | `timestampSeconds`, `matchTimeRemaining`, `phase` (AUTO/TELEOP/ENDGAME/DISABLED), `alliance` | DriverStation |
| Pose & motion | `Pose2d fieldToRobot`, `ChassisSpeeds velocity`, `poseConfidence` | `SwerveDrivePoseEstimator` (§5) |
| Mechanism states | `turretAngle`, `hoodAngle`, `shooterRPS`, `intakeDeployed`, `atSetpoint` flags | `Mechanisms` layer, read once/loop |
| Possession | `boolean hasGamePiece` (+ `count`) | sensorless proxy (§4), **null/false sentinel until wired** |
| Vision targets | `List<VisionTarget>` (`tagId`, `Translation2d`, `ambiguity`, `timestamp`, `trusted`) | PerceptionProvider |
| Opponents | `List<Translation2d> opponents` (+ timestamps) | **empty list sentinel until sensed (§2–3)** |
| Field game pieces | `List<Translation2d> gamePieces` (+ confidence) | **empty list sentinel until sensed (§2–3)** |

Refresh: in one early place each loop (a `RobotState.update()` from `robotPeriodic`, or a dedicated subsystem's `periodic()` that runs first), gather pose-estimator output + mechanism reads + sensor booleans + newest vision frame, build a brand-new `WorldModel`, publish it. The decision layer (`SkillInterpreter` / a utility scorer / a BT) reads `RobotState.current()`. Keep a small ring buffer of recent snapshots *only* if you need latency compensation beyond what the pose estimator already does — don't over-build.

---

## 2. Opponent / robot & game-piece detection (open-weight, edge)

### What the research says

License is the make-or-break axis: AGPL/GPL copyleft attaches to anything you distribute (AGPL even to networked use), while Apache-2.0/MIT are clean. **There is no permissively-licensed model in the mainstream "YOLO" lineage** — that gap is exactly what RT-DETR/RF-DETR fill.

- **Ultralytics (YOLOv5/v8/YOLO11/v10-packaged): AGPL-3.0**, Enterprise license needed to stay closed. [Ultralytics license](https://www.ultralytics.com/license), [YOLO11 docs](https://docs.ultralytics.com/models/yolo11). Easiest to train, best-supported on FRC hardware; YOLO11n is ~2.6M params / a few MB and hits ~120 FPS on a Jetson Orin Nano at 640² TensorRT FP16. [Jetson blog](https://www.ultralytics.com/blog/ultralytics-yolo11-on-nvidia-jetson-orin-nano-super-fast-and-efficient).
- **YOLOv7/v9 (GPL-3.0), YOLOv10 (AGPL-3.0)** — all copyleft; v10 adds NMS-free end-to-end (lower, more deterministic latency). [license guide](https://medium.com/@bingbai.jp/yolo-model-licenses-a-developers-guide-da722767b6f8), [v10 LICENSE](https://github.com/THU-MIG/yolov10/blob/main/LICENSE).
- **RT-DETR / RT-DETRv2 (Apache-2.0)** — Baidu/Peking U real-time DETR; v2 is in HF Transformers (Feb 2025), easy to fine-tune, NMS-free. [GitHub](https://github.com/lyuwenyu/RT-DETR), [HF v2 weights](https://huggingface.co/PekingU/rtdetr_v2_r18vd).
- **RF-DETR (Apache-2.0 for Nano/Small/Base/Large)** — Roboflow; **purpose-built for fine-tuning small datasets**, exports ONNX/TFLite; Nano = 2.3 ms/frame on a T4 (TensorRT FP16). Best permissive + edge + easy-to-train pick. [GitHub](https://github.com/roboflow/rf-detr), [docs](https://rfdetr.roboflow.com/).
- **Open-vocab / zero-shot (YOLO-World GPL-3.0; Grounding DINO; OWL-ViT)** — great for "no labels," but too slow for an FRC loop on small hardware and/or GPL. Use them **offline to auto-label** a dataset, then distill into a small YOLO/RF-DETR. [YOLO-World](https://github.com/AILab-CVC/YOLO-World), [Grounding DINO 1.5 Edge >10 FPS on Orin NX](https://arxiv.org/abs/2405.10300), [OWL-ViT too heavy for edge](https://roboflow.com/compare/grounding-dino-vs-owl-vit).
- **What FRC teams actually run:**
  - **PhotonVision object detection** on Orange Pi 5 (RK3588 NPU): supports **only 640×640 Ultralytics YOLOv5/v8/YOLO11 converted to `.rknn` and quantized** — it converts, does not train; default model is AGPL. [about](https://docs.photonvision.org/en/latest/docs/objectDetection/about-object-detection.html), [OPi setup](https://docs.photonvision.org/en/latest/docs/objectDetection/opi.html). Community **2026 REBUILT "Fuel" models already exist**. [CD thread](https://www.chiefdelphi.com/t/unofficial-2026-photon-vision-fuel-detection-model-orange-pi-only-download/511990).
  - **Limelight Neural Detector** via Google Coral (4 TOPS, INT8): plug Coral, set pipeline to "Neural Detector," free trainer + Roboflow export. [getting started](https://docs.limelightvision.io/docs/docs-limelight/pipeline-neural/getting-started-with-neural-networks), [trainer](https://tools.limelightvision.io/neural-network-trainer).
  - **Pre-trained FRC datasets on Roboflow Universe** (fine-tunable, mostly export YOLOv5/8/11): [FRC robot detection ~160 imgs](https://universe.roboflow.com/a-xvsqd/frc-robot-detection), [FRC 2026 Fuel ~706 imgs](https://universe.roboflow.com/frcroboraiders/frc-2026-fuel-sbrdk), [FRC 2026 ReBuilt Fuel](https://universe.roboflow.com/myworkspace-mliyg/frc-2026-rebuilt-fuel-detection/dataset/1). You don't start from zero.

### Comparison table (open-weight detectors)

| Model | License | Sizes | Edge speed (hardware) | Fine-tune effort | Link |
|---|---|---|---|---|---|
| YOLO11 (Ultralytics) | **AGPL-3.0** | n(~2.6M)/s/m/l/x | n≈120, s≈100, m≈61 FPS (Orin Nano, TRT FP16, 640²) | Very easy | [docs](https://docs.ultralytics.com/models/yolo11) |
| YOLOv8 (Ultralytics) | **AGPL-3.0** | n/s/m/l/x | Real-time on Orin Nano / RK3588 | Very easy | [docs](https://docs.ultralytics.com/) |
| YOLOv5 (Ultralytics) | **AGPL-3.0** | n/s/m/l/x | Real-time; RK3588 + Coral via FRC tools | Very easy | [GitHub](https://github.com/ultralytics/yolov5) |
| YOLOv10 (THU) | **AGPL-3.0** | n/s/m/b/l/x, NMS-free | Slightly faster than v8; lower latency | Easy | [LICENSE](https://github.com/THU-MIG/yolov10/blob/main/LICENSE) |
| YOLOv7 / YOLOv9 | **GPL-3.0** | tiny…x / t…e | Real-time on GPU | Moderate | [guide](https://medium.com/@bingbai.jp/yolo-model-licenses-a-developers-guide-da722767b6f8) |
| **RF-DETR** (Roboflow) | **Apache-2.0** | Nano/Small/Base(~29M)/Large | Nano 2.3 ms (~430 FPS) T4 TRT FP16; ONNX/TFLite for edge | **Easy (built for it)** | [GitHub](https://github.com/roboflow/rf-detr) |
| **RT-DETRv2** (PekingU/Baidu) | **Apache-2.0** | r18/r34/r50/r101 | Real-time on GPU/Orin; r18 lightest | Easy (HF Transformers) | [HF](https://huggingface.co/PekingU/rtdetr_v2_r18vd) |
| YOLO-World (Tencent) | **GPL-3.0** | open-vocab s/m/l/x | ~52 FPS V100; heavy on small HW | Prompt or fine-tune | [GitHub](https://github.com/AILab-CVC/YOLO-World) |
| Grounding DINO 1.5 Edge | gated/API | Tiny/Base + Edge | >10 FPS @640² Orin NX (TRT) | Zero-shot | [arXiv](https://arxiv.org/abs/2405.10300) |
| OWL-ViT / OWLv2 | Apache-2.0 | ViT-B/L | Too slow for edge | Zero-shot | [compare](https://roboflow.com/compare/grounding-dino-vs-owl-vit) |

### What we'd actually use (ranked)

1. **Just make it work this season:** Orange Pi 5 + **PhotonVision** running a **YOLO11n `.rknn`** model, or **Limelight + Coral** neural detector. Both real-time, FRC-supported, have existing 2026 Fuel models. Only watch-item: AGPL (fine for open robot code; flag only if you ever close-source).
2. **If license cleanliness matters:** fine-tune **RF-DETR-Nano (Apache-2.0)** on a Roboflow FRC dataset; deploy on Jetson Orin Nano / mini-PC via ONNX/TensorRT (RK3588/PhotonVision won't ingest it — YOLOv5/8/11 only — so this means your own inference runtime).
3. **No labels yet:** auto-label with Grounding DINO / YOLO-World offline, then train a small YOLO11n or RF-DETR-Nano for the on-robot detector. Never run open-vocab models live on the coprocessor.

---

## 3. Monocular depth / ranging — 2D box → field position

### What the research says

The killer fact: most open monocular depth models output **relative (affine-invariant) depth** — "farther than," not "X meters" — so they need external scaling to give metric range anyway. Even "metric" models depend on the camera focal length you'd feed them.

- **Depth Anything V2** (NeurIPS 2024): **Small (24.8M) is Apache-2.0; Base/Large/Giant are CC-BY-NC-4.0**; flagship outputs relative depth, metric needs separate fine-tuned checkpoints. [GitHub](https://github.com/DepthAnything/Depth-Anything-V2), [paper](https://arxiv.org/abs/2406.09414). Speed: Small ViT-S ≈ **98.6 ms (10.1 FPS) on Jetson Orin NX** — and FRC coprocessors are weaker than an Orin NX. [edge timing](https://arxiv.org/pdf/2603.26250).
- **MiDaS** (relative, explicitly up-to-scale-and-shift) [GitHub](https://github.com/isl-org/MiDaS); **ZoeDepth** (metric) [GitHub](https://github.com/isl-org/ZoeDepth); **Metric3D v2** (metric, *requires correct focal length* → still pinhole-tied) [GitHub](https://github.com/YvanYin/Metric3D), [paper](https://arxiv.org/abs/2404.15506); **Marigold** (diffusion, relative, slowest — non-real-time on edge) [GitHub](https://github.com/prs-eth/marigold). Overview: [HF guide](https://huggingface.co/blog/Isayoften/monocular-depth-estimation-guide).
- **Classic known-size pinhole:** `distance = focal_px × real_height_m / pixel_height`. Calibrate `focal_px` once with a known object at a known distance. [PyImageSearch (2015)](https://pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/), peer-reviewed error analysis [Leorna & Brinkman 2022](https://besjournals.onlinelibrary.wiley.com/doi/full/10.1111/2041-210X.13880). Bearing = `atan2(u − cx, focal_px)` from the box-center column.
- **Ground-plane homography / IPM:** robots sit on the floor, so the **bottom edge of the box is a point on Z=0**; with known camera height + pitch a 3×3 homography maps it straight to floor (X,Y) — and **needs no knowledge of the opponent's size**. [OpenCV IPM Q&A](https://answers.opencv.org/question/2309/inverse-perspective-mapping/), [tutorial](https://nilesh0109.medium.com/camera-image-perspective-transformation-to-different-plane-using-opencv-5e389dd56527). Sensitive to pitch/height error (fine on a flat FRC field). [error note](https://arxiv.org/pdf/1809.01268).
- **It's the same math as AprilTag solvePnP:** all of the above use the same intrinsic matrix K you already obtain from AprilTag calibration; a known-size object is just PnP with assumed 3D points, ground homography is PnP specialized to Z=0. [PhotonVision VisionEstimation](https://javadocs.photonvision.org/release/src-html/org/photonvision/estimation/VisionEstimation.html).

### What we'd actually use (ranked)

1. **Ground-plane homography (IPM) off the box bottom edge.** Flat field, fixed measurable camera height/pitch, reuses AprilTag intrinsics, one 3×3 matrix-multiply per detection (microseconds, no GPU), and doesn't need the opponent's size. Risk: the box bottom must be the real floor contact (occlusion/clipping biases range).
2. **Known-size pinhole** using the FRC-mandated **bumper height (~6 in / 0.152 m)** as the known dimension. Equally cheap, slightly more robust to camera-pitch error; cross-check it against #1.
3. **Neural monocular depth — skip it here.** Not metric without anchoring to the same geometry you'd use anyway, single-digit FPS on edge for the Small model, and licensing limits the good variants. Reserve only for dense free-space mapping of unknown obstacles, never for ranging one robot whose size/floor contact you can model.

Compute **range** from #1 (or #2) and **bearing** from the box-center column; fuse the two range estimates for robustness.

---

## 4. Possession / game-piece sensing (software, no beam-break yet)

### What the research says

- **Intake current-spike** is battle-tested: grabbing a piece does mechanical work, so motor current rises above the free-spin baseline; latch possession on threshold. Canonical writeup measured ~18 A free → ~22 A with a ball, latching a boolean. [Mr. McTavish, "Using motors as a sensor in FRC" (2021)](https://mrmctavish.wordpress.com/2021/02/18/using-motors-as-a-sensor-in-frc/), [CD: Using Output Current for Intake Object Detection](https://www.chiefdelphi.com/t/using-output-current-for-intake-object-detection/430825).
- **Watch stator, not supply current** — stator is the current through the windings and the better torque/load proxy. [BoVLB FRC Tips](https://bovlb.github.io/frc-tips/burnout/), [Phoenix 6 current control](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html). Gotcha: if your stator current **limit** is below the possession spike, the controller clamps the very signal you're detecting. [FRC 1072 2024 build thread](https://www.chiefdelphi.com/t/frc-1072-2024-build-thread/449143).
- **Velocity-dip** is the complementary signal: a loaded roller drags below its setpoint. Phoenix 6 has a first-class `VelocityFilterTimeConstant` to pre-smooth before thresholding. [Phoenix 6 changelog/API](https://api.ctr-electronics.com/changelog.html), mechanism rationale [gm0 Active Intake](https://gm0.org/en/latest/docs/common-mechanisms/active-intake/index.html).
- **Vision intake confirmation** is weaker (held pieces are often self-occluded; lighting/latency failure modes) — use it for **acquisition aiming**, not as the possession latch. [PhotonVision target data](https://docs.photonvision.org/en/latest/docs/programming/photonlib/using-target-data.html).
- **Beam-break is the gold standard** — clean binary, near-zero latency, independent of load, and tells you *where* the piece is; current/velocity is a free but inferential proxy (false-triggers on inrush/jams, false-negatives on light pieces). [frc-4931 Sensors wiki](https://github.com/frc-4931/2014/wiki/Sensors).

### What we'd actually use

Primary = **intake stator-current threshold**, debounced ~3–5 loops, armed only while the intake is commanded and past spin-up. Secondary/confirming = **velocity dip** on the same roller (pre-smooth with `VelocityFilterTimeConstant`). Make the latch **sticky** (hold "have piece" until an eject/feed event). Expose a clean boolean seam `hasPiece()` so a real beam-break drops in later with zero downstream changes. Keep vision for acquisition aiming only.

---

## 5. Sensor fusion for pose (AprilTag estimation is built but off)

### What the research says

- **`SwerveDrivePoseEstimator`** is a drop-in for odometry that Kalman-fuses encoders + gyro with **latency-compensated** vision; state is `Pose2d [x, y, θ]`. Call `update()` every loop; `addVisionMeasurement()` as often or rarely as you like. Defaults: state 0.1/0.1 m, 0.1 rad; vision 0.9/0.9 m, 0.9 rad — **smaller std dev = trusted more**. [WPILib pose estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html), [API](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/estimator/SwerveDrivePoseEstimator.html).
- **Pass the image capture timestamp** to `addVisionMeasurement(pose, timestampSeconds, stdDevs)`, never `getFPGATimestamp()` — the estimator replays odometry from that time. WPILib guidance: **vision heading std dev very large, gyro heading std dev small (gyro owns rotation), scale vision x/y std dev with tag distance**. [PhotonVision pose example](https://docs.photonvision.org/en/latest/docs/examples/poseest.html), [advanced strategies](https://docs.photonvision.org/en/latest/docs/integration/advancedStrategies.html), [PhotonPoseEstimator](https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html).
- **MultiTag** (`MULTI_TAG_PNP_ON_COPROCESSOR`) is the most accurate strategy; fall back to lowest-ambiguity single tag. **Reject ambiguity > 0.2**; caveat: **MultiTag reports ambiguity = 0**, which skews naive ambiguity-keyed std-dev formulas. [3D tracking docs](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html), [AdvantageKit #141](https://github.com/Mechanical-Advantage/AdvantageKit/issues/141).
- **Limelight MegaTag2** (2024) is ambiguity-free even from a single tag *because you feed it heading* — call `SetRobotOrientation(yaw)` every frame; recommended std devs `(0.7, 0.7, 9999999)` (trust x/y, never vision yaw); reject when |ω| > 360°/s or tag count == 0. [MegaTag2 docs](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2), [announcement](https://www.chiefdelphi.com/t/introducing-megatag2-by-limelight-vision/461243). Community std-dev tuning: [CD thread](https://www.chiefdelphi.com/t/localization-finding-standard-deviations-for-setvisionmeasurementstddevs/496596).

### What we'd actually use

1. `SwerveDrivePoseEstimator` behind the existing `Drive` adapter; `update()` every loop with Pigeon yaw + module positions.
2. **Trust gyro for heading, vision for translation** — set vision heading std dev effectively infinite. Single most important knob.
3. **Feed real capture timestamps** for latency compensation.
4. **Gate before every `addVisionMeasurement`:** reject 0 tags; reject single-tag ambiguity > 0.2 (prefer MultiTag); reject while |ω| > 360°/s; reject off-field / implausible jumps.
5. **Scale x/y std dev with distance + tag count** (tight ~0.5 m multi-tag close, loose several meters single-tag far; reject single tags beyond ~4 m).

Repo tie-in: pose fusion is already hard-gated behind `VisionConstants.kTransformsMeasured` (false) until ROBOT_TO_TURRET / TURRET_TO_CAMERA are measured — correct, a guessed transform poisons odometry. The above is exactly the wiring to enable once those transforms are real. Also consider aligning the existing **0.3 ambiguity gate down to 0.2** (PhotonVision's recommended value).

---

## Simplest viable v1 plan

**Goal:** ship the immutable `WorldModel` snapshot now, wiring only what you already sense, with everything else as explicit sentinels.

1. **Define `record WorldModel`** with all fields from §1. Rebuild it once per loop in a single `RobotState.update()` called early in `robotPeriodic`; publish behind one `AtomicReference`. Decision layer reads `RobotState.current()`.
2. **Wire today (real data):** `timestamp`, `matchTimeRemaining`, `phase`, `alliance` (DriverStation); `fieldToRobot` + `velocity` (start as raw odometry, upgrade to the pose estimator); `mechanism states` (read the `Mechanisms` layer); `vision targets + confidence` (PerceptionProvider). 
3. **Sentinels (leave null/empty, document why):** `hasGamePiece = false`, `opponents = List.of()`, `gamePieces = List.of()`. Decision code must treat these as "unknown," never "none present."
4. **First upgrade — pose fusion (§5):** swap odometry for `SwerveDrivePoseEstimator`; turn on gated `addVisionMeasurement()` once `kTransformsMeasured` is true. Populate `poseConfidence`.
5. **Second upgrade — possession (§4):** implement `hasPiece()` from intake **stator-current spike + velocity dip**, sticky + debounced; flip the `hasGamePiece` sentinel to real. Beam-break later behind the same seam.
6. **Third upgrade — opponents / game pieces (§2–3):** add a **PhotonVision (OPi 5) or Limelight+Coral YOLO** detector; turn each 2D box into a field position via **ground-plane homography / known-size pinhole** (reuse AprilTag intrinsics). Fill the `opponents` / `gamePieces` lists with timestamps + confidence. No neural depth.

Throughout: keep it flat, immutable, one snapshot per tick; bias every choice toward simple, open-weight, permissively licensed, and edge-deployable.
