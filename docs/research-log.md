# Research log

Every external source consulted while porting this project to ROS 2 Jazzy and building
the Isaac Sim twin — why it was needed, and what it actually changed. Kept because
several of these overturned an assumption, and the reasoning is worth being able to
re-check later.

Grouped by the question that prompted the search.

---

## Odometry calibration methodology

| Source | Why | Conclusion / effect |
|---|---|---|
| Borenstein & Feng, **UMBmark: A Benchmark Test for Measuring Odometry Errors in Mobile Robots**, SPIE Mobile Robots, Philadelphia, Oct 1995 — [author page](https://websites.umich.edu/~johannb/umbmark.htm), [PDF](https://johnloomis.org/ece445/topics/odometry/borenstein/paper60.pdf) | "How do robotics researchers actually measure odometry error?" | **The standard**, and explicitly for differential-drive robots — which we had just *measured* ours to be. Gave the bidirectional 4×4 m square procedure (5 runs CW + 5 CCW, stop at each corner, turn on the spot, drive slowly), the cluster centre-of-gravity formulas (Eq. 2–3), and the single accuracy figure `E_max,syst = max(r_cg,cw, r_cg,ccw)` (Eq. 4). Adopted wholesale, scaled to a 2 m square. |
| Borenstein & Feng, **Measurement and Correction of Systematic Odometry Errors Caused by Kinematics Imperfections in Mobile Robots** — [PDF](https://johnloomis.org/ece445/topics/odometry/borenstein/paper58.pdf) | UMBmark measures error but the correction factors are in the companion paper | Supplied the derivation: `α` from the **sum** of CW/CCW x-offsets (4.24a), `β` from the **difference** (4.20a), then `R = (L/2)/sin(β/2)` (4.21), `Ed = (R+b/2)/(R−b/2)` (4.22), `Eb = 90/(90−α)` (4.27), and `c_L`/`c_R` (4.31–4.32). The y-forms (4.20b, 4.24b) give α and β independently — adopted as a contamination check. |
| Why the *bidirectional* square (same papers, §3) | Was tempted by a simpler "drive 1 m, measure" test | A unidirectional test can report near-zero error while both dominant errors are large, because **they cancel in one direction and add in the other**. This is the entire justification for 10 runs instead of 2. |
| Skid-steer / ICR odometry literature — e.g. [Experimental kinematics for wheeled skid-steer mobile robots](https://www.researchgate.net/publication/224296401_Experimental_kinematics_for_wheeled_skid-steer_mobile_robots), [Tightly-Coupled LiDAR-IMU-Wheel Odometry with Online Calibration for Skid-Steering Robots](https://arxiv.org/pdf/2404.02515) | Our car is 4-wheel skid-steer, but UMBmark assumes 2-wheel differential | UMBmark still applies as a first-order model, but **expect `Eb` to be large**: turning a skid-steer is slip, so the effective track is much wider than the physical 0.135 m. Also confirmed results are **surface-dependent** — calibrate on the surface you drive on. The rigorous treatment is an ICR-based model; out of scope for now, noted as the upgrade path. |

## micro-ROS: can a frozen Humble firmware talk to a Jazzy agent?

The dominant risk of the whole port, since the factory firmware is binary-only.

| Source | Why | Conclusion / effect |
|---|---|---|
| [eProsima Micro XRCE-DDS docs](https://micro-xrce-dds.docs.eprosima.com/en/latest/) | v2.x and v3.x both exist; is a v2.x client compatible with a v3.x agent? | **No compatibility statement published either way.** So this could not be settled by reading — it drove the decision to test empirically before spending effort on the port. It worked. |
| [micro-ROS-Agent GitHub (jazzy branch)](https://github.com/micro-ROS/micro-ROS-Agent/tree/jazzy) | Is there a Jazzy agent at all? | Yes. |
| [Docker Hub `microros/micro-ros-agent` tags](https://hub.docker.com/r/microros/micro-ros-agent) | Prebuilt image? | `jazzy` tag exists, amd64, 159 MB. Used directly; no source build needed. |
| [`micro_ros_espidf_component`, jazzy branch README](https://github.com/micro-ROS/micro_ros_espidf_component/blob/jazzy/README.md) | The course PDFs demand ESP-IDF **v5.1.2** | **Disproved the PDFs.** The jazzy branch supports IDF v5.2–v5.5 *and* v6.0. The v5.1.2 figure belongs to the obsolete humble branch. Confirmed by building Yahboom's own sample unmodified on v5.4.1. |

## ESP-IDF version management

| Source | Why | Conclusion / effect |
|---|---|---|
| [ESP-IDF Installation Manager (EIM) v0.8 announcement](https://developer.espressif.com/blog/2026/03/esp-idf-installation-manager/), [EIM docs](https://docs.espressif.com/projects/idf-im-ui/en/latest/) | Three IDF trees on this machine with disagreeing metadata | EIM is Espressif's official multi-version manager as of 2026, and was **already installed** at `~/.espressif/eim_gui/eim`. Made it the single source of truth; retired the legacy `idf-env.json`. |
| [ESP-IDF versions guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/versions.html) | Side-by-side best practice | One tree per *release tag* in a version-named directory; never leave one on `master` (which is exactly what had happened); activate per-terminal via `export.sh`, never from `.bashrc`. |

## Isaac Sim

| Source | Why | Conclusion / effect |
|---|---|---|
| [Isaac Sim ROS 2 installation docs](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html) | Would this repeat the Humble/Jazzy fight? | **No** — Jazzy is the *recommended* distro on Ubuntu 24.04. The one compatibility question that turned out easy. |
| [IsaacLab discussion: RTX 50-series support](https://github.com/isaac-sim/IsaacLab/discussions/1888), [TiledCamera bug on Blackwell](https://github.com/isaac-sim/IsaacLab/issues/4951) | RTX 5070 Ti is Blackwell (`sm_120`), quite new | Blackwell needs Isaac Sim **≥ 5.1.0** → chose 6.0.1. Known rough edges around Blackwell rendering; none hit so far. |
| [Workstation install docs](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_workstation.html) | Install method and URL | Binary workstation build (self-contained Python, sidesteps noble's PEP 668). Real URL is `downloads.isaacsim.nvidia.com`; the `download.isaacsim.omniverse.nvidia.com` host in older docs **404s**. 13.0 GB, MD5 verified. |
| [Isaac Sim GitHub releases](https://github.com/isaac-sim/IsaacSim/releases) | Latest version | v6.0.1 (2026-06-22). Source only — binaries are on NVIDIA's CDN. |
| [URDF Importer docs](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/import_urdf.html), [issue #300](https://github.com/isaac-sim/IsaacSim/issues/300) | Convert the car's URDF headlessly | Docs are for older versions; **the API changed in 6.0.1** — `_urdf` and the `URDFParseAndImportFile` command are gone, replaced by `URDFImporter`/`URDFImporterConfig` with `usd_path` as a *directory*. Found by reading the installed extension source, not the docs. |

## Vendor material

| Source | Why | Conclusion / effect |
|---|---|---|
| [YahboomTechnology/Mirco-Ros-Car_VM](https://github.com/YahboomTechnology/Mirco-Ros-Car_VM), [MicroROS-Board](https://github.com/YahboomTechnology/MicroROS-Board), [MicroROS-Car-Pi5](https://github.com/YahboomTechnology/MicroROS-Car-Pi5) | Is the source mirrored anywhere on GitHub? | **No.** All Yahboom repos are documentation-only; the Google Drive bundle is the sole source of code. Settled early and shaped the whole asset-handling approach. |
| Nav2 Jazzy reference `nav2_params.yaml` (shipped in `nav2_bringup`) | Authoritative plugin names and new nodes | The reference *installed on this machine* beat any web source. Gave the `::` plugin naming, showed `plugin_lib_names` commented out and reserved for custom plugins, and revealed `collision_monitor`/`docking_server` as new required blocks. Fixed four separate breakages. |

---

## Sources that changed a decision

Worth calling out, because these are the ones that earned their time:

1. **UMBmark's bidirectional argument** — would otherwise have run a naive straight-line test that can return ~zero error while both dominant errors are large.
2. **micro_ros_espidf_component jazzy README** — killed a pointless ESP-IDF v5.1.2 install.
3. **eProsima's *silence*** on v2.x/v3.x compatibility — the absence of an answer was itself the finding, and pushed the whole plan toward testing the risk early instead of building on top of it.
4. **Isaac Sim's installed extension source** — the published docs described an API that no longer exists in 6.0.1.
5. **Nav2's own reference params** — the local file was more authoritative than anything online.
