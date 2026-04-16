# RTNode-Reticulum Transport Node 

A custom firmware for the that operates as a **Transport Node** — in Boundry mode only and bridging a local LoRa radio network with a remote TCP/IP backbone (such as [rmap.world](https://rmap.world)) over WiFi.

The following devices are currently supported with Transport Node Boundary operations.
*other devices pending **Transport Node** incorporation are offered as default barebones microreticulum - see [Releases]() for full listing.*

- **Heltec WiFi LoRa 32 V3**
- **Heltec WiFi LoRa 32 V4**
-  **XIAO ESP32SS (ESP32-S3 + SX1262/LR121)**
-  **LilyGo T3S3-MSVR (ESP32-S3 + SX1280/LR1121)** 
  
*This project was primarily developed with the use of AI assistance. - in all cases code was reviewed audited and implemented with human hands on kb*  

```
  Android / Sideband                                             Remote
  ┌──────────┐          ┌────────────┐                         Reticulum
  │ Sideband │◄── BT ──►│ RNode (BT) │                         Backbone
  │   App    │          └─────┬──────┘                         (rnsd /
  └──────────┘                │                                rmap.world)
                         LoRa Radio                                ▲
                              │            ┌──────────────┐  WiFi  │
                       ◄── RF mesh ──────► │ RTNode       │ ◄─TCP──┘
                              │            │Transport Node│    ▲
                        Other RNodes       └──────────────┘    │
                                                           ┌───┴───┐
                                                           │ Router│
                                                           └───────┘
```

Built on [microReticulum](https://github.com/attermann/microReticulum) (a C++ port of the [Reticulum](https://reticulum.network/) network stack) and the [RNode firmware](https://github.com/markqvist/RNode_Firmware) by Mark Qvist.

## Features

- **Bidirectional LoRa ↔ TCP bridging** — local LoRa mesh nodes can reach the global Reticulum backbone and vice versa
- **Web-based configuration portal** — WiFi SSID/password, backbone host/port, LoRa parameters, all configurable via captive portal
- **OLED status display** — real-time status indicators for LoRa, WiFi, WAN (backbone), LAN (local TCP), plus IP address, port, and airtime
- **Optional local TCP server** — serve local devices on your WiFi in addition to the backbone connection
- **Automatic reconnection** — WiFi and TCP connections recover from drops with exponential backoff
- **ESP32 memory-optimized** — table sizes, timeouts, and caching tuned for the constrained MCU environment
- **Dual board support** — supports both Heltec V3 (8MB flash) and V4 (16MB flash, 2MB PSRAM) with automatic board and PSRAM detection

## Hardware

This firmware was designed for the **Heltec WiFi LoRa 32 V4** and expanded to accomodate the **XIAO ESP32SS with a WIO SX1262 shield** and **LilyGo T3S3-MSVR** (ESP32 + SX1280). This board was chosen for its 2MB PSRAM (4MB T3S3 and 8MB for xiao esp32s3) with LoRa capabilities. While the V3 is supported, it uses the ESP32-S3FN8 which has **no PSRAM**. The firmware **detects PSRAM at runtime** and allocates the TLSF memory pool from SPIRAM when available, falling back to internal SRAM (~170 KB) on boards without PSRAM.


| Component    |  Heltec V3  |  Heltec V4  |    T3S3 v1.2    |  Xiao esp32s3 - *Plus Sense Board* |
|--------------|-------------|-------------|------------|----------------|
|  **MCU**     | ESP32-S3 (ESP32-S3FN8) | ESP32-S3 (ESP32-S3FH4R2) | ESP32-S3 (ESP32-S3FH4R2) | ESP32-S3 (ESP32-S3R8) |
| **Flash**    | 8 MB        | 16 MB       |  4MB (qspi)      |  16MB     |
| **PSRAM**    | None        | 2 MB (QSPI) |  2MB (qspi)      |  8MB     |
| **Radio**    | SX1262 | SX1262 + GC1109 PA |  SX1280/LR1121  | SX1262 / LR1121 |
| **TX Power** | Up to 22 dBm | Up to 28 dBm |  20dBm  |  22dBm   |
| **Display**  | SSD1306 OLED 128×64 | SSD1306 OLED 128×64 | SSD1306 OLED 128×64 | none |
| **WiFi**     | 2.4 GHz 802.11 b/g/n | 2.4 GHz 802.11 b/g/n | 2.4 GHz b/g/n Wi-Fi & Bluetooth 5.0 | 2.4GHz b/g/n Wi-Fi and Bluetooth 5.0/Bluetooth Mesh|
| **USB**      | Native USB CDC | Native USB CDC |  Native USB CDC | Native USB CDC|


## Quick Start

The following devices offer RTNode enhanced memory allocation and active portal OTA configuration features in addition to ports of the base [microReticulum](https://github.com/attermann/microReticulum) firmware.

- lilygo-t3-s3-sx1280-pa-boundry
- lilygo-t3-s3-sx1280-pa-boundry-local
- lilygo-t3-s3-lr1121-pa-boundry
- lilygo-t3-s3-lr1121-pa-boundry-local 
- heltec_V3_boundary
- heltec_V3_boundary-local             
- heltec_V4_boundary               
- heltec_V4_boundary-local            
- seeed_xiao_esp32s3_lr1121_boundary
- seeed_xiao_esp32s3_lr1121_boundary-local
- seeed_xiao_esp32s3_sx1262_boundary
- seeed_xiao_esp32s3_sx1262_boundary-local
  
### Release options

- See releases for available device bins for build an/or build/flash to suit using the enviroments available in platformio.ini
- Release options are qualified as default, boundary, and boundary-local as follows:
  
**Key differences:**

- **Default:** *bare-bones microreticulum transport node port. No boundary mode, no PSRAM usage, no TLSF, uses no_ota.csv memalloc partition table to minimize app partition and maximize space for logging*

- **boundary:** *Full production boundary node. Enables PSRAM usage and two layer segregated fit (TLSF) dynamic memalloc, default*csv partition table permits OTA flash for active portal config but requires two slots for app storage and remaining mem for logging.*

- **boundary-local:** *Same as boundary mode config above without TLSF/allocator.*
*Note: See **[Routing & Memory Customizations](https://github.com/GrayHatGuy/RTNode-HeltecV4/blob/main/README.md#routing--memory-customizations)** for more details on device memory and limitations of table logging.

The table below is provides details on the RTNode devices with boundary enhancements:

| variant type | default | boundary | boundary-local |	
|-------------------|-----------------------|-----------------------|-----------------------|
| Environment | heltec_wifi_lora_32_V4 | heltec_V4_boundary | heltec_V4_boundary-local |
| Partitions | no_ota.csv | default_16MB.csv |	default_16MB.csv |
| Flash/PSRAM mode | default |	qio/qio_qspi | qio/qio_qspi |
| BOARD_HAS_PSRAM |	no | yes | yes |
| BOUNDARY_MODE	| no | yes | yes |
| RNS_USE_TLSF	| no | yes | no |
| RNS_USE_ALLOCATOR	| no | yes | no |

| variant type | default | boundary | boundary-local |	
|-------------------|-----------------------|-----------------------|-----------------------|
| Environment | heltec_wifi_lora_32_V3 | heltec_V3_boundary | heltec_V3_boundary-local |
| Partitions | no_ota.csv | default_8MB.csv |	default_8MB.csv |
| Flash/PSRAM mode | default |	qio/qio_qspi | qio/qio_qspi |
| BOARD_HAS_PSRAM |	no | yes | yes |
| BOUNDARY_MODE	| no | yes | yes |
| RNS_USE_TLSF	| no | yes | no |
| RNS_USE_ALLOCATOR	| no | yes | no |

| variant type | default | boundary | boundary-local |
|-------------------|-----------------------|-----------------------|-----------------------|
| Environment | seeed_xiao_esp32s3 | xiao_esp32s3_boundary | xiao_esp32s3_boundary-local |
| Partitions | no_ota.csv | default_16MB.csv |	default_16MB.csv |
| Flash/PSRAM mode | default |	qio/qio_qspi | qio/qio_qspi |
| BOARD_HAS_PSRAM |	no | yes | yes |
| BOUNDARY_MODE	| no | yes | yes |
| RNS_USE_TLSF	| no | yes | no |
| RNS_USE_ALLOCATOR	| no | yes | no |

| variant type | default | boundary | boundary-local |	
|-------------------|-----------------------|-----------------------|-----------------------|
| Environment | t3s3_sx1280_pa & t3s3_lr1121_pa | t3s3_sx1280_pa_boundary & t3s3_lr1121_pa_boundary | t3s3_sx1280_pa_boundary-local & t3s3_lr1121_pa_boundary-local |
| Partitions | no_ota.csv | partitions_4mb_ota.csv | partitions_4mb_ota.csv |
| Flash/PSRAM mode | default |	qio/qio_qspi | qio/qio_qspi |
| BOARD_HAS_PSRAM |	no | yes | yes |
| BOUNDARY_MODE	| no | yes | yes |
| RNS_USE_TLSF	| no | yes | no |
| RNS_USE_ALLOCATOR	| no | yes | no |

### Option A: Easy Flash (no PlatformIO required)

The easiest way to flash a pre-built firmware. You only need Python 3 and a USB cable.

```bash
# Clone this repo (or download just flash.py + the firmware binary)
git clone https://github.com/jrl290/RTNode-HeltecV4.git
cd RTNode-HeltecV4

# Download latest firmware from GitHub Releases and flash
# (auto-detects V3 vs V4 from flash size)
python flash.py

# Optional: use your machine's installed esptool instead of the bundled copy
python flash.py --use-system-esptool

# Or specify board explicitly
python flash.py --board v3
python flash.py --board v4

# Or flash a local binary
python flash.py --file rtnode_heltec_v4.bin
```

By default, `flash.py` uses the bundled `Release/esptool/esptool.py` for reproducible flashing. Only use `--use-system-esptool` if you explicitly want to override that with a host-installed esptool.

The flash utility auto-detects whether a V3 or V4 is connected by querying the flash size (8MB = V3, 16MB = V4). You can override with `--board v3` or `--board v4`. It will list all available serial ports and prompt you to choose one. If no ports are detected, you may need to hold the **BOOT** button while pressing **RESET** to enter download mode.

### Option B: Build from Source (PlatformIO)
*Note: At present only Boundry mode and environments in [Quick Start](https://github.com/GrayHatGuy/RTNode-HeltecV4/blob/main/README.md#quick-start) are supported.
For development or customization:

```bash
# Prerequisites: PlatformIO installed (VS Code extension or CLI)

git clone https://github.com/jrl290/RTNode-HeltecV4.git
cd RTNode-HeltecV4

# Build for V4
pio run -e heltec_V4_boundary

# Build for V3
pio run -e heltec_V3_boundary

# Build for Seeed Xiao esp32S3 with WIO SX1622 (Boundry Mode Only)
pio run -e seeed_xiao_esp32s3_boundary

# Build for Lilygo T3S3 (Boundry Mode Only)
pio run -e lilygo-t3-s3-sx1280-pa-boundary

# Flash (via PlatformIO)
# update environment to reflect device type
pio run -e heltec_V4_boundary -t upload

# Or create a merged binary and flash with the utility
python flash.py --merge-only    # creates merged firmware bin
python flash.py                 # flash it (auto-detects board)

# Monitor serial output (optional)
pio device monitor -e heltec_V4_boundary
```

### Option C: Manual esptool Flash

If you have the merged binary (`rtnode_heltec_v4.bin`), you can flash it with a single esptool command:

```bash
esptool.py --chip esp32s3 --port /dev/ttyACM0 --baud 921600 \
  write_flash -z --flash_mode qio --flash_freq 80m --flash_size 16MB \
  0x0 rtnode_heltec_v4.bin
```

Replace `/dev/ttyACM0` with your serial port (`/dev/cu.usbmodem*` on macOS, `COM3` on Windows).

On first boot (or if no configuration is found), the device automatically enters the **Configuration Portal**.

## Configuration Portal

### Entering Config Mode

The config portal activates automatically on:
- **First boot** — when no saved configuration exists
- **Button hold >5 seconds** — hold the PRG button for 5+ seconds, the device reboots into config mode

When active, the device creates a WiFi access point named **`RNode-Boundary-Setup`** (open network). A captive portal should appear automatically when you connect; if not, browse to `http://192.168.4.1`.

### Config Page Options

The web form has four sections:

#### 📶 WiFi Network
| Field | Description |
|-------|-------------|
| **WiFi** | Enable/Disable (disable for LoRa-only repeater mode) |
| **SSID** | Your WiFi network name |
| **Password** | WiFi password |

#### 🌐 TCP Backbone
| Field | Description |
|-------|-------------|
| **Mode** | `Disabled` or `Client (connect to backbone)` |
| **Backbone Host** | IP address or hostname of backbone server (e.g. `rmap.world`) |
| **Backbone Port** | TCP port (default: `4242`) |

#### 📡 Local TCP Server (optional)
| Field | Description |
|-------|-------------|
| **Local TCP Server** | Enable/Disable — runs a TCP server on your WiFi for local Reticulum nodes to connect |
| **TCP Port** | Port to listen on (default: `4242`) |

#### 📻 LoRa Radio
| Field | Description |
|-------|-------------|
| **Frequency** | e.g. `867.200` MHz — must match your other RNodes |
| **Bandwidth** | 7.8 kHz – 500 kHz (typically `125 kHz`) |
| **Spreading Factor** | SF6 – SF12 (typically `SF7` for backbone, `SF10` for long range) |
| **Coding Rate** | 4/5 – 4/8 |
| **TX Power** | 2 – 28 dBm |

After saving, the device reboots with the new configuration applied.

## OLED Display Layout

The 128×64 OLED is split into two panels:

### Left Panel — Status Indicators (64×64)

```
 ● LORA          ← filled circle = radio online
 ○ wifi          ← unfilled circle = WiFi disconnected
 ● WAN           ← filled = backbone TCP connected
 ● LAN           ← filled = local TCP client connected
 ────────────────
 Air:0.3%        ← current LoRa airtime
 ▓▓▓▓▓ |||||||   ← battery, signal quality
```

- **Filled circle (●)** = active/connected
- **Unfilled circle (○)** = inactive/disconnected
- Labels are UPPERCASE when active, lowercase when inactive (except LAN which is always uppercase)
- **LAN row is hidden** when the Local TCP Server is disabled in configuration — the remaining layout stays in place

### Right Panel — Device Info (64×64)

```
 ▓▓ RTNode-HV4 ▓▓  ← title bar (inverted)
 867.200MHz       ← LoRa frequency
 SF7 125k         ← spreading factor & bandwidth
 ────────────────  ← separator
 192.168.1.42     ← WiFi IP address (or "No WiFi")
 Port:4242        ← Local TCP server port
 ────────────────  ← separator
```

- **Port** shows the Local TCP server port (the port local nodes connect to), not the backbone port
- **Port line is hidden** when the Local TCP Server is disabled

## Interface Modes

The firmware runs up to **three RNS interfaces** simultaneously, using different interface modes to control announce propagation and routing behavior:

### LoRa Interface — `MODE_ACCESS_POINT`

The LoRa radio operates in **Access Point mode**. In Reticulum, this means:
- The interface broadcasts its own announces but **blocks rebroadcast of remote announces** from crossing to LoRa
- This prevents backbone announces (hundreds of remote destinations) from flooding the limited-bandwidth LoRa channel
- Local nodes discover the transport node directly; the transport node answers path requests for remote destinations from its cache
*Note: Only boundary mode has been verified on Xiao esp32s3 and Lilygo T3S3*

### TCP Backbone Interface — `MODE_BOUNDARY`

The TCP backbone connection uses `MODE_BOUNDARY` (`0x20`), a custom transport mode adapted for the memory-constrained ESP32 environment. In this mode:
- Incoming announces from the backbone are received and cached, but **not stored in the path table by default** — only stored when specifically requested via a path request from a local LoRa node
- This prevents the path table (limited to 48 entries on ESP32) from being overwhelmed by thousands of backbone destinations
- When the path table needs to be culled, **backbone-learned paths are evicted first**, preserving locally-needed LoRa paths

### Optional Local TCP Server — `MODE_ACCESS_POINT`

If enabled, a TCP server on the WiFi network allows local Reticulum nodes to connect. It also uses Access Point mode, with the same announce filtering as LoRa.
*Note: Only boundary mode has been verified on Xiao esp32s3 and Lilygo T3S3*

**Implementation details:**
- Each TCP interface must have a **unique name** to produce a unique interface hash — the backbone uses `"TcpInterface"` and the local server uses `"LocalTcpInterface"`. Without distinct names, both interfaces produce the same hash, causing the interface map lookup to fail when routing packets.
- TCP interfaces are configured with a **10 Mbps bitrate**, which causes Reticulum's Transport to prefer TCP paths over LoRa paths (typically ~1–10 kbps) when both are available for the same destination.
- When the Local TCP Server is disabled, its status indicator (LAN) and port number are hidden from the OLED display.

## Routing & Memory Limits

The ESP32-S3 has limited RAM compared to a desktop Reticulum node. Several customizations were made to the microReticulum library to operate reliably within these constraints:

### Memory allocation partitioning schemes limitations and rationale

**Flash Partition table configuration options with memalloc comparison (4MB Devices):**

| Partition CSV | Application Size | Logging (Flash) Size | OTA | Support Best Use Case | Rationale | Used |
|----------------|----------------|----------------|----------------|----------------|----------------|----------------
| noota_3g.csv | 1.0 MB | ~3.0 MB | No | Max logging storage | Insufficient App space | No |
| no_ota.csv | 2.0 MB | ~2.0 MB | No | Balanced, large app + storage | Max logging without OTA | Yes |
| default.csv | 1.2 MB (x2) | ~1.5 MB | Yes | Standard dev with OTA support | Max logging with OTA for config fails | No |
| huge_app.csv | 3.1 MB | ~0.19 MB | No | Massive apps, almost no logging | Insufficient logging space | No |
| min_spiffs.csv | 1.9 MB (x2) | ~0.19 MB | Yes | Large app + OTA, minimal logs | Insufficient logging space | No |
| partitions_4mb_ota.csv | 1.3 MB (x2) | ~1.4 MB | Yes | fit to bin with OTA | only option for OTA app size | Yes - T3S3 custom |

*This chart provides the user with a reference understanding of the flash space consumed by the app, OTA, and entitlement for maximizing logging including rationale as to why the partition was chosen*
- *Note: due to the limited flash size on the T3S3 4MB with a app binary of 1.2MB with two slots for OTA and permit logging a custom partitioned FAT was created (partitions_4mb_ota.csv). There is very limited space for the app bin size to grow and to log tables.  Possible solutions may be to configure the device without OTA/build flags and or upgrade to a T3S3 T-Beam Supreme with 16MB of flash*
- See **[Memory Usage](https://github.com/GrayHatGuy/RTNode-HeltecV4/blob/main/README.md#memory-usage)** for breakdown of memory allocated per device.

### Table Size Limits

| Table | Default (Desktop) | RTNode-HeltecV4 | Rationale |
|-------|-------------------|-----------|-----------|
| Path table (`_destination_table`) | Unbounded | **48 entries** | Prevents unbounded growth; backbone-learned paths evicted first |
| Hash list (`_hashlist`) | 1,000,000 | **32** | Packet dedup list; small is fine for low-throughput LoRa |
| Path request tags (`_max_pr_tags`) | 32,000 | **32** | Pending path requests rarely exceed a few dozen |
| Known destinations | 100 | **24** | Identity cache; rarely need more on a transport node |
| Max queued announces | 16 | **4** | Outbound announce queue; LoRa is slow, no point queuing many |
| Max receipts | 1,024 | **20** | Packet receipt tracking |

*Note: The Heltec V4 is provided as a benchmark use this table as a scale to your devcice specific memory contraints as listed in [Hardware]()

### Timeout Reductions

| Setting | Default | RTNode-HeltecV4 | Rationale |
|---------|---------|-----------|-----------|
| Destination timeout | 7 days | **1 day** | Free memory faster; stale paths re-resolve automatically |
| Pathfinder expiry | 7 days | **1 day** | Same as above |
| AP path time | 24 hours | **6 hours** | AP paths go stale faster in mesh environments |
| Roaming path time | 6 hours | **1 hour** | Mobile nodes change paths frequently |
| Table cull interval | 5 seconds | **60 seconds** | Less CPU overhead on culling |
| Job/Clean/Persist intervals | 5m/15m/12h | **60s/60s/60s** | More frequent housekeeping for MCU stability |

### Selective Backbone Caching

The most critical optimization: **backbone announces are not stored in the path table by default**. A backbone like `rmap.world` may advertise hundreds of destinations. Storing them all would evict every local LoRa path.

Instead:
1. Backbone announces are received and their packets cached to flash storage
2. When a local LoRa node requests a path, the transport node checks its cache and responds directly
3. Only **specifically requested** paths get a path table entry
4. Path table culling prioritizes evicting backbone entries over local ones

### Default Route Forwarding

When a transport-addressed packet arrives from LoRa but the transport node has no path table entry for it, the firmware:
1. Strips the transport headers (converts `HEADER_2` → `HEADER_1/BROADCAST`)
2. Forwards the raw packet to the backbone interface
3. Creates reverse-table entries so proofs can route back to the sender

This acts as a **default route** — any packet the transport node can't route locally gets forwarded to the backbone.

### Cached Packet Unpacking Fix

The original microReticulum `get_cached_packet()` function called `update_hash()` after deserializing cached packets from flash. However, `update_hash()` only computes the packet hash — it does **not** parse the raw bytes into fields like `destination_hash`, `data`, `flags`, etc.

This was changed to call `unpack()` instead, which parses all packet fields AND computes the hash. Without this fix, path responses contained empty destination hashes and were silently dropped by LoRa nodes.

> **Note:** `unpack()` only parses the plaintext routing envelope (destination hash, flags, hops, transport headers). It does not decrypt the end-to-end encrypted payload. Every Reticulum transport node performs equivalent header parsing during normal routing — this is standard behavior, not a security concern.

### Path Table Update Fix

The C++ `std::map::insert()` method silently does nothing when a key already exists — unlike Python's `dict[key] = value` which replaces. The original microReticulum code used `insert()` to update path table entries, meaning stale LoRa paths were never replaced by newer TCP paths (or vice versa).

This was fixed by calling `erase()` before `insert()`, ensuring updated path entries always replace stale ones. Without this fix, the transport node would continue routing packets via an old interface even after a better path was learned.

### Interface Name Uniqueness

Each RNS interface must have a **unique name** because the name is hashed to produce the interface identifier used in path table lookups. If two interfaces share the same name, they produce the same hash, and `std::map` can only store one — causing the Transport layer to fail to resolve the correct outbound interface for packets.

The TcpInterface constructor accepts an explicit `name` parameter: the backbone uses `"TcpInterface"` and the local server uses `"LocalTcpInterface"`.

## Connecting to the Backbone

### Example: Connect to rmap.world

In the configuration portal:
1. Set WiFi SSID and password
2. Set TCP Backbone Mode to **Client**
3. Set Backbone Host to `rmap.world`
4. Set Backbone Port to `4242`
5. Save and reboot

### Example: Local rnsd Server

On your server, configure `rnsd` with a TCP Server Interface in `~/.reticulum/config`:

```ini
[interfaces]
  [[TCP Server Interface]]
    type = TCPServerInterface
    listen_host = 0.0.0.0
    listen_port = 4242
```

Then configure the transport node as a **Client** pointing to your server's IP.

### Example: rnsd Connects to Transport Node

On your server, configure `rnsd` with a TCP Client Interface:

```ini
[interfaces]
  [[TCP Client to Transport Node]]
    type = TCPClientInterface
    target_host = <transport-node-ip>
    target_port = 4242
```

Set the transport node's **Local TCP Server** to **Enabled** (port 4242).

## Architecture

### Key Files

| File | Purpose |
|------|---------|
| `RNode_Firmware.ino` | Main firmware — transport mode initialization, interface setup, button handling |
| `BoundaryMode.h` | Transport node state struct, EEPROM load/save, configuration defaults |
| `BoundaryConfig.h` | Web-based captive portal for configuration |
| `TcpInterface.h` | TCP interface for both backbone and local server (implements `RNS::InterfaceImpl`) with HDLC framing, unique naming, and 10 Mbps bitrate |
| `Display.h` | OLED display layout — transport node status page |
| `flash.py` | Flash utility — list serial ports, download from GitHub, merge & flash firmware |
| `Boards.h` | Board variant definitions for V3 and V4 |
| `platformio.ini` | Build targets: `heltec_V3_boundary`, `heltec_V4_boundary`, and `heltec_V4_boundary-local` |

### Library Patches

The firmware depends on [microReticulum](https://github.com/attermann/microReticulum) `0.2.4`, automatically fetched by PlatformIO on first build. After the first build, the library sources under `.pio/libdeps/heltec_V4_boundary/microReticulum/src/` need the patches described in "Routing & Memory Customizations" above. Key files modified:

| File | Changes |
|------|---------|
| `Transport.cpp` | Selective caching, default route forwarding, transport-aware culling, `get_cached_packet()` unpack fix, path table `erase()+insert()` fix, memory limits |
| `Transport.h` | `MODE_BOUNDARY`, `PacketEntry`, `Callbacks`, `cull_path_table()`, configurable table sizes |
| `Identity.cpp` | `_known_destinations_maxsize` = 24, `cull_known_destinations()` |
| `Type.h` | `MODE_BOUNDARY` = 0x20, reduced `MAX_QUEUED_ANNOUNCES`, `MAX_RECEIPTS`, shorter timeouts |

### Memory Usage 

**xiao_esp32s3**
```
rnode_firmware_xiao_esp32s3.elf                                       
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       86068 B     362240 B     23.76%
     iram0_2_seg:     1294972 B    8388576 B     15.44%
     dram0_0_seg:      163784 B     345856 B     47.36%
     drom0_0_seg:     1666068 B   33554400 B      4.97%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          16 B         8 KB      0.20%

RAM:   [===       ]  28.7% (used 94084 bytes from 327680 bytes)
Flash: [========  ]  83.2% (used 1743845 bytes from 2097152 bytes)

rnode_firmware_xiao_esp32s3_boundary.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       87532 B     362240 B     24.16%
     iram0_2_seg:     1306164 B    8388576 B     15.57%
     dram0_0_seg:      165648 B     345856 B     47.90%
     drom0_0_seg:     1678392 B   33554400 B      5.00%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          40 B         8 KB      0.49%

RAM:   [===       ]  28.8% (used 94492 bytes from 327680 bytes)
Flash: [===       ]  27.0% (used 1768825 bytes from 6553600 bytes)

rnode_firmware_xiao_esp32s3_boundary_local.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       85988 B     362240 B     23.74%
     iram0_2_seg:     1305524 B    8388576 B     15.56%
     dram0_0_seg:      164040 B     345856 B     47.43%
     drom0_0_seg:     1677632 B   33554400 B      5.00%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          40 B         8 KB      0.49%

RAM:   [===       ]  28.8% (used 94420 bytes from 327680 bytes)
Flash: [===       ]  26.9% (used 1765881 bytes from 6553600 bytes)
```
**t3s3_sx1280_pa**
```
rnode_firmware_t3s3_sx1280_pa.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       87840 B     362240 B     24.25%
     iram0_2_seg:     1321708 B    8388576 B     15.76%
     dram0_0_seg:      168280 B     345856 B     48.66%
     drom0_0_seg:     1750608 B   33554400 B      5.22%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          16 B         8 KB      0.20%

RAM:   [===       ]  29.5% (used 96824 bytes from 327680 bytes)
Flash: [========= ]  85.5% (used 1792409 bytes from 2097152 bytes)

rnode_firmware_t3s3_sx1280_pa_boundary.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       76440 B     362240 B     21.10%
     iram0_2_seg:      907268 B    8388576 B     10.82%
     dram0_0_seg:      129152 B     345856 B     37.34%
     drom0_0_seg:     1179932 B   33554400 B      3.52%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          40 B         8 KB      0.49%

RAM:   [==        ]  21.1% (used 69088 bytes from 327680 bytes)
Flash: [======    ]  59.7% (used 1252941 bytes from 2097152 bytes)

rnode_firmware_t3s3_sx1280_pa_boundary_local.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       74896 B     362240 B     20.68%
     iram0_2_seg:      906520 B    8388576 B     10.81%
     dram0_0_seg:      127528 B     345856 B     36.87%
     drom0_0_seg:     1179196 B   33554400 B      3.51%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          40 B         8 KB      0.49%

RAM:   [==        ]  21.1% (used 69016 bytes from 327680 bytes)
Flash: [======    ]  59.6% (used 1249913 bytes from 2097152 bytes)
```
**t3s3_lr1121_pa**
```
rnode_firmware_t3s3_lr1121_pa.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       87728 B     362240 B     24.22%
     iram0_2_seg:     1321672 B    8388576 B     15.76%
     dram0_0_seg:      168144 B     345856 B     48.62%
     drom0_0_seg:     1750700 B   33554400 B      5.22%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          16 B         8 KB      0.20%

RAM:   [===       ]  29.5% (used 96800 bytes from 327680 bytes)
Flash: [========= ]  85.5% (used 1792353 bytes from 2097152 bytes)


rnode_firmware_t3s3_lr1121_pa_boundary.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       76324 B     362240 B     21.07%
     iram0_2_seg:      907340 B    8388576 B     10.82%
     dram0_0_seg:      129016 B     345856 B     37.30%
     drom0_0_seg:     1180028 B   33554400 B      3.52%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          40 B         8 KB      0.49%

RAM:   [==        ]  21.1% (used 69064 bytes from 327680 bytes)
Flash: [======    ]  59.7% (used 1252993 bytes from 2097152 bytes)

rnode_firmware_t3s3_lr1121_pa_boundary_local.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       74780 B     362240 B     20.64%
     iram0_2_seg:      906556 B    8388576 B     10.81%
     dram0_0_seg:      127392 B     345856 B     36.83%
     drom0_0_seg:     1179292 B   33554400 B      3.51%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          40 B         8 KB      0.49%

RAM:   [==        ]  21.1% (used 68992 bytes from 327680 bytes)
Flash: [======    ]  59.6% (used 1249929 bytes from 2097152 bytes)
```
**heltec32v3**
```
rnode_firmware_heltec32v3.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       84756 B     362240 B     23.40%
     iram0_2_seg:     1330764 B    8388576 B     15.86%
     dram0_0_seg:      164784 B     345856 B     47.65%
     drom0_0_seg:     1753008 B   33554400 B      5.22%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          16 B         8 KB      0.20%

RAM:   [===       ]  29.4% (used 96396 bytes from 327680 bytes)
Flash: [========= ]  85.8% (used 1800273 bytes from 2097152 bytes)

node_firmware_heltec32v3_boundary.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       74404 B     362240 B     20.54%
     iram0_2_seg:      904916 B    8388576 B     10.79%
     dram0_0_seg:      126984 B     345856 B     36.72%
     drom0_0_seg:     1182724 B   33554400 B      3.52%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          40 B         8 KB      0.49%

RAM:   [==        ]  21.0% (used 68952 bytes from 327680 bytes)
Flash: [====      ]  37.4% (used 1251105 bytes from 3342336 bytes)

rnode_firmware_heltec32v3_boundary_local.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       72860 B     362240 B     20.11%
     iram0_2_seg:      904148 B    8388576 B     10.78%
     dram0_0_seg:      125360 B     345856 B     36.25%
     drom0_0_seg:     1181980 B   33554400 B      3.52%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          40 B         8 KB      0.49%

RAM:   [==        ]  21.0% (used 68880 bytes from 327680 bytes)
Flash: [====      ]  37.3% (used 1248049 bytes from 3342336 bytes)
```
**heltec32v4**
```
rnode_firmware_heltec32v4.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       87208 B     362240 B     24.07%
     iram0_2_seg:     1325600 B    8388576 B     15.80%
     dram0_0_seg:      167464 B     345856 B     48.42%
     drom0_0_seg:     1751812 B   33554400 B      5.22%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          16 B         8 KB      0.20%

RAM:   [===       ]  29.5% (used 96628 bytes from 327680 bytes)
Flash: [========= ]  85.7% (used 1796637 bytes from 2097152 bytes)

rnode_firmware_heltec32v4_boundary.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       76868 B     362240 B     21.22%
     iram0_2_seg:      899772 B    8388576 B     10.73%
     dram0_0_seg:      129680 B     345856 B     37.50%
     drom0_0_seg:     1181508 B   33554400 B      3.52%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          40 B         8 KB      0.49%

RAM:   [==        ]  21.1% (used 69184 bytes from 327680 bytes)
Flash: [==        ]  19.0% (used 1247481 bytes from 6553600 bytes)

rnode_firmware_heltec32v4_boundary_local.elf
Memory region         Used Size  Region Size  %age Used
     iram0_0_seg:       75324 B     362240 B     20.79%
     iram0_2_seg:      898948 B    8388576 B     10.72%
     dram0_0_seg:      128056 B     345856 B     37.03%
     drom0_0_seg:     1180772 B   33554400 B      3.52%
    rtc_iram_seg:          33 B       8176 B      0.40%
    rtc_data_seg:          44 B       8176 B      0.54%
    rtc_slow_seg:          40 B         8 KB      0.49%

RAM:   [==        ]  21.1% (used 69112 bytes from 327680 bytes)
Flash: [==        ]  19.0% (used 1244377 bytes from 6553600 bytes)
```
## License

This project is licensed under the **GNU General Public License v3.0** — see [LICENSE](LICENSE) for details.

Based on:
- [RNode Firmware](https://github.com/markqvist/RNode_Firmware) by Mark Qvist (GPL-3.0)
- [microReticulum](https://github.com/attermann/microReticulum) by Chris Attermann (GPL-3.0)
- [Reticulum](https://reticulum.network/) by Mark Qvist (MIT)

