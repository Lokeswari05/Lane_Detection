# Lane Detection  – Complete Guide
## Algorithm: RGB→Gray→ROI→SDCT→Blob→RANSAC→Overlay
### FPGA Vision Remote Lab | Cyclone V 5CEBA2F17C6

---

## Project File Structure

```
lane_detect_v2/
├── rtl/
│   ├── lane_pkg.vhd           ← Shared constants & helper functions
│   ├── rgb2gray.vhd           ← Stage 1: RGB to Grayscale
│   ├── roi_mask.vhd           ← Stage 2: Region of Interest mask
│   ├── line_buffer_7tap.vhd   ← 6-line buffer (vertical filter prep)
│   ├── sdct_edge.vhd          ← Stage 3: SDCT Edge Detection
│   ├── blob_analysis.vhd      ← Stage 4: Binary Blob Analysis
│   ├── ransac_line.vhd        ← Stage 5: RANSAC Line Fitting
│   ├── lane_overlay.vhd       ← Stage 6: Lane Overlay Renderer
│   ├── pixel_delay.vhd        ← Generic delay shift register
│   └── lane_detect.vhd        ← TOP LEVEL entity
├── sim/
│   └── sim_lane_detect.vhd    ← Self-checking testbench
├── constraints/
│   └── lane_detect.sdc        ← 74.25 MHz timing constraints
└── lane_detect_default_Cyclone_V.qsf  ← Pin assignments
```

---

## Algorithm Pipeline

```
VIDEO IN (RGB 8-bit, 1280×720@60Hz, 74.25 MHz)
         │
    [Stage 1] rgb2gray.vhd
         │  Gray = (77·R + 150·G + 29·B) / 256
         │  Latency: 1 clk
         │
    [Stage 2] roi_mask.vhd
         │  Mask rows 0..287 to zero (keep bottom 60% only)
         │  ROI_TOP = 288  (720 × 40% = 288)
         │  Latency: 1 clk
         │
    [Stage 3a] line_buffer_7tap.vhd
         │  6-line circular RAM → 7 vertical taps
         │  Latency: 1 clk (registered output)
         │
    [Stage 3b] sdct_edge.vhd
         │  Horizontal 7-tap SDCT: kernel [-1,-2,-1,0,+1,+2,+1]
         │  Vertical  7-tap SDCT: same kernel on line-buffer taps
         │  Edge mag = |H| + |V|;  binary = (mag > threshold)
         │  Latency: 4 clks (3 H-shift + 1 output reg)
         │
    [Stage 4] blob_analysis.vhd
         │  Run-length encode edge pixels per row
         │  Compute centroid X per blob (min width = 3 px)
         │  Store up to 8 blob centroids per line
         │  Latency: 1 clk (publish at DE falling edge)
         │
    [Stage 5] ransac_line.vhd  (asynchronous to pixel stream)
         │  Accumulate blob history (64-entry circular buffer)
         │  Every 8 lines: run RANSAC on left/right blobs
         │  Fit line: X = A·row/256 + B
         │  Output: lane_l_a, lane_l_b, lane_r_a, lane_r_b
         │
    [Stage 6] lane_overlay.vhd
         │  Delayed original RGB input (8 clk delay)
         │  Compute lane X-intercept per row from RANSAC params
         │  Left  lane  → GREEN  overlay (R=0,   G=255, B=0)
         │  Right lane  → BLUE   overlay (R=0,   G=0,   B=255)
         │  Fill region → +64 R, +64 G  (yellow tint)
         │  Latency: 1 clk
         │
VIDEO OUT (RGB 8-bit, same timing as input + 9 total pipeline clks)
```

---

## STEP 1 – Create Quartus Project

1. Open **Quartus Prime Lite** (v19.1 or later recommended)
2. **File → New Project Wizard**
3. Settings:
   - **Directory**: `C:/your_path/lane_detect_v2`
   - **Project Name**: `lane_detect`
   - **Top-Level Entity Name**: `lane_detect`  ← must match exactly
4. **Next** → Choose **Empty Project** → **Next**
5. **Do NOT add files manually** – the QSF file handles this
6. Device:
   - **Family**: Cyclone V (E/GX/GT/SX/SE/ST)
   - **Name filter**: `5CEBA2F17C6`
   - Select the device → **Finish**

---

## STEP 2 – Import Pin Assignments & Source Files

1. **Assignments → Import Assignments**
2. Browse to `lane_detect_default_Cyclone_V.qsf`
3. Click **OK**

> ✅ This imports ALL source file paths AND pin assignments in one step.

---

## STEP 3 – Add Timing Constraints

1. **Assignments → Settings → Timing Analyzer (Timing)**
2. Under "SDC Files" click **Add**
3. Browse to `constraints/lane_detect.sdc` → **Open** → **OK**

---

## STEP 4 – Simulation in Questa/ModelSim

### 4a. Link Questa to Quartus
1. **Tools → Options → EDA Tool Options**
2. Find **"Questa Intel FPGA"** row
3. Set path: `C:/intelfpga_lite/24.1std/questa_fse/win64`
4. Click **OK**

### 4b. Configure Testbench
1. **Assignments → Settings → EDA Tool Settings → Simulation**
2. Tool name: `Questa Intel FPGA (VHDL)`
3. Click **Test Benches…** → **New**
4. Fill in:
   - Test bench name: `sim_tb`
   - Top-level module in testbench: `sim_lane_detect`
5. Click **Add** → browse to `sim/sim_lane_detect.vhd` → **Add**
6. Click **OK** → **OK** → **OK**

### 4c. Run RTL Simulation
1. **Analysis & Synthesis** must complete first: press Ctrl+K
2. **Tools → Run Simulation Tool → RTL Simulation**
3. Questa opens with all files pre-compiled
4. In Transcript window: `run -all`
5. ✅ **Expected output:**
   ```
   ** Failure: Simulation completed - EVERYTHING OK: Left (GREEN) and Right (BLUE) lanes detected!
   ```
   *(In Questa, assert severity failure = simulation stop = correct!)*

---

## STEP 5 – Full Compilation

1. Click **▶ Compile Design** (or Ctrl+L)
2. All 5 tasks should complete with ✅:
   - Analysis & Synthesis
   - Fitter (Place & Route)
   - Assembler → generates **`output_files/lane_detect.sof`**
   - EDA Netlist Writer
   - Timing Analysis (check: all paths met)
3. Check Timing Analysis: **Tools → Timing Analyzer → Report Timing Summary**
   - All setup slacks should be **positive**

---

## STEP 6 – FPGA Vision Remote Lab Upload

1. Open browser: **https://fpga-vision-lab.h-brs.de/weblab/login**
2. Login (or use demo credentials: `demo` / `welcome`)
3. Select experiment: **"C V all"** (Cyclone V boards)
4. Click **Reservieren** (Reserve) – 5 minute session starts
5. Upload:
   - Click **"Datei auswählen"**
   - Navigate to `output_files/lane_detect.sof`
   - Click **"Upload FPGA binary and start experiment"**
6. Choose a road/lane image using **< >** arrows
7. Click **"Update Output Image and Core Current"**

### ✅ Expected Output Image
- **Grayscale background** (road surface)
- **GREEN line** drawn on detected left lane boundary
- **BLUE line** drawn on detected right lane boundary
- **Yellow tint** between the two lanes (fill region)
- Above ROI (top 40% of image): original colour passthrough

### LED indicators
- **LED[0]** ON → Left lane RANSAC fit valid
- **LED[1]** ON → Right lane RANSAC fit valid

---

## STEP 7 – Mode Switches (enable_in)

| Switch pattern | Mode |
|---|---|
| `000` | Grayscale only (Stage 1 debug) |
| `001` | Edge binary map (Stage 3 debug – white edges) |
| `010` | Edge magnitude (Stage 3 debug – grayscale intensity) |
| `011` | **Lane overlay (DEFAULT – final result)** |
| `1xx` | Lane overlay (same as 011) |

Set switches BEFORE uploading binary, or use the Switch controls in the Remote Lab web interface if available.

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| "No signal" in Remote Lab | .sof didn't compile / wrong device | Check device = 5CEBA2F17C6, recompile |
| All black output | Wrong mode switch setting | Set enable_in = "011" |
| No GREEN/BLUE lines | RANSAC hasn't converged yet | Normal for first 2-3 frames; wait or choose image with clear lanes |
| Lanes in wrong position | Edge threshold too high/low | Change EDGE_THR in lane_pkg.vhd (default=30), recompile |
| Timing analysis fails | Combinatorial path too long | Reduce HIST_DEPTH in ransac_line.vhd from 64 to 32 |
| Compilation error on lane_pkg | Package not first in compile order | Ensure lane_pkg.vhd is first in QSF file list |

---

*FPGA Vision Remote Lab – Hochschule Bonn-Rhein-Sieg*

