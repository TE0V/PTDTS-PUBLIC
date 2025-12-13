# Audio Array Setup Guide

## Overview

The PTDTS system uses a **ReSpeaker 4-microphone array** with **ODAS (Open embeddeD Audition System)** for acoustic drone detection and localization.

## Hardware Requirements

- **ReSpeaker 4-Mic Array (XMOS XVF3800)**
  - USB connection
  - Appears as "ArrayUAC10" or "ReSpeaker 4 Mic Array (UAC1.0)"
  - 4-channel circular microphone array (46.5mm radius)

## Software Installation

### 1. ODAS Installation

ODAS has been installed on this system at:
- **Binary:** `/usr/local/bin/odaslive`
- **Source:** `/opt/odas/`
- **Config directory:** `/opt/odas/config/`

To manually install or update ODAS:
```bash
cd /opt/odas
git pull
cd build
cmake .. && make -j4 && make install
```

### 2. Dependencies

Required packages (already installed):
- `libasound2-dev` - ALSA audio library
- `libpulse-dev` - PulseAudio library
- `libfftw3-dev` - Fast Fourier Transform library
- `libconfig-dev` - Configuration file parser
- `libjson-c-dev` - JSON library
- `alsa-utils` - ALSA utilities (arecord, aplay, etc.)

## Hardware Connection

1. **Connect ReSpeaker Array**
   - Plug the ReSpeaker 4-mic array into a USB port
   - Wait a few seconds for it to be recognized

2. **Verify Connection**
   ```bash
   arecord -l
   ```
   You should see output like:
   ```
   card X: ArrayUAC10 [ReSpeaker 4 Mic Array (UAC1.0)], device 0: USB Audio
   ```

3. **Test Microphone**
   ```bash
   arecord -D hw:X,0 -f S32_LE -r 16000 -c 4 test.wav
   # Speak into the mic, then Ctrl+C
   aplay test.wav
   ```

## ODAS Configuration

The PTDTS system **auto-generates** ODAS configuration based on `config/config.yaml`:

```yaml
acoustic:
  enabled: true
  mic_radius_mm: 46.5      # ReSpeaker array geometry
  mic_count: 4              # 4 microphones
  odas_port: 9000          # TCP port for detections
  odas_config_path: /opt/odas/config/ptdts_respeaker.cfg
  energy_threshold: 0.6    # Detection sensitivity (0-1)
  frequency_min: 100.0     # Hz - Lower bound for drone signatures
  frequency_max: 500.0     # Hz - Upper bound for drone signatures
```

The config is auto-generated at runtime if not present. You can also manually create a config using the Python code in `src/hardware/acoustic.py`.

## Testing

### Quick Test Script
```bash
python test_audio_array.py
```

This will check:
- ✓ Audio device detection
- ✓ ODAS installation
- ✓ ODAS connection
- ✓ Python integration

### Manual ODAS Test

1. **Generate a test config** (if needed):
   ```bash
   python -c "
   from src.hardware.acoustic import ODASAcousticDetector
   detector = ODASAcousticDetector(
       mic_radius_mm=46.5,
       mic_count=4,
       odas_port=9000
   )
   config_path = detector._generate_odas_config()
   print(f'Config generated: {config_path}')
   "
   ```

2. **Start ODAS manually**:
   ```bash
   odaslive -c /tmp/ptdts_odas_config.json
   ```

3. **In another terminal, listen for detections**:
   ```bash
   nc localhost 9000
   ```
   You should see JSON messages with sound source locations.

### Full System Test

Run the main application:
```bash
python main.py
```

The system will:
1. Auto-detect the ReSpeaker array
2. Generate ODAS configuration
3. Start ODAS server
4. Begin listening for acoustic detections
5. Trigger pan motor when a drone is detected

## Acoustic Detection Flow

```
Sound Source
     ↓
ReSpeaker Array (4 mics)
     ↓
ODAS (Sound Source Localization)
     ↓
TCP Socket (port 9000)
     ↓
ODASAcousticDetector (Python)
     ↓
State Machine (LISTENING → PANNING)
     ↓
Pan Motor (aim at azimuth)
     ↓
Camera Detection (visual confirmation)
```

## Calibration

To align the acoustic array with the pan motor coordinate system:

```bash
./run_calibration.sh acoustic
```

This will:
1. Prompt you to position a sound source at known angles
2. Measure the ODAS-detected azimuth
3. Calculate the offset between systems
4. Update `config.yaml` with `calibration.acoustic_offset`

## Troubleshooting

### No Audio Devices Found
- Check USB connection
- Try `lsusb` to see if the device is recognized
- Check `dmesg | tail` for USB errors

### ODAS Connection Failed
- Verify ODAS is running: `ps aux | grep odas`
- Check port 9000 is not in use: `netstat -tuln | grep 9000`
- Review logs in `/tmp/ptdts_odas.log` (if enabled)

### No Detections
- Check energy threshold (try lowering from 0.6 to 0.3)
- Verify frequency range matches your sound source
- Check ODAS is receiving audio: look for activity in ODAS logs
- Ensure sound source is loud enough and within range (< 50m)

### High False Positive Rate
- Increase energy threshold (try 0.7-0.8)
- Narrow frequency range to drone-specific signatures
- Check for environmental noise (fans, wind, etc.)

## Advanced Configuration

### Custom ODAS Parameters

Edit the auto-generated config in `src/hardware/acoustic.py` or create a custom config file:

```json
{
  "microphones": {
    "nChannels": 4,
    "gain": [1.0, 1.0, 1.0, 1.0],
    "mics": [
      {"x": 0.0465, "y": 0.0, "z": 0.0},
      {"x": 0.0, "y": 0.0465, "z": 0.0},
      {"x": -0.0465, "y": 0.0, "z": 0.0},
      {"x": 0.0, "y": -0.0465, "z": 0.0}
    ]
  },
  "ssl": {
    "enabled": true,
    "nChannels": 4,
    "nPots": 4,
    "nTracks": 2,
    "threshold": 0.6,
    "frequency": {"min": 100, "max": 500}
  }
}
```

### Simulation Mode

For testing without hardware:
```python
from src.hardware.acoustic import SimulatedAcoustic

detector = SimulatedAcoustic(detection_rate=0.1)
detector.start()
```

This generates synthetic detections for development and testing.

## Performance

- **Detection latency:** < 100ms (ODAS processing + network)
- **Angular resolution:** ~5-10° (depends on array geometry and SNR)
- **Range:** 10-50m (depends on sound source intensity and ambient noise)
- **Update rate:** ~10 Hz (ODAS SSL tracking)

## References

- [ODAS GitHub](https://github.com/introlab/odas)
- [ReSpeaker 4-Mic Array Documentation](https://wiki.seeedstudio.com/ReSpeaker_4_Mic_Array_for_Raspberry_Pi/)
- PTDTS Architecture: `docs/ARCHITECTURE.md`
- Calibration Guide: `docs/CALIBRATION.md`
