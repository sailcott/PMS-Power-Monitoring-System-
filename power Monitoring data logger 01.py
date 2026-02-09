#!/usr/bin/env python3
from pymodbus.client import ModbusSerialClient as ModbusClient
import struct
from datetime import datetime
import time
import openpyxl
from openpyxl import Workbook
import os
import json
import ssl
import uuid
import paho.mqtt.client as mqtt
from bisect import bisect_left

# ===============================
# USER / ENV CONFIG
# ===============================
BUGGY_ID = os.getenv("BUGGY_ID", "buggy01")

# Modbus / DCM230-2
PORT = os.getenv("MODBUS_PORT", "/dev/ttyUSB0")
BAUDRATE = int(os.getenv("MODBUS_BAUD", "9600"))
SLAVE_ID = int(os.getenv("MODBUS_SLAVE_ID", "1"))

# Logging
EXCEL_FILE = os.getenv("EXCEL_FILE", "dcm230_log.xlsx")
LOG_INTERVAL = float(os.getenv("LOG_INTERVAL", "2.5"))  # seconds

# MQTT from environment variables
MQTT_HOST = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))  # 8883 for TLS by default
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")
MQTT_TOPIC = os.getenv("MQTT_TOPIC", "buggy1/data")
CLIENT_BASE = os.getenv("MQTT_CLIENT_ID", "monitoring_01")
UNIQUE_ID_SUFFIX = os.getenv("MQTT_UNIQUE_SUFFIX", "yes").lower() in ("1", "true", "yes")
MQTT_TLS = os.getenv("MQTT_TLS", "auto").lower()  # "auto" | "true" | "false"
MQTT_QOS = int(os.getenv("MQTT_QOS", "0"))
MQTT_RETAIN = os.getenv("MQTT_RETAIN", "false").lower() in ("1", "true", "yes")

# Battery / SOC
BAT_CAPACITY_AH = float(os.getenv("BAT_CAPACITY_AH", "100.0"))  # usable capacity
R_INTERNAL_OHM = float(os.getenv("R_INTERNAL_OHM", "0.04"))     # initial guess; refined automatically
CHEMISTRY = os.getenv("BAT_CHEMISTRY", "leadacid")               # "leadacid" or "lifepo4"
I_REST_THR = float(os.getenv("SOC_I_REST_THR", "2.0"))          # A
REST_SECS = float(os.getenv("SOC_REST_SECS", "60.0"))           # s
EMA_ALPHA = float(os.getenv("SOC_EMA_ALPHA", "0.12"))           # 0..1
MAX_RATE_PCT_PER_S = float(os.getenv("SOC_MAX_RATE", "0.3"))    # %/s
CURRENT_SIGN = int(os.getenv("CURRENT_SIGN", "1"))              # +1: +A=discharge; -1 if opposite

# Auto R estimator
R_EST_I_REST_THR = float(os.getenv("R_EST_I_REST_THR", "2.0"))       # A
R_EST_I_LOAD_THR = float(os.getenv("R_EST_I_LOAD_THR", "10.0"))      # A
R_EST_MIN_DV = float(os.getenv("R_EST_MIN_DV", "0.2"))               # V
R_EST_REST_HOLD = float(os.getenv("R_EST_REST_HOLD", "5.0"))         # s resting before Vrest capture
R_EST_LOAD_STABILIZE = float(os.getenv("R_EST_LOAD_STABILIZE", "2.0"))  # s after load begins
R_EST_EMA_ALPHA = float(os.getenv("R_EST_EMA_ALPHA", "0.25"))        # 0..1

# ===============================
# SOC Estimator
# ===============================
class SmartSOC:
    """
    Coulomb counting + voltage fusion:
      - IR-drop comp: V_oc = V + I * R_internal
      - Voltage→SOC via lookup curve (resting)
      - Rest detection to bias toward voltage at rest
      - EMA smoothing + rate limit for stable display

    Convention: update() expects current_a > 0 = DISCHARGE.
    """
    def __init__(self, capacity_ah=100.0, r_internal=0.04, i_rest_thr=2.0,
                 rest_secs=60.0, ema_alpha=0.12, max_rate_pct_per_s=0.3,
                 chemistry="leadacid"):
        self.capacity_ah = capacity_ah
        self.r_internal = r_internal
        self.i_rest_thr = i_rest_thr
        self.rest_secs = rest_secs
        self.ema_alpha = ema_alpha
        self.max_rate = max_rate_pct_per_s

        if chemistry == "leadacid":
            self.volt_points = [54.0, 52.8, 51.6, 50.4, 49.2, 48.4, 47.6, 46.8, 46.0, 45.2, 44.0]
            self.soc_points  = [100 ,   90 ,   80 ,   70 ,   60 ,   50 ,   40 ,   30 ,   20 ,   10 ,    0]
        elif chemistry == "lifepo4":
            self.volt_points = [55.0, 53.0, 52.0, 51.5, 51.2, 50.8, 50.4, 49.6, 48.8, 48.0, 46.0]
            self.soc_points  = [100 ,   95 ,   90 ,   80 ,   70 ,   60 ,   50 ,   30 ,   20 ,   10 ,    0]
        else:
            raise ValueError("Unknown chemistry")

        self.soc_cc = None
        self.soc_out = None
        self._last_t = None
        self._rest_start = None

    def _interp_soc_from_v(self, voc):
        if voc >= self.volt_points[0]: return 100.0
        if voc <= self.volt_points[-1]: return 0.0
        asc_v = list(reversed(self.volt_points))
        asc_soc = list(reversed(self.soc_points))
        i = bisect_left(asc_v, voc)
        v0, v1 = asc_v[i-1], asc_v[i]
        s0, s1 = asc_soc[i-1], asc_soc[i]
        return s0 + (s1 - s0) * (voc - v0) / (v1 - v0)

    def _rate_limit(self, target, dt):
        if self.soc_out is None or dt <= 0:
            return target
        max_delta = self.max_rate * dt
        delta = target - self.soc_out
        if   delta >  max_delta: return self.soc_out + max_delta
        elif delta < -max_delta: return self.soc_out - max_delta
        return target

    def update(self, voltage_v, current_a, now=None):
        t = time.time() if now is None else now

        if self._last_t is None:
            self._last_t = t
            v_soc = self._interp_soc_from_v(voltage_v)
            self.soc_cc = v_soc
            self.soc_out = v_soc
            self._rest_start = t if abs(current_a) <= self.i_rest_thr else None
            return round(self.soc_out, 1)

        dt = t - self._last_t
        self._last_t = t

        # Coulomb counting
        self.soc_cc -= (current_a * dt) / 3600.0 / self.capacity_ah * 100.0
        self.soc_cc = max(0.0, min(100.0, self.soc_cc))

        # IR compensation
        voc = voltage_v + current_a * self.r_internal
        v_soc = self._interp_soc_from_v(voc)

        # Rest detection + fusion
        at_rest = abs(current_a) <= self.i_rest_thr
        if at_rest:
            self._rest_start = self._rest_start or t
            rest_time = t - self._rest_start
        else:
            self._rest_start = None
            rest_time = 0.0

        if at_rest and rest_time >= self.rest_secs:
            k = 0.6
        elif at_rest:
            k = 0.85
        else:
            k = 0.95

        fused = k * self.soc_cc + (1 - k) * v_soc

        # Rate limit + EMA smoothing
        target = self._rate_limit(fused, dt)
        self.soc_out = target if self.soc_out is None else \
            (self.ema_alpha * target + (1 - self.ema_alpha) * self.soc_out)

        self.soc_out = max(0.0, min(100.0, self.soc_out))
        return round(self.soc_out, 1)

# ===============================
# Auto Internal Resistance Estimator
# ===============================
class InternalResistanceEstimator:
    """Detect REST -> LOAD transitions and estimate R ≈ ΔV / I."""
    def __init__(self, i_rest_thr=2.0, i_load_thr=10.0, min_dv=0.2,
                 rest_hold=5.0, load_stabilize=2.0, ema_alpha=0.25,
                 initial_r=0.04):
        self.i_rest_thr = i_rest_thr
        self.i_load_thr = i_load_thr
        self.min_dv = min_dv
        self.rest_hold = rest_hold
        self.load_stabilize = load_stabilize
        self.ema_alpha = ema_alpha

        self.r_ema = initial_r
        self.state = "UNKNOWN"
        self.t_rest_start = None
        self.v_rest_sample = None
        self.t_load_start = None

    def update(self, voltage_v, current_a, now=None):
        t = time.time() if now is None else now
        abs_i = abs(current_a)

        # RESTING
        if abs_i <= self.i_rest_thr:
            if self.state != "RESTING":
                self.state = "RESTING"
                self.t_rest_start = t
                self.v_rest_sample = None
                self.t_load_start = None
            elif (self.v_rest_sample is None) and (t - self.t_rest_start >= self.rest_hold):
                self.v_rest_sample = voltage_v  # capture rest voltage

        else:
            # LOADING
            if self.state != "LOADING":
                self.state = "LOADING"
                self.t_load_start = t
            if (self.v_rest_sample is not None) and (t - self.t_load_start >= self.load_stabilize) and (abs_i >= self.i_load_thr):
                dv = self.v_rest_sample - voltage_v  # positive if sag
                if dv >= self.min_dv and abs_i > 1e-6:
                    r_sample = dv / abs_i
                    self.r_ema = self.ema_alpha * r_sample + (1 - self.ema_alpha) * self.r_ema
                    self.v_rest_sample = None
                    return round(self.r_ema, 5)

        return None

# ===============================
# Helpers
# ===============================
def read_float(client, address):
    """
    Read 32-bit IEEE754 float (big-endian word order) from two input registers.
    Adjust endianness if your DCM230-2 differs.
    """
    rr = client.read_input_registers(address=address, count=2, unit=SLAVE_ID)
    if rr.isError():
        print(f"[ERROR] Failed to read address {address}")
        return None
    decoder = struct.pack('>HH', rr.registers[0], rr.registers[1])
    return round(struct.unpack('>f', decoder)[0], 2)

# ===============================
# Excel init
# ===============================
if not os.path.exists(EXCEL_FILE):
    wb = Workbook()
    ws = wb.active
    ws.append([
        "Timestamp", "Voltage (V)", "Current (A)", "Power (W)",
        "Import Energy (kWh)", "Export Energy (kWh)", "SOC (%)", "R_internal (Ohm)"
    ])
    wb.save(EXCEL_FILE)

# ===============================
# Modbus & MQTT setup
# ===============================
client = ModbusClient(
    method='rtu',
    port=PORT,
    baudrate=BAUDRATE,
    parity='N',
    stopbits=1,
    bytesize=8,
    timeout=1
)

# Build unique client ID (avoid "Session taken over")
client_id = CLIENT_BASE if not UNIQUE_ID_SUFFIX else f"{CLIENT_BASE}_{uuid.uuid4().hex[:8]}"
mqtt_client = mqtt.Client(client_id=client_id)

# Auth
if MQTT_USERNAME:
    mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

# TLS (auto/true/false)
use_tls = (MQTT_TLS == "true") or (MQTT_TLS == "auto" and MQTT_PORT == 8883)
if use_tls:
    context = ssl.create_default_context()
    # If you use self-signed certs and want to skip CA validation, uncomment:
    # context.check_hostname = False
    # context.verify_mode = ssl.CERT_NONE
    mqtt_client.tls_set_context(context)

# LWT
mqtt_client.will_set(MQTT_TOPIC, payload=json.dumps({
    "type": "status",
    "buggyId": BUGGY_ID,
    "ts": int(time.time() * 1000),
    "data": {"online": False}
}), qos=MQTT_QOS, retain=True)

# Connect
mqtt_client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)

if not client.connect():
    print(f"[FATAL] Could not connect on {PORT}")
    raise SystemExit(1)

# Init estimators
soc_estimator = SmartSOC(
    capacity_ah=BAT_CAPACITY_AH,
    r_internal=R_INTERNAL_OHM,
    i_rest_thr=I_REST_THR,
    rest_secs=REST_SECS,
    ema_alpha=EMA_ALPHA,
    max_rate_pct_per_s=MAX_RATE_PCT_PER_S,
    chemistry=CHEMISTRY
)
r_estimator = InternalResistanceEstimator(
    i_rest_thr=R_EST_I_REST_THR,
    i_load_thr=R_EST_I_LOAD_THR,
    min_dv=R_EST_MIN_DV,
    rest_hold=R_EST_REST_HOLD,
    load_stabilize=R_EST_LOAD_STABILIZE,
    ema_alpha=R_EST_EMA_ALPHA,
    initial_r=R_INTERNAL_OHM
)

print("✅ Data logger started. Press Ctrl+C to stop.")
try:
    # announce online
    mqtt_client.publish(MQTT_TOPIC, json.dumps({
        "type": "status",
        "buggyId": BUGGY_ID,
        "ts": int(time.time() * 1000),
        "data": {"online": True}
    }), qos=MQTT_QOS, retain=True)

    while True:
        loop_start = time.time()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # ---- Read sensors (addresses: adjust if needed) ----
        voltage = read_float(client, 0)
        current_raw = read_float(client, 6)
        power = read_float(client, 12)
        energy_import = read_float(client, 72)
        energy_export = read_float(client, 74)

        if (voltage is None) or (current_raw is None):
            print(f"[{timestamp}] Read error: voltage/current None")
            time.sleep(LOG_INTERVAL)
            continue

        # Apply sign convention (+A discharge)
        current = CURRENT_SIGN * current_raw

        # ---- Auto R estimation ----
        new_r = r_estimator.update(voltage_v=voltage, current_a=current)
        if new_r is not None:
            soc_estimator.r_internal = new_r
            print(f"[{timestamp}] 🔧 Updated R_internal → {new_r:.5f} Ω")

        # ---- SOC update ----
        soc_percent = soc_estimator.update(voltage_v=voltage, current_a=current)

        # Console view
        print(f"[{timestamp}] V={voltage:.2f}V, I={current_raw:.2f}A (conv:{current:.2f}A), "
              f"P={power}W, IMP={energy_import}kWh, EXP={energy_export}kWh, "
              f"SOC={soc_percent:.1f}%, Rint≈{soc_estimator.r_internal:.5f}Ω")

        # ---- Excel log (detailed) ----
        wb = openpyxl.load_workbook(EXCEL_FILE)
        ws = wb.active
        ws.append([timestamp, voltage, current_raw, power, energy_import, energy_export,
                   soc_percent, round(soc_estimator.r_internal, 5)])
        wb.save(EXCEL_FILE)

        # ---- MQTT minimal telemetry (your shape) ----
        ts_unix_ms = int(time.time() * 1000)
        telemetry = {
            "type": "power",
            "buggyId": BUGGY_ID,
            "ts": ts_unix_ms,
            "data": {
                "voltage": voltage,          # V
                "current": current,          # A (sign fixed; +A=discharge)
                "percentage": soc_percent    # %
            }
        }
        mqtt_client.publish(MQTT_TOPIC, json.dumps(telemetry), qos=MQTT_QOS, retain=MQTT_RETAIN)

        # Pace the loop
        elapsed = time.time() - loop_start
        time.sleep(max(0.0, LOG_INTERVAL - elapsed))

except KeyboardInterrupt:
    print("\n🛑 Logging stopped by user.")
finally:
    # announce offline
    try:
        mqtt_client.publish(MQTT_TOPIC, json.dumps({
            "type": "status",
            "buggyId": BUGGY_ID,
            "ts": int(time.time() * 1000),
            "data": {"online": False}
        }), qos=MQTT_QOS, retain=True)
    except Exception:
        pass
    client.close()
    mqtt_client.disconnect()
