# 🔋 Buggy Battery Telemetry Logger (Modbus + SOC + MQTT + Excel)

This Python script runs on a Linux device (ex: Raspberry Pi) and acts as a
**battery telemetry system** for an electric buggy or vehicle.

It:

- Reads power data from a DCM230-2 meter via Modbus RTU
- Estimates battery SOC (State of Charge) intelligently
- Learns internal battery resistance automatically
- Logs data to Excel
- Publishes telemetry to MQTT
- Announces online/offline status

---

## 📦 Main Features

✔ Modbus RTU reading  
✔ Smart SOC estimation (coulomb + voltage fusion)  
✔ Auto battery resistance learning  
✔ Excel logging  
✔ MQTT telemetry publishing  
✔ TLS support  
✔ Environment variable configuration  

---

---

# ⚙️ CONFIGURATION (Environment Variables)

You can override defaults without editing code:

BUGGY_ID=buggy01  
MODBUS_PORT=/dev/ttyUSB0  
MODBUS_BAUD=9600  
MODBUS_SLAVE_ID=1  

EXCEL_FILE=dcm230_log.xlsx  
LOG_INTERVAL=2.5  

MQTT_HOST=broker_ip  
MQTT_PORT=1883  
MQTT_USERNAME=user  
MQTT_PASSWORD=pass  
MQTT_TOPIC=buggy1/data  

BAT_CAPACITY_AH=100  
BAT_CHEMISTRY=leadacid  
R_INTERNAL_OHM=0.04  

---

---

# 🔁 MAIN LOOP FLOW

Every LOG_INTERVAL seconds:

1) Read Modbus registers  
2) Convert current sign (+A = discharge)  
3) Estimate internal resistance  
4) Estimate SOC %  
5) Print console output  
6) Append to Excel file  
7) Publish MQTT telemetry  
8) Sleep until next cycle  

---

---

# 📊 MODBUS REGISTERS USED

DCM230-2 default registers:

| Address | Meaning |
|--------|--------|
| 0      | Voltage |
| 6      | Current |
| 12     | Power |
| 72     | Import Energy |
| 74     | Export Energy |

32-bit floats are read from two registers each.

---

---

# 🧠 SOC ESTIMATION LOGIC (SmartSOC Class)

SOC is calculated using:

## 1️⃣ Coulomb Counting

Tracks amp-hours over time:

SOC -= (Current × Time) / Capacity

---

## 2️⃣ IR Drop Compensation

Battery voltage drops under load:

Voc = V + I × Rinternal

Voc is then mapped to SOC using lookup tables.

---

## 3️⃣ Voltage → SOC Curves

Lead-acid example:

Voltage → SOC

54.0V = 100%  
52.8V = 90%  
48.4V = 50%  
44.0V = 0%  

LiFePO4 has its own curve.

Linear interpolation is used between points.

---

---

## 4️⃣ Rest Detection

If current < threshold for REST_SECS seconds:

→ trust voltage more  
→ slowly correct coulomb drift

---

---

## 5️⃣ Smoothing & Rate Limiting

SOC output is stabilized using:

- Exponential moving average
- Maximum % change per second

This avoids jumping values on dashboards.

---

---

# 🔧 INTERNAL RESISTANCE ESTIMATION

The script detects:

REST → LOAD transitions.

When a load is applied:

R ≈ ΔV / I

Where:

ΔV = Vrest − Vload

Results are smoothed and stored back into SOC model.

This allows automatic battery aging calibration.

---

---

# 📡 MQTT PAYLOAD FORMAT

Telemetry messages:

```json
{
  "type": "power",
  "buggyId": "buggy01",
  "ts": 1700000000000,
  "data": {
    "voltage": 52.3,
    "current": 12.4,
    "percentage": 83.5
  }
}
```

Status messages:

ONLINE:
```json
{
  "type": "status",
  "buggyId": "buggy01",
  "data": { "online": true }
}
```

OFFLINE:
```json
{
  "type": "status",
  "buggyId": "buggy01",
  "data": { "online": false }
}
```

---

---

# 📁 EXCEL OUTPUT

File: dcm230_log.xlsx

Columns:

Timestamp  
Voltage  
Current  
Power  
Import Energy  
Export Energy  
SOC  
R_internal  

Each loop appends a new row.

---

---

# 🏁 SUMMARY

This script is designed for:

Electric buggies  
Battery telemetry  
Fleet monitoring  
IoT dashboards  
Remote logging  

It provides:

✔ Accurate SOC  
✔ Automatic calibration  
✔ MQTT publishing  
✔ Offline logging  
✔ Production-style telemetry  

---

Author: Buggy Telemetry System
