# Bathroom Fan Remote (HomeKit) + SHT30 Sensor (ESP32)

This project turns an ESP32 into a **HomeKit-enabled bathroom fan controller** with **automatic humidity control**.

It is made for bathroom fans with **3 fixed speed levels**, controlled via “remote button inputs” (the ESP32 simulates those button presses using short GPIO pulses).

In addition to fan control, the device includes an **SHT30 / SHT3x temperature + humidity sensor**, and publishes those readings to Apple Home (HomeKit).

---

## What you get in Apple Home (HomeKit)

After pairing the accessory, you will see:

### 1) **Fan tile**
- **On / Off button**
- **Speed slider (0–100%)**

### 2) **Bathroom Temperature**
- Live temperature (°C)

### 3) **Bathroom Humidity**
- Live humidity (% RH)

---

## How the fan control works (end-user view)

Your fan has **3 real speeds**, even though HomeKit shows a slider:

| Fan slider in HomeKit | Real fan mode |
|-----------------------|--------------|
| 0–33%                 | Low *(treated as Off)* |
| 34–66%                | Medium |
| 67–100%               | High |

To keep the user experience clear, the slider will “snap” to:

- **0%** (Low / Off)
- **50%** (Medium)
- **100%** (High)

---

## “Off” in the Home app = Low speed
When you press **Off** in HomeKit, the fan is placed into **Low mode**.

This is intentional:
- It provides a safe default state
- After a reboot, the fan always returns to a predictable level

So in everyday usage:

**Off (HomeKit)** → Fan **Low speed**  
**On (HomeKit)** → Fan **Medium speed** (default “On” level)

---

## Automatic mode (default)

This accessory is designed to work in **Automatic mode by default**.

That means the fan can automatically switch between:

- **Low** (HomeKit Off)
- **Medium**
- **High**

based on the bathroom humidity.

### Typical shower behavior
When you start showering:
- humidity rises quickly
- the fan automatically switches to **High**

After the shower:
- the fan slowly returns to **Medium**
- then back to **Low** when the bathroom is dry again

---

## 