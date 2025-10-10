# 📡 MAC Address Setup Guide

This guide will help you configure the MAC addresses for your air quality sensor system communication. 

## 🤔 What is a MAC Address?

For this sensor system to work properly, the **drone ESP32** needs to know the **ground station ESP32's** MAC address so it can send sensor data wirelessly using ESP-NOW technology.

## 🎯 What You Need to Do

You need to:
1. Find the MAC address of your ground station ESP32
2. Update the drone ESP32 code with that MAC address

## 📋 Step-by-Step Instructions

### Step 1: Find Your Ground Station's MAC Address

#### Option A: Using the Mac Address Scanner (Easiest)

1. **Connect your ground station ESP32** to your computer via USB cable
2. **Open Visual Studio Code** 
3. **Navigate to** the `drone_esp32` folder in this project
4. **Upload the MAC address scanner**:
   - Open the file `drone_esp32/src/macAddress.cpp`
   - This is a special program that will show you the MAC address
   - Upload this program to your **ground station ESP32**
   - Make sure to comment out the code in main.cpp, do this
   ```
   ctrl+a (command for mac)
   ctrl+/
   ```
5. **Open the Serial Monitor**:
   - Looks like a power cable in the bottom
   - You should see something like: `[DEFAULT] ESP32 Board MAC Address: 88:13:bf:82:46:3c`
6. **Write down this MAC address** - you'll need it in the next step!

### Step 2: Update the Drone ESP32 Code

Now you need to tell the drone ESP32 where to send the sensor data.

1. **Open the main drone code**:
   - Navigate to `drone_esp32/src/main.cpp`
   - Look for line 44 (around the top of the file)

2. **Find this line**:
   ```cpp
   uint8_t broadcastAddress[] = {0x88, 0x13, 0xbf, 0x82, 0x46, 0x3c}; // Replace with correct receiver MAC
   ```

3. **Replace the MAC address**:
   - If your ground station MAC address is `AA:BB:CC:DD:EE:FF`
   - Change the line to look like this:
   ```cpp
   uint8_t broadcastAddress[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}; // Replace with correct receiver MAC
   ```

#### 🔄 How to Convert MAC Address Format

The MAC address needs to be converted from `AA:BB:CC:DD:EE:FF` format to `{0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}` format.

**Example:**
- **Original:** `88:13:bf:82:46:3c`
- **Converted:** `{0x88, 0x13, 0xbf, 0x82, 0x46, 0x3c}`

**Rules:**
- Remove the colons (`:`)
- Add `0x` before each pair of characters
- Put curly braces `{}` around everything
- Separate each part with commas
- Keep lowercase letters lowercase

### Step 3: Upload and Test

1. **Save the file** after making your changes
2. **Upload the updated code** to your drone ESP32
3. **Upload the ground station code** to your ground station ESP32
4. **Test the connection**:
   - Power on both devices
   - Check the serial monitor for connection messages
   - You should see "ESP-NOW Connected" on the drone's display

## 🔧 Troubleshooting

### ❌ "ESP-NOW send error" Messages
- **Problem:** The MAC addresses don't match
- **Solution:** Double-check that you copied the MAC address correctly

### ❌ No Data Received on Ground Station
- **Problem:** Communication not working
- **Solution:** 
  - Verify both devices are powered on
  - Check that both devices are on the same WiFi channel (channel 6)
  - Make sure devices are within range (ESP-NOW works up to ~200 meters)

### ❌ Can't Find MAC Address
- **Problem:** The MAC address scanner isn't working
- **Solution:**
  - Check that the USB cable is properly connected
  - Try a different USB port
  - Make sure the ESP32 is getting power (LED should be on)

## 📝 Quick Reference

### Current Default MAC Address
The code currently has this default MAC address set:
```
88:13:bf:82:46:3c
```

### File Locations
- **Drone code to modify:** `drone_esp32/src/main.cpp` (line 44)
- **MAC scanner tool:** `drone_esp32/src/macAddress.cpp`
- **Ground station code:** `ground_station/ground_esp32/src/main.cpp`

## 🆘 Need Help?

[Shivam Walia](https://www.linkedin.com/in/shivam-walia1/) — Mechatronics Engineering @ UWaterloo
