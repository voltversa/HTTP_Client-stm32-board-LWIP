# HTTP_Client-stm32-board-LWIP
# Title of the project

STM32F746G-DISCO Network Weather station (FPGA → PC → PHP/Apache → STM32)

# by

Mahmoud Mostafa (R0997618)

# Brief description

This project builds an end‑to‑end weather telemetry pipeline:

1. **FPGA + BME280** — An FPGA reads **temperature, humidity, and pressure** from a Bosch BME280 sensor via I²C and streams the values over **UART** to a PC.

2. **PC layer** — A small **Python** script receives the UART stream and saves the latest readings into files (temp.txt,press.txt,hum.txt).

3. **Web layer** — A **PHP** script on **Apache** (XAMPP) reads that file and exposes the values through a HTTP endpoint.

4. **STM32 client** — An **STM32F746G‑Discovery** board (with LwIP) acts as an HTTP/TCP client: it sends a `GET` request to the PHP endpoint, parses the response (JSON‑like string), and shows the **Temperature / Humidity / Pressure** on the 480×272 LCD. A **touch button** lets you cycle between the three screens. A **screensaver** blanks the LCD after inactivity and wakes on touch.

The result is a complete embedded‑to‑web‑to‑embedded data path demonstrating digital sensors, UART, simple web serving, TCP/IP with LwIP, and an LCD UI with touch.

# Used Libs, compiler and apps

* **LwIP version:2.1.2
* **CubeMX version:6.6.1
* **STM32CubeF7 HAL/BSP:
* **Board BSPs used:** `stm32746g_discovery.h`, `…_lcd.h`, `…_sdram.h`, `…_ts.h`, `…_qspi.h`
* **Toolchain/IDE:** *System Workbench for STM32 (SW4STM32) vX.Y* **or** *STM32CubeIDE vX.Y* (both are compatible)
* **Compiler:** ARM GCC (from IDE toolchain)
* **PC side:** Python 3.11.9 (pyserial), XAMPP (Apache + PHP 7/8)
* **Sensor:** Bosch BME280 (I2C)

# System architecture

```
BME280  →  FPGA (I²C → UART)  →  PC (Python)  →  Apache/PHP endpoint  →  STM32F746G (LwIP TCP client + LCD UI)
```

* **Transport:** UART (FPGA→PC), local HTTP over Ethernet (STM32→PHP)
* **Endpoint (example):** `GET http://192.168.69.11/xampp/readtemp.php?q=brussels,b&appid=1234`
* **Response (example JSON):** `{"temp":"23.4","hum":"55.0","press":"1013.2"}`

# Hardware

* STM32F746G‑Discovery board (LCD 480×272, touch, SDRAM, QSPI Flash, user button, LEDs)
* LAN connection (DHCP or static, configured in LwIP/CubeMX)
* FPGA board with BME280

# How to build (STM32)

1. **Open the project** in SW4STM32 or STM32CubeIDE.
2. Ensure CubeMX‑generated files are present (HAL, BSP, LwIP middleware).
3. Configure **LwIP** (static IP) and **ethernet interface** (PHY) in CubeMX if regenerating.
4. Build the project (Debug or Release). Flash to the STM32F746G‑DISCO.

# How to run (end‑to‑end)

**PC side**

PC side

Connect the FPGA UART to the PC (e.g., COM6 @ 9600 8N1) and run your Python receiver. It parses the incoming UART lines and atomically writes the latest readings into separate files inside your webroot:

C:/xampp/htdocs/xampp/temp.txt

C:/xampp/htdocs/xampp/press.txt

C:/xampp/htdocs/xampp/hum.txt



Python script uses temporary files (*_tmp.txt) and os.replace() to avoid partial reads while PHP serves the files.

Install XAMPP (Apache + PHP) and place readtemp.php in C:/xampp/htdocs/xampp/. The PHP reads the three files and responds with JSON:
<?php
if (isset($_GET["q"]) && isset($_GET["appid"])) {
    $fp = fopen("log.txt", "a");
    fprintf($fp, "%s: Asked for the weather in %s with appkey: %s

**STM32 side**

1. Connect the board to the same LAN as the PC/Apache machine.
2. In `main.c`, set **server IP** and endpoint path:

   * `IP4_ADDR(&server_ip, 192,168,69,11);`
   * `"GET /xampp/readtemp.php?q=brussels,b&appid="` (adjust path/query as needed)
3. Reset the board. Once **netif** is up, the app periodically performs a GET request, parses `temp/hum/press`, and updates the LCD. Touch the **NEXT** icon to cycle screens.


* **Networking:**

  * `MX_LWIP_Init()` initializes LwIP. The code checks `netif_is_up(&gnetif)` before connecting.
  * `tcp_client_connect()` creates a `tcp_pcb`, connects to `server_ip:80`, and registers callbacks.
* **HTTP GET (raw TCP):**

  * On `connected_callback()`, the client writes a minimal HTTP/1.0 GET header in three `tcp_write()` calls, then sets `tcp_recv()`.
* **Parsing:**

  * `recv_callback()` copies the payload to `rx_buffer` and searches for `"temp"`, `"hum"`, `"press"`. The extracted strings are copied into `latest_temp`, `latest_hum`, `latest_press` and flagged as ready.
* **LCD & Touch UI:**

  * Foreground and background layers are initialized; icons are drawn; `draw_screen()` shows the selected metric.
  * Touch toggles **Temperature → Humidity → Pressure** via a **NEXT** icon.
* **Screensaver:**

  * `checkScreensaver()` turns LCD off after `SCREENSAVER_DELAY` without touch; wakes on touch.

# Configuration knobs

* `SCREENSAVER_DELAY` — inactivity timeout (ms)
* HTTP endpoint path and **server IP** (in `connected_callback()` and `tcp_client_connect()`)
* Touch icon position/size (from `next.h` defines; adjust as needed)

# Optional: Extras :
index.php act like a clint to show live values on good User interface website.


---
