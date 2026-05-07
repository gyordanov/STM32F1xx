#!/usr/bin/env python3
"""
himill_flash.py — Flash HiMill D1/D1S firmware without MaxmakeLAB.

Protocol reverse-engineered from MaxmakeLAB v0.9.16 USB capture on
2026-04-24. See himill_d1s_research/README.md for board background.

Usage:
    tools/himill_flash.py [--port /dev/ttyACM0] [--no-boot] <firmware.bin>
    tools/himill_flash.py stm32 [--port /dev/ttyACM0] [--no-boot] <firmware.bin>
    tools/himill_flash.py esp3d --host <ip-or-url> [--password <admin-pwd>] <firmware.bin>
    tools/himill_flash.py wifi-setup [--port /dev/ttyACM0]
    tools/himill_flash.py wifi-status [--port /dev/ttyACM0]

Examples:
    # Flash our grblHAL build, auto-detect port, let the running app
    # accept the [BOOT] trigger:
    tools/himill_flash.py firmware.bin

    # Restore MaxMake stock firmware:
    tools/himill_flash.py ~/Downloads/do_not_backup/HiMill_D1-v1.0.34-20251222.bin

    # Board is ALREADY in bootloader mode (skip the [BOOT] trigger):
    tools/himill_flash.py --no-boot firmware.bin

    # Flash ESP3D over its HTTP /updatefw endpoint. This does NOT use the
    # SD-card firmware.bin -> firmware.bin.ok flow:
    tools/himill_flash.py esp3d --host 192.168.1.114 firmware_V1.0.2_20260407.bin

    # Configure ESP3D WiFi over the STM32 USB serial bridge:
    tools/himill_flash.py wifi-setup
    tools/himill_flash.py wifi-status

Protocol summary (values in hex, little-endian where multi-byte):
    Trigger (over running firmware's CDC):
        "[BOOT]\\n"                        — enter bootloader

    Handshake (with bootloader):
        host -> dev  51 05 0a              — hello
        dev  -> host a1 02 01              — ready, bootloader v1.2
                     a1 04 01              — ready, bootloader v1.4

    Data transfer (repeat per chunk):
        host -> dev  52 seq[2] 00 04 <1024 firmware bytes>    — full chunk
                     53 seq[2] len[2] 00 <len firmware bytes> — partial (final)
        dev  -> host a2 seq[2] 01          — chunk ack

    Finalize / jump to app:
        host drops DTR and RTS on the CDC port (USB control transfer
        CDC SET_CONTROL_LINE_STATE with DTR=0, RTS=0). The 8 bytes
        21 22 00 00 00 00 00 00 seen in the MaxmakeLAB capture are the
        USB setup packet for this request, NOT application data.
        Earlier versions of this script wrote those bytes as bulk data
        — wrong; the bootloader ignored them. Classic Arduino-style
        "drop DTR to reset into app" trick.
"""

import argparse
import getpass
import http.client
import json
import pathlib
import secrets
import sys
import time
import urllib.parse

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial required: pip install --user pyserial")

try:
    import usb.core
    import usb.util
    HAS_PYUSB = True
except ImportError:
    HAS_PYUSB = False


STM_VID = 0x0483
STM_PID = 0x5740
ESP32_IMAGE_MAGIC = 0xE9

HANDSHAKE_REQ = bytes.fromhex("51050a")
HANDSHAKE_ACK_LEN = 3
HANDSHAKE_ACKS = {
    bytes.fromhex("a10201"): "1.2",
    bytes.fromhex("a10401"): "1.4",
}
NO_WARRANTY_CONFIRMATION = "i agree"

CHUNK_SIZE = 1024
HTTP_UPLOAD_CHUNK_SIZE = 64 * 1024
USB_ESP_BAUDRATE = 115200
USB_ESP_DEFAULT_TIMEOUT = 8.0
USB_ESP_SCAN_TIMEOUT = 25.0
USB_ESP_CONNECT_TIMEOUT = 45.0
CMD_FULL   = 0x52  # full-size chunk
CMD_FINAL  = 0x53  # partial last chunk
ACK_PREFIX = 0xA2  # chunk ack first byte


def find_port():
    """Scan serial ports for the STMicroelectronics Virtual COM Port."""
    matches = [p for p in serial.tools.list_ports.comports()
               if p.vid == STM_VID and p.pid == STM_PID]
    if not matches:
        sys.exit(f"No device found with VID:PID {STM_VID:04x}:{STM_PID:04x}. "
                 "Is the D1S plugged in?")
    if len(matches) > 1:
        print(f"Multiple candidates found, using first: {matches[0].device}",
              file=sys.stderr)
    return matches[0].device


def enter_bootloader(port):
    """Send [BOOT] to the running firmware. The device resets + re-enumerates
    within ~2-3 seconds. Caller should re-find the port after this returns."""
    print(f"→ Sending [BOOT] to {port}")
    with serial.Serial(port, baudrate=115200, timeout=1) as s:
        s.write(b"[BOOT]\n")
        s.flush()
        # Give the firmware a moment to process before it resets itself
        time.sleep(0.3)
    print("  Waiting for bootloader re-enumeration…")
    time.sleep(3.0)


def wait_for_port(timeout=10.0):
    """Poll for the VID:PID to re-appear after a reset."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        matches = [p for p in serial.tools.list_ports.comports()
                   if p.vid == STM_VID and p.pid == STM_PID]
        if matches:
            return matches[0].device
        time.sleep(0.2)
    sys.exit("Bootloader port did not re-appear within timeout.")


def handshake(ser):
    print("→ Handshake")
    ser.write(HANDSHAKE_REQ)
    ser.flush()
    reply = ser.read(HANDSHAKE_ACK_LEN)
    bootloader_version = HANDSHAKE_ACKS.get(reply)
    if bootloader_version is None:
        expected = " or ".join(ack.hex() for ack in HANDSHAKE_ACKS)
        sys.exit(f"Handshake failed: expected {expected}, "
                 f"got {reply.hex() or '(nothing)'}")
    print(f"  OK ({reply.hex()}, bootloader v{bootloader_version})")
    if bootloader_version == "1.2":
        print("  WARNING: bootloader v1.2 is known to be more vulnerable "
              "to power loss during flashing.")
    return bootloader_version


def require_no_warranty_confirmation(firmware_path):
    print()
    print("NO WARRANTY / FLASHING RISK NOTICE")
    print()
    print("This is an unofficial firmware flashing tool for HiMill D1/D1S.")
    print("It is provided as-is, with no warranty and no guarantee of recovery.")
    print("Flashing the wrong file, losing USB connection, or losing power can")
    print("leave the controller unable to boot. Bootloader v1.2 is specifically")
    print("known to be more vulnerable to power loss during flashing.")
    print("This may void vendor support or warranty coverage.")
    print("Before continuing, use stable power, keep the machine in a safe state,")
    print("and make sure the selected firmware is intended for this controller.")
    print()
    print(f"Firmware selected: {firmware_path}")
    print()
    try:
        answer = input(f'Type "{NO_WARRANTY_CONFIRMATION}" to continue: ')
    except EOFError:
        sys.exit("Confirmation required; aborting.")
    if answer.strip().casefold() != NO_WARRANTY_CONFIRMATION:
        sys.exit("Confirmation not accepted; aborting.")
    print()


def send_chunk(ser, seq, chunk, final=False):
    """A chunk is header + data + 1-byte trailing checksum.
    Checksum = sum(data_bytes) & 0xFF — decoded from MaxmakeLAB's capture
    by matching trailer byte to header-free data sum on chunks 0 and 1.
    Without the checksum byte the bootloader still ACKs per-chunk, but
    refuses to commit / jump-to-app at the end — the device stays stuck
    in boot mode."""
    if final:
        # 0x53 seq[2] len[2] <len bytes> checksum[1]
        hdr = bytes([CMD_FINAL,
                     seq & 0xFF, (seq >> 8) & 0xFF,
                     len(chunk) & 0xFF, (len(chunk) >> 8) & 0xFF])
    else:
        # 0x52 seq[2] 00 04 <1024 bytes> checksum[1]
        hdr = bytes([CMD_FULL,
                     seq & 0xFF, (seq >> 8) & 0xFF,
                     0x00, 0x04])

    checksum = sum(chunk) & 0xFF
    ser.write(hdr + chunk + bytes([checksum]))
    ser.flush()

    ack = ser.read(4)
    if len(ack) != 4 or ack[0] != ACK_PREFIX:
        sys.exit(f"\nChunk {seq} NAK: expected a2 xx xx 01, "
                 f"got {ack.hex() or '(nothing)'}")
    ack_seq = ack[1] | (ack[2] << 8)
    if ack_seq != seq:
        sys.exit(f"\nChunk {seq} ACK seq mismatch: got {ack_seq}")


def send_jump_to_app_control_transfer():
    """End-of-flash handshake: reproduce MaxmakeLAB's exact USB sequence
    so the bootloader sees the "we're done" signal it expects.

    Observed in USB capture, the sequence is:
      1. CLEAR_FEATURE(ENDPOINT_HALT) on bulk IN (endpoint 0x81)
      2. CLEAR_FEATURE(ENDPOINT_HALT) on bulk OUT (endpoint 0x01)
      3. CLEAR_FEATURE(ENDPOINT_HALT) on interrupt IN (endpoint 0x82)
      4. CDC SET_CONTROL_LINE_STATE(wValue=0x0000, DTR=0 RTS=0)

    Linux's CDC-ACM driver doesn't issue CLEAR_STALL on non-halted
    endpoints, so pyserial alone skips this. Bypass via pyusb and send
    each transfer explicitly. The bootloader interprets these as
    "flash complete, jump to app" regardless of actual endpoint state.
    """
    if not HAS_PYUSB:
        print("  (pyusb not installed — falling back to pyserial DTR drop; "
              "may not trigger jump-to-app)", file=sys.stderr)
        print("  To install: pip install --user pyusb", file=sys.stderr)
        return

    dev = usb.core.find(idVendor=STM_VID, idProduct=STM_PID)
    if dev is None:
        print("  (no USB device found via pyusb)", file=sys.stderr)
        return

    # The CDC interface contains both bulk data endpoints and is the
    # target for the class-specific control transfer.
    iface = 0

    was_kernel_claimed = False
    try:
        if dev.is_kernel_driver_active(iface):
            dev.detach_kernel_driver(iface)
            was_kernel_claimed = True
    except (usb.core.USBError, NotImplementedError):
        pass

    try:
        # CLEAR_FEATURE(ENDPOINT_HALT) on each of the three endpoints.
        # Standard CDC-ACM layout:
        #   0x81 = bulk IN  (device → host — grblHAL replies, ACKs)
        #   0x01 = bulk OUT (host → device — commands, firmware chunks)
        #   0x82 = interrupt IN (notifications)
        for ep_addr in (0x81, 0x01, 0x82):
            try:
                dev.clear_halt(ep_addr)
            except usb.core.USBError as e:
                # Some kernels refuse CLEAR_HALT on non-halted endpoints
                # — log but keep going; the bootloader may still accept
                # just a subset of the sequence.
                print(f"  clear_halt ep 0x{ep_addr:02x} error "
                      f"(non-fatal): {e}", file=sys.stderr)

        # Final CDC SET_CONTROL_LINE_STATE(0x0000). This is what MaxmakeLAB
        # sends right before the device re-enumerates running the new app.
        dev.ctrl_transfer(
            bmRequestType=0x21,   # host → device, Class, Interface
            bRequest=0x22,        # CDC SET_CONTROL_LINE_STATE
            wValue=0x0000,        # DTR=0, RTS=0
            wIndex=0x0000,
            data_or_wLength=b"",
            timeout=1000,
        )
    except usb.core.USBError as e:
        print(f"  pyusb control transfer error: {e}", file=sys.stderr)
    finally:
        usb.util.dispose_resources(dev)
        if was_kernel_claimed:
            try:
                dev.attach_kernel_driver(iface)
            except usb.core.USBError:
                pass


def flash(port, firmware_path):
    data = pathlib.Path(firmware_path).read_bytes()
    total = len(data)
    full_chunks = total // CHUNK_SIZE
    remainder = total - full_chunks * CHUNK_SIZE

    print(f"→ Flashing {firmware_path} ({total} bytes, "
          f"{full_chunks} full + "
          f"{1 if remainder else 0} partial chunks)")

    with serial.Serial(port, baudrate=115200, timeout=5) as ser:
        handshake(ser)

        for i in range(full_chunks):
            chunk = data[i * CHUNK_SIZE : (i + 1) * CHUNK_SIZE]
            send_chunk(ser, i, chunk, final=False)
            pct = (i + 1) * 100 // (full_chunks + (1 if remainder else 0))
            print(f"  chunk {i+1}/{full_chunks + (1 if remainder else 0)} "
                  f"({pct}%)   ", end="\r", flush=True)

        if remainder:
            tail = data[full_chunks * CHUNK_SIZE:]
            send_chunk(ser, full_chunks, tail, final=True)
            print(f"  chunk {full_chunks+1}/{full_chunks+1} (100%)   ")
        else:
            print()

        # Keep serial port open — we need the USB device alive while we
        # send the final CDC SET_CONTROL_LINE_STATE. pyserial's dtr/rts
        # wrapping emits intermediate 0x02 states that differ from what
        # MaxmakeLAB sends; fall through to pyusb for an exact match.
        pass

    # MaxmakeLAB sends a single CDC SET_CONTROL_LINE_STATE with
    # DTR=0 RTS=0 as its jump-to-app trigger — explicit control
    # transfer, not a pyserial DTR side-effect. Reproduce it verbatim
    # via pyusb so there's no ambiguity about what lands on the wire.
    print("→ Jump to app (explicit CDC SET_CONTROL_LINE_STATE 0x0000)")
    send_jump_to_app_control_transfer()
    time.sleep(0.3)

    print("✓ Flash complete. Device should re-enumerate running the new firmware.")


def normalize_esp3d_base(host):
    """Return (scheme, netloc, path_prefix) for ESP3D HTTP requests."""
    if "://" not in host:
        host = "http://" + host

    parsed = urllib.parse.urlparse(host)
    if parsed.scheme not in ("http", "https") or not parsed.netloc:
        sys.exit(f"Invalid ESP3D host/url: {host!r}")

    return parsed.scheme, parsed.netloc, parsed.path.rstrip("/")


def esp3d_route(base, path):
    return base[2] + path


def esp3d_connection(base, timeout):
    conn_class = (http.client.HTTPSConnection
                  if base[0] == "https" else http.client.HTTPConnection)
    return conn_class(base[1], timeout=timeout)


def esp3d_request(base, method, path, headers=None, body=None, timeout=10.0):
    conn = esp3d_connection(base, timeout)
    try:
        conn.request(method, esp3d_route(base, path),
                     body=body, headers=headers or {})
        response = conn.getresponse()
        payload = response.read()
        return response.status, response.getheaders(), payload
    except OSError as e:
        raise RuntimeError(f"{method} {base[1]}{path} failed: {e}") from e
    finally:
        conn.close()


def header_value(headers, name):
    lname = name.lower()
    for key, value in headers:
        if key.lower() == lname:
            return value
    return None


def response_text(payload):
    return payload.decode("utf-8", errors="replace").strip()


def esp3d_cookie(headers):
    cookie = header_value(headers, "Set-Cookie")
    if not cookie:
        return None
    return cookie.split(";", 1)[0]


def esp3d_login(base, args):
    headers = {}

    if args.password is None:
        if args.user != "admin":
            sys.exit("--user requires --password")
        try:
            status, reply_headers, payload = esp3d_request(
                base, "GET", "/login", timeout=args.timeout)
        except RuntimeError as e:
            sys.exit(str(e))
        if status == 401:
            sys.exit("ESP3D authentication is enabled; rerun with "
                     "--password <admin-password>.")
        if status >= 400:
            sys.exit(f"ESP3D /login returned HTTP {status}: "
                     f"{response_text(payload)}")
        cookie = esp3d_cookie(reply_headers)
        if cookie:
            headers["Cookie"] = cookie
        return headers

    query = urllib.parse.urlencode({
        "USER": args.user,
        "PASSWORD": args.password,
        "SUBMIT": "yes",
    })
    try:
        status, reply_headers, payload = esp3d_request(
            base, "GET", "/login?" + query, timeout=args.timeout)
    except RuntimeError as e:
        sys.exit(str(e))

    if status != 200:
        sys.exit(f"ESP3D login failed with HTTP {status}: "
                 f"{response_text(payload)}")

    cookie = esp3d_cookie(reply_headers)
    if cookie:
        headers["Cookie"] = cookie
    else:
        print("  Login accepted without a session cookie "
              "(authentication may be disabled).")
    return headers


def parse_esp3d_capabilities(text):
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        info = {}
        for line in text.splitlines():
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            info[key.strip()] = value.strip()
        return info

    if isinstance(parsed, dict) and isinstance(parsed.get("data"), dict):
        return parsed["data"]
    if isinstance(parsed, dict):
        return parsed
    return {}


def esp3d_probe(base, headers, timeout):
    query = urllib.parse.urlencode({"cmd": "[ESP800]json"})
    try:
        status, _, payload = esp3d_request(
            base, "GET", "/command?" + query, headers=headers, timeout=timeout)
    except RuntimeError as e:
        sys.exit(str(e))

    text = response_text(payload)
    if status == 401:
        sys.exit("ESP3D rejected the capability probe; check admin password.")
    if status != 200:
        sys.exit(f"ESP3D capability probe failed with HTTP {status}: {text}")

    info = parse_esp3d_capabilities(text)
    if not info:
        sys.exit("ESP3D capability probe did not return usable capabilities. "
                 "Use --skip-probe only if you are sure this is ESP3D.")

    hostname = info.get("Hostname", "?")
    fw_version = info.get("FWVersion", "?")
    web_update = info.get("WebUpdate", "?")
    auth = info.get("Authentication", "?")
    print(f"→ ESP3D: host={hostname} fw={fw_version} "
          f"web-update={web_update} auth={auth}")

    if web_update == "Disabled":
        sys.exit("ESP3D reports WebUpdate=Disabled; cannot use /updatefw.")


def validate_esp3d_firmware(firmware_path, force=False):
    size = firmware_path.stat().st_size
    if size == 0:
        sys.exit(f"Firmware is empty: {firmware_path}")

    with firmware_path.open("rb") as f:
        magic = f.read(1)
    if magic != bytes([ESP32_IMAGE_MAGIC]) and not force:
        sys.exit("This file does not look like an ESP32 app image "
                 "(missing 0xE9 magic byte). Use --force to override.")
    return size


def multipart_token(value):
    return value.replace("\\", "\\\\").replace('"', '\\"')


def esp3d_upload_firmware(base, headers, args, firmware_size):
    upload_name = args.upload_name or args.firmware.name
    upload_name = pathlib.PurePosixPath(upload_name).name
    if "\r" in upload_name or "\n" in upload_name or not upload_name:
        sys.exit(f"Invalid upload filename: {upload_name!r}")

    upload_filename = "/" + upload_name.lstrip("/")
    size_field = upload_filename + "S"
    boundary = "----himill-esp3d-" + secrets.token_hex(8)

    preamble = (
        f"--{boundary}\r\n"
        f"Content-Disposition: form-data; "
        f"name=\"{multipart_token(size_field)}\"\r\n\r\n"
        f"{firmware_size}\r\n"
        f"--{boundary}\r\n"
        f"Content-Disposition: form-data; name=\"myfile[]\"; "
        f"filename=\"{multipart_token(upload_filename)}\"\r\n"
        "Content-Type: application/octet-stream\r\n\r\n"
    ).encode("utf-8")
    epilogue = f"\r\n--{boundary}--\r\n".encode("utf-8")
    total_len = len(preamble) + firmware_size + len(epilogue)

    request_headers = dict(headers)
    request_headers.update({
        "Content-Type": f"multipart/form-data; boundary={boundary}",
        "Content-Length": str(total_len),
        "Connection": "close",
    })

    print(f"→ Uploading ESP3D firmware to {base[0]}://{base[1]}/updatefw "
          f"as {upload_filename} ({firmware_size} bytes)")

    conn = esp3d_connection(base, args.timeout)
    sent = 0
    last_pct = -1

    def send_part(data):
        nonlocal sent, last_pct
        conn.send(data)
        sent += len(data)
        pct = min(100, int((sent * 100) / total_len))
        if pct != last_pct:
            print(f"  upload {pct:3d}%   ", end="\r", flush=True)
            last_pct = pct

    try:
        conn.putrequest("POST", esp3d_route(base, "/updatefw"))
        for key, value in request_headers.items():
            conn.putheader(key, value)
        conn.endheaders()

        send_part(preamble)
        with args.firmware.open("rb") as f:
            while True:
                chunk = f.read(HTTP_UPLOAD_CHUNK_SIZE)
                if not chunk:
                    break
                send_part(chunk)
        send_part(epilogue)
        print("  upload 100%   ")

        response = conn.getresponse()
        payload = response.read()
        status = response.status
    except OSError as e:
        sys.exit(f"ESP3D upload failed: {e}")
    finally:
        conn.close()

    text = response_text(payload)
    if status == 401:
        sys.exit("ESP3D rejected the firmware upload; check admin password.")
    if status != 200:
        sys.exit(f"ESP3D firmware upload returned HTTP {status}: {text}")

    try:
        reply = json.loads(text)
    except json.JSONDecodeError:
        sys.exit(f"ESP3D firmware upload returned non-JSON response: {text}")

    upload_status = str(reply.get("status", "")).lower()
    if upload_status not in ("ok", "3", "success", "successful"):
        sys.exit(f"ESP3D firmware update failed: {text}")

    print("✓ ESP3D accepted firmware update; reboot should start now.")


def esp3d_wait_for_reboot(base, timeout):
    print("→ Waiting for ESP3D HTTP service to come back")
    deadline = time.time() + timeout
    time.sleep(2.0)
    while time.time() < deadline:
        try:
            status, _, _ = esp3d_request(base, "GET", "/login", timeout=2.0)
            if status < 500:
                print("✓ ESP3D is reachable again.")
                return
        except RuntimeError:
            pass
        time.sleep(1.0)
    print("  Timed out waiting for ESP3D to come back. The upload was accepted, "
          "so check the board manually before retrying.", file=sys.stderr)


def usb_esp_escape_param(value):
    if value is None:
        return ""
    if "\r" in value or "\n" in value:
        sys.exit("ESP3D command parameters cannot contain newlines.")
    if "\\" in value:
        sys.exit("ESP3D serial command escaping cannot safely carry backslashes.")
    return "".join(("\\" + c) if c.isspace() else c for c in value)


def usb_esp_command_text(cmd, param=None, admin_password=None):
    text = f"[ESP{cmd}]"
    if param is not None:
        text += usb_esp_escape_param(param)
    text += " json"
    if admin_password:
        text += " pwd=" + usb_esp_escape_param(admin_password)
    return text


def extract_json_objects(text):
    objects = []
    start = None
    depth = 0
    in_string = False
    escaped = False

    for idx, char in enumerate(text):
        if start is None:
            if char == "{":
                start = idx
                depth = 1
                in_string = False
                escaped = False
            continue

        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue

        if char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                candidate = text[start:idx + 1]
                try:
                    objects.append(json.loads(candidate))
                except json.JSONDecodeError:
                    pass
                start = None

    return objects


def find_esp3d_json(raw, cmd):
    cmd = str(cmd)
    fallback = None
    for obj in extract_json_objects(raw):
        if fallback is None:
            fallback = obj
        if str(obj.get("cmd", "")) == cmd:
            return obj
    return fallback


def open_usb_esp_serial(port):
    ser = serial.Serial(port, baudrate=USB_ESP_BAUDRATE, timeout=0.1,
                        write_timeout=2)
    ser.dtr = True
    ser.rts = False
    time.sleep(0.25)
    ser.reset_input_buffer()
    return ser


def usb_esp_command(ser, command, expected_cmd, timeout=USB_ESP_DEFAULT_TIMEOUT,
                    raw=False):
    ser.reset_input_buffer()
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()

    deadline = time.time() + timeout
    buffer = bytearray()
    while time.time() < deadline:
        chunk = ser.read(512)
        if chunk:
            buffer.extend(chunk)
            text = buffer.decode("utf-8", errors="replace")
            obj = find_esp3d_json(text, expected_cmd)
            if obj is not None:
                return obj, text
        else:
            time.sleep(0.03)

    text = buffer.decode("utf-8", errors="replace")
    if raw:
        return None, text
    sys.exit(f"No ESP3D JSON response for {command!r} within {timeout:.1f}s.\n"
             f"Raw response:\n{text.strip() or '(empty)'}")


def ensure_esp_ok(response, label):
    if str(response.get("status", "")).lower() != "ok":
        sys.exit(f"{label} failed: {json.dumps(response, ensure_ascii=False)}")


def esp420_map(response):
    data = response.get("data", [])
    if not isinstance(data, list):
        return {}
    result = {}
    for item in data:
        if isinstance(item, dict) and "id" in item:
            result[str(item["id"])] = str(item.get("value", ""))
    return result


def print_wifi_status_from_map(status):
    hostname = status.get("hostname", "?")
    wifi = status.get("wifi", "?")
    mode = status.get("wifi mode", "?")
    ssid = status.get("SSID", "")
    ip = status.get("ip", "")
    signal = status.get("signal", "")

    print(f"Hostname: {hostname}")
    print(f"WiFi:     {wifi}")
    print(f"Mode:     {mode}")
    print(f"SSID:     {ssid or '(not connected)'}")
    print(f"IP:       {ip or '(none)'}")
    if signal:
        print(f"Signal:   {signal}")


def parse_scan_networks(response):
    data = response.get("data", [])
    if not isinstance(data, list):
        return []

    best_by_ssid = {}
    for item in data:
        if not isinstance(item, dict):
            continue
        ssid = str(item.get("SSID", ""))
        if not ssid:
            continue
        try:
            signal = int(str(item.get("SIGNAL", "0")).rstrip("%"))
        except ValueError:
            signal = 0
        protected = str(item.get("IS_PROTECTED", "1")) == "1"
        current = best_by_ssid.get(ssid)
        if current is None or signal > current["signal"]:
            best_by_ssid[ssid] = {
                "ssid": ssid,
                "signal": signal,
                "protected": protected,
            }

    return sorted(best_by_ssid.values(), key=lambda item: item["signal"],
                  reverse=True)


def choose_wifi_network(networks):
    if not networks:
        sys.exit("No WiFi networks found.")

    print("\nAvailable WiFi networks:")
    for idx, network in enumerate(networks, start=1):
        security = "secure" if network["protected"] else "open"
        print(f"  {idx:2d}. {network['ssid']} "
              f"({network['signal']}%, {security})")

    while True:
        choice = input(f"Select network [1-{len(networks)}]: ").strip()
        if choice.isdigit():
            index = int(choice)
            if 1 <= index <= len(networks):
                return networks[index - 1]
        print("Invalid selection.")


def wait_for_wifi_connection(ser, ssid, timeout, admin_password=None):
    deadline = time.time() + timeout
    last_status = {}
    while time.time() < deadline:
        try:
            response, _ = usb_esp_command(
                ser,
                usb_esp_command_text("420", admin_password=admin_password),
                "420",
                timeout=USB_ESP_DEFAULT_TIMEOUT,
                raw=True,
            )
        except (serial.SerialException, OSError):
            response = None

        if response:
            last_status = esp420_map(response)
            current_ssid = last_status.get("SSID", "")
            current_ip = last_status.get("ip", "")
            if current_ssid == ssid and current_ip and current_ip != "0.0.0.0":
                return last_status

        time.sleep(2.0)

    return last_status


def main_wifi_status(argv=None):
    ap = argparse.ArgumentParser(
        prog=f"{pathlib.Path(sys.argv[0]).name} wifi-status",
        description="Print ESP3D WiFi SSID and IP over STM32 USB serial.")
    ap.add_argument("--port",
                    help="Serial port (auto-detect via VID:PID if omitted)")
    ap.add_argument("--timeout", type=float, default=USB_ESP_DEFAULT_TIMEOUT)
    ap.add_argument("--admin-password",
                    help="ESP3D admin password if authentication is enabled")
    ap.add_argument("--raw", action="store_true",
                    help="Print the raw ESP3D JSON response too")
    args = ap.parse_args(argv)

    port = args.port or find_port()
    with open_usb_esp_serial(port) as ser:
        response, raw = usb_esp_command(
            ser,
            usb_esp_command_text("420", admin_password=args.admin_password),
            "420",
            timeout=args.timeout,
        )

    if args.raw:
        print(raw.strip())
    print_wifi_status_from_map(esp420_map(response))


def main_wifi_setup(argv=None):
    ap = argparse.ArgumentParser(
        prog=f"{pathlib.Path(sys.argv[0]).name} wifi-setup",
        description="Scan and configure ESP3D WiFi over STM32 USB serial.")
    ap.add_argument("--port",
                    help="Serial port (auto-detect via VID:PID if omitted)")
    ap.add_argument("--ssid",
                    help="Skip scan selection and use this SSID")
    ap.add_argument("--password",
                    help="WiFi password; prompts securely if omitted")
    ap.add_argument("--admin-password",
                    help="ESP3D admin password if authentication is enabled")
    ap.add_argument("--scan-timeout", type=float, default=USB_ESP_SCAN_TIMEOUT)
    ap.add_argument("--connect-timeout", type=float,
                    default=USB_ESP_CONNECT_TIMEOUT)
    ap.add_argument("--no-restart", action="store_true",
                    help="Save settings but do not send [ESP444]RESTART")
    ap.add_argument("--no-wait", action="store_true",
                    help="Do not wait for WiFi to reconnect after restart")
    args = ap.parse_args(argv)

    port = args.port or find_port()
    with open_usb_esp_serial(port) as ser:
        print(f"→ USB serial: {port}")
        print("→ Enabling ESP3D WIFI-STA mode")
        response, _ = usb_esp_command(
            ser,
            usb_esp_command_text("110", "WIFI-STA",
                                 admin_password=args.admin_password),
            "110",
            timeout=USB_ESP_DEFAULT_TIMEOUT,
        )
        ensure_esp_ok(response, "WIFI-STA mode")

        if args.ssid:
            selected = {"ssid": args.ssid, "protected": True, "signal": 0}
        else:
            print("→ Scanning WiFi networks")
            response, _ = usb_esp_command(
                ser,
                usb_esp_command_text("410", admin_password=args.admin_password),
                "410",
                timeout=args.scan_timeout,
            )
            ensure_esp_ok(response, "WiFi scan")
            selected = choose_wifi_network(parse_scan_networks(response))

        ssid = selected["ssid"]
        if args.password is not None:
            password = args.password
        elif selected["protected"]:
            password = getpass.getpass(f"Password for {ssid}: ")
        else:
            password = ""

        print(f"→ Setting STA SSID: {ssid}")
        response, _ = usb_esp_command(
            ser,
            usb_esp_command_text("100", ssid,
                                 admin_password=args.admin_password),
            "100",
            timeout=USB_ESP_DEFAULT_TIMEOUT,
        )
        ensure_esp_ok(response, "Set SSID")

        print("→ Setting STA password")
        pass_param = password if password else "NOPASSWORD"
        response, _ = usb_esp_command(
            ser,
            usb_esp_command_text("101", pass_param,
                                 admin_password=args.admin_password),
            "101",
            timeout=USB_ESP_DEFAULT_TIMEOUT,
        )
        ensure_esp_ok(response, "Set password")

        if args.no_restart:
            print("✓ WiFi settings saved. Restart skipped.")
            return

        print("→ Restarting ESP3D")
        response, _ = usb_esp_command(
            ser,
            usb_esp_command_text("444", "RESTART",
                                 admin_password=args.admin_password),
            "444",
            timeout=USB_ESP_DEFAULT_TIMEOUT,
        )
        ensure_esp_ok(response, "ESP3D restart")

        if args.no_wait:
            print("✓ ESP3D restart requested.")
            return

        time.sleep(5.0)
        status = wait_for_wifi_connection(
            ser, ssid, args.connect_timeout,
            admin_password=args.admin_password,
        )

    if status.get("SSID") == ssid and status.get("ip") not in ("", "0.0.0.0"):
        print("✓ ESP3D connected:")
    else:
        print("  ESP3D did not report a completed STA connection before timeout.")
    print_wifi_status_from_map(status)


def main_esp3d(argv=None):
    ap = argparse.ArgumentParser(
        prog=f"{pathlib.Path(sys.argv[0]).name} esp3d",
        description="Flash ESP3D firmware over the HTTP /updatefw endpoint.")
    ap.add_argument("firmware", type=pathlib.Path,
                    help="ESP3D/ESP32 firmware .bin")
    ap.add_argument("--host", required=True,
                    help="ESP3D IP/host or URL, e.g. 192.168.1.114")
    ap.add_argument("--user", default="admin",
                    help="Admin username when authentication is enabled "
                         "(default: admin)")
    ap.add_argument("--password",
                    help="Admin password when authentication is enabled")
    ap.add_argument("--upload-name",
                    help="Multipart filename to present to ESP3D "
                         "(default: firmware basename)")
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="HTTP request timeout in seconds (default: 15)")
    ap.add_argument("--reboot-timeout", type=float, default=60.0,
                    help="Seconds to wait for ESP3D after update (default: 60)")
    ap.add_argument("--skip-probe", action="store_true",
                    help="Skip [ESP800] capability probe before upload")
    ap.add_argument("--dry-run", action="store_true",
                    help="Validate firmware and ESP3D endpoint, but do not "
                         "upload")
    ap.add_argument("--no-wait", action="store_true",
                    help="Do not wait for ESP3D to reboot after upload")
    ap.add_argument("--force", action="store_true",
                    help="Allow flashing a file without ESP32 image magic")
    args = ap.parse_args(argv)

    if not args.firmware.exists():
        sys.exit(f"Firmware not found: {args.firmware}")
    if not args.firmware.is_file():
        sys.exit(f"Firmware is not a file: {args.firmware}")

    firmware_size = validate_esp3d_firmware(args.firmware, args.force)
    base = normalize_esp3d_base(args.host)
    headers = esp3d_login(base, args)
    if not args.skip_probe:
        esp3d_probe(base, headers, args.timeout)
    if args.dry_run:
        print("✓ Dry run complete; firmware was not uploaded.")
        return

    esp3d_upload_firmware(base, headers, args, firmware_size)
    if not args.no_wait:
        esp3d_wait_for_reboot(base, args.reboot_timeout)


def main_stm32(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("firmware", type=pathlib.Path,
                    help="Firmware .bin file to flash (e.g. firmware.bin from "
                         "a PlatformIO grblHAL build, or a stock MaxMake .bin)")
    ap.add_argument("--port",
                    help="Serial port (auto-detect via VID:PID if omitted)")
    ap.add_argument("--no-boot", action="store_true",
                    help="Skip the [BOOT] trigger; assume device is already "
                         "in bootloader mode (e.g. after a previous failed flash)")
    args = ap.parse_args(argv)

    if not args.firmware.exists():
        sys.exit(f"Firmware not found: {args.firmware}")
    if not args.firmware.is_file():
        sys.exit(f"Firmware is not a file: {args.firmware}")

    port = args.port or find_port()
    require_no_warranty_confirmation(args.firmware)

    if not args.no_boot:
        enter_bootloader(port)
        port = wait_for_port()

    print(f"→ Bootloader port: {port}")
    flash(port, args.firmware)


def main():
    argv = sys.argv[1:]
    if argv and argv[0] in ("esp3d", "esp"):
        main_esp3d(argv[1:])
    elif argv and argv[0] in ("wifi-setup", "wifi"):
        main_wifi_setup(argv[1:])
    elif argv and argv[0] in ("wifi-status", "wifi-info"):
        main_wifi_status(argv[1:])
    elif argv and argv[0] == "stm32":
        main_stm32(argv[1:])
    else:
        main_stm32(argv)


if __name__ == "__main__":
    main()
