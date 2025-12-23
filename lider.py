import time, struct, socket, zlib
from pymavlink import mavutil
import datetime

PIXHAWK_PORT = "udp:127.0.0.1:14550"
BAUD = 0


TARGET_IP = "127.0.0.1"
TARGET_PORT = 7777


TARGET_GS_IP = "127.0.0.1"
TARGET_GS_PORT = 5005
DRONE_ID = 10

MAGIC = b"PGPS"
VER = 2

MODE_OTHER = 0
MODE_GUIDED = 1
MODE_LAND = 2

def open_link(device, baud):
    print(f"Baglanti kuruluyor: {device}")
    m = mavutil.mavlink_connection(device, baud=baud, autoreconnect=True)
    try:
        m.mav.request_data_stream_send(m.target_system, m.target_component,
                                       mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    except Exception:
        pass
    return m

def map_mode_key(hb_msg) -> int:
    try:
        s = mavutil.mode_string_v10(hb_msg).upper()
    except Exception:
        return MODE_OTHER
    if s.startswith("GUIDED"):
        return MODE_GUIDED
    if s.startswith("LAND"):
        return MODE_LAND
    return MODE_OTHER

def pack_v2(ts_ms, lat_e7, lon_e7, alt_rel_mm, armed: bool, mode_key: int, heading_cdeg: int):

    core = MAGIC + struct.pack(">B", VER) + struct.pack(">QiiiBBH",
            ts_ms, lat_e7, lon_e7, alt_rel_mm,
            1 if armed else 0, mode_key, heading_cdeg)
    crc = zlib.crc32(core) & 0xFFFFFFFF
    return core + struct.pack(">I", crc)
def main():
    mav = open_link(PIXHAWK_PORT, BAUD)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    armed = False
    mode_key = MODE_OTHER
    current_mode_str = "UNKNOWN"

    print("Sistem Baslatildi. Veri bekleniyor...")

    while True:
        msg = mav.recv_match(blocking=True, timeout=1.0)
        if not msg:
            continue

        tnow = time.time()

        if msg.get_type() == "HEARTBEAT":
            armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            mode_key = map_mode_key(msg)

            try:
                current_mode_str = mavutil.mode_string_v10(msg).upper()
            except:
                current_mode_str = "UNKNOWN"
            continue

        if msg.get_type() != "GLOBAL_POSITION_INT":
            continue

        lat_e7 = int(msg.lat)
        lon_e7 = int(msg.lon)
        alt_rel_mm = int(msg.relative_alt)
        hdg = int(msg.hdg)
        ts_ms = int(tnow * 1000)


        pkt = pack_v2(ts_ms, lat_e7, lon_e7, alt_rel_mm, armed, mode_key, hdg)
        try:
            sock.sendto(pkt, (TARGET_IP, TARGET_PORT))
        except Exception:
            pass


        state_info = "Lider (SIM)"

        time_str = datetime.datetime.now().strftime("%H:%M:%S")
        alt_m = alt_rel_mm / 1000.0
        log_feedback = f"[{time_str}] GPS OK | Alt: {alt_m:.1f}m | Mod: {current_mode_str}"

        active_bit = 1
        fuel = 100
        crit_fuel = 0
        pid_val = "0.0 0.0"

        new_payload = f"{DRONE_ID} - {state_info} - {active_bit} - {log_feedback} - {current_mode_str} - {fuel} - {crit_fuel} - {pid_val}"

        try:
            sock.sendto(new_payload.encode('utf-8'), (TARGET_GS_IP, TARGET_GS_PORT))


        except Exception:
            pass

if __name__ == "__main__":
    main()