import math, time, threading, struct, socket, zlib, logging, queue
from pymavlink import mavutil
import datetime

# --- SİMÜLASYON NOTU ---
time.sleep(1.0)

# ---------------- LOGGING ----------------
logging.basicConfig(
    filename="follower_middle_log_SIM.txt",
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    force=True
)

logger = logging.getLogger()


class FlushFileHandler(logging.FileHandler):
    def emit(self, record):
        super().emit(record)
        self.flush()


for h in logger.handlers:
    logger.removeHandler(h)

flush_handler = FlushFileHandler("follower_middle_log_SIM.txt")
formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
flush_handler.setFormatter(formatter)
logger.addHandler(flush_handler)
logger.setLevel(logging.DEBUG)

# ---------------- AĞ VE BAĞLANTI AYARLARI (DRONE 20) ----------------

# Drone 20 Bağlantısı (SITL Instance 2)
FOLLOWER_CONNECTION = "udp:127.0.0.1:14560"
BAUD = 0

# KİMİ DİNLİYORUZ? -> Drone 10'u (Lider)
LISTEN_IP = "127.0.0.1"
LISTEN_PORT = 7777

# Arkadakine (Drone 30) Yayın Yap
BROADCAST_IP = "127.0.0.1"
BROADCAST_PORT = 7788

# Yer İstasyonu
GS_IP = "127.0.0.1"
GS_PORT = 5005
DRONE_ID = 20

# ----------------------------------------------------------------------

LOOP_HZ = 15.0
DT = 1.0 / LOOP_HZ
DIST_SET = 15.0
BAND_MIN = 14.90
BAND_MAX = 15.05
VXY_MAX = 2.0
VZ_MAX = 0.8
WATCHDOG_S = 1.0

MIN_SAFE_DIST = 7.0  # 7 metre fazla yaklaşırsan kaçış modu devreye girsin

# EŞİK DEĞERİ (3 metreden uzaksak lidere bak, yakınsak hizalan)
YAW_ALIGN_THRESH = 3.0

KP_POS = 0.5
KP_ALT = 0.3

MAGIC = b"PGPS"
VER_V2 = 2

MODE_OTHER = 0
MODE_GUIDED = 1
MODE_LAND = 2

state = {
    "leader_lat": None,
    "leader_lon": None,
    "leader_alt_rel": None,
    "leader_armed": None,
    "leader_mode_key": MODE_OTHER,
    "leader_heading": 0.0,  # Liderin Yönü
    "t_last_leader": 0.0,
    "follower_lat": None,
    "follower_lon": None,
    "follower_alt_rel": None,
    "follower_armed": False,
    "follower_heading": 0.0,  # Kendi Yönümüz
    "follower_mode_key": MODE_OTHER,
    "follower_mode_str": "UNKNOWN",
    "log_msg": "Sistem Baslatiliyor (Drone 20)...",
    "pid_h": 0.0,
    "pid_v": 0.0
}

lock = threading.Lock()


# ----------------------------------------------------------------------
# MATEMATİK
# ----------------------------------------------------------------------

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def meters_per_deg(lat_deg: float):
    lat = math.radians(lat_deg)
    m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat) + 1.175 * math.cos(4 * lat)
    m_per_deg_lon = 111412.84 * math.cos(lat) - 93.5 * math.cos(3 * lat)
    return m_per_deg_lat, m_per_deg_lon


def latlon_to_local_xy(lat_ref, lon_ref, lat, lon):
    mlat, mlon = meters_per_deg(lat_ref)
    dx = (lon - lon_ref) * mlon
    dy = (lat - lat_ref) * mlat
    return dx, dy


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (
            math.sin(dlat / 2) ** 2
            + math.cos(math.radians(lat1))
            * math.cos(math.radians(lat2))
            * math.sin(dlon / 2) ** 2
    )
    return 2 * R * math.asin(math.sqrt(a))


# ----------------------------------------------------------------------
# MAVLINK BAĞLANTISI
# ----------------------------------------------------------------------

def open_link(connection_string):
    print(f"Baglanti kuruluyor (Drone 20): {connection_string}")
    m = mavutil.mavlink_connection(connection_string, baud=BAUD, autoreconnect=True)
    m.wait_heartbeat()
    print("Baglanti Basarili!")
    try:
        m.mav.request_data_stream_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
        )
    except Exception as e:
        print(f"Stream hatasi: {e}")
    return m


follower = open_link(FOLLOWER_CONNECTION)
cmd_queue = queue.Queue(maxsize=50)
BOOT_T0 = time.monotonic()


def time_boot_ms():
    return int((time.monotonic() - BOOT_T0) * 1000) & 0xFFFFFFFF


# ----------------------------------------------------------------------
# VELOCITY + YAW KOMUTU
# ----------------------------------------------------------------------

def _send_velocity_yaw_internal(conn, vx, vy, vz, target_yaw_deg):
    # Hız ve Yaw Aktif (Mask: 2503)
    type_mask = 2503
    yaw_rad = math.radians(target_yaw_deg)

    try:
        conn.mav.set_position_target_local_ned_send(
            0,
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,  # Pos
            vx, vy, vz,  # Vel
            0, 0, 0,  # Acc
            yaw_rad, 0  # Yaw, YawRate
        )
    except Exception as e:
        print(f"Komut Hatasi: {e}")


def _set_mode_internal(conn, mode_name):
    try:
        mode_id = conn.mode_mapping().get(mode_name)
        if mode_id is not None:
            conn.mav.set_mode_send(
                conn.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
    except Exception:
        pass


def send_velocity_yaw(vx, vy, vz, yaw_deg):
    try:
        cmd_queue.put_nowait({"type": "velocity", "vx": vx, "vy": vy, "vz": vz, "yaw": yaw_deg})
    except queue.Full:
        pass


def set_mode_guided():
    try:
        cmd_queue.put_nowait({"type": "mode", "mode": "GUIDED"})
    except queue.Full:
        pass


def set_mode_land():
    try:
        cmd_queue.put_nowait({"type": "mode", "mode": "LAND"})
    except queue.Full:
        pass


def follower_writer(conn):
    while True:
        try:
            cmd = cmd_queue.get(timeout=0.1)
            if cmd["type"] == "velocity":
                _send_velocity_yaw_internal(conn, cmd["vx"], cmd["vy"], cmd["vz"], cmd["yaw"])
            elif cmd["type"] == "mode":
                _set_mode_internal(conn, cmd["mode"])
            cmd_queue.task_done()
        except queue.Empty:
            continue
        except Exception:
            pass


threading.Thread(target=follower_writer, args=(follower,), daemon=True).start()


# ----------------------------------------------------------------------
# OKUYUCU THREADLER
# ----------------------------------------------------------------------

def map_mode_key(hb_msg) -> int:
    try:
        s = mavutil.mode_string_v10(hb_msg).upper()
    except:
        return MODE_OTHER
    if "GUIDED" in s: return MODE_GUIDED
    if "LAND" in s: return MODE_LAND
    return MODE_OTHER


def follower_reader(conn):
    while True:
        msg = conn.recv_match(blocking=True, timeout=1.0)
        if not msg: continue
        msg_type = msg.get_type()

        with lock:
            if msg_type == "HEARTBEAT":
                state["follower_armed"] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                state["follower_mode_key"] = map_mode_key(msg)
                try:
                    state["follower_mode_str"] = mavutil.mode_string_v10(msg).upper()
                except:
                    state["follower_mode_str"] = "UNKNOWN"

            elif msg_type == "GLOBAL_POSITION_INT":
                state["follower_lat"] = msg.lat / 1e7
                state["follower_lon"] = msg.lon / 1e7
                state["follower_alt_rel"] = msg.relative_alt / 1000.0
                state["follower_heading"] = msg.hdg / 100.0


threading.Thread(target=follower_reader, args=(follower,), daemon=True).start()



# UDP DİNLEYİCİ (Drone 10'dan Gelen Veri)


def parse_udp_packet(data: bytes):
    if len(data) != 33: return None
    core = data[:-4]
    crc_recv = struct.unpack(">I", data[-4:])[0]
    if (zlib.crc32(core) & 0xFFFFFFFF) != crc_recv: return None
    if core[:4] != MAGIC: return None

    ts_ms, lat_e7, lon_e7, alt_rel_mm, flags, mode_key, hdg_cdeg = struct.unpack(">QiiiBBH", core[5:])

    return {
        "lat": lat_e7 / 1e7,
        "lon": lon_e7 / 1e7,
        "alt_rel": alt_rel_mm / 1000.0,
        "armed": bool(flags & 0x01),
        "mode_key": mode_key,
        "heading": hdg_cdeg / 100.0
    }


def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(1.0)
    print(f"Drone 10 (Lider) Dinleniyor (UDP): {LISTEN_PORT}")

    while True:
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            continue

        parsed = parse_udp_packet(data)
        if not parsed: continue

        with lock:
            state["leader_lat"] = parsed["lat"]
            state["leader_lon"] = parsed["lon"]
            state["leader_alt_rel"] = parsed["alt_rel"]
            state["leader_armed"] = parsed["armed"]
            state["leader_mode_key"] = parsed["mode_key"]
            state["leader_heading"] = parsed["heading"]
            state["t_last_leader"] = time.time()


threading.Thread(target=udp_listener, daemon=True).start()



# BROADCAST & GS


def pack_v2(ts_ms, lat_e7, lon_e7, alt_rel_mm, armed: bool, mode_key: int, heading_cdeg: int):
    core = MAGIC + struct.pack(">B", VER_V2) + struct.pack(">QiiiBBH",
                                                           ts_ms, lat_e7, lon_e7, alt_rel_mm, 1 if armed else 0,
                                                           mode_key, heading_cdeg)
    crc = zlib.crc32(core) & 0xFFFFFFFF
    return core + struct.pack(">I", crc)


def position_broadcaster():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    while True:
        time.sleep(0.1)

        with lock:
            lat = state["follower_lat"]
            lon = state["follower_lon"]
            alt_rel = state["follower_alt_rel"]
            armed = state["follower_armed"]
            mode_key = state["follower_mode_key"]
            hdg = state["follower_heading"]

            mode_str = state["follower_mode_str"]
            log_msg = state["log_msg"]
            pid_h_val = state["pid_h"]
            pid_v_val = state["pid_v"]

        if lat is None or lon is None: continue

        try:
            # Arkadaki (Drone 30) için yayın
            pkt = pack_v2(int(time.time() * 1000), int(lat * 1e7), int(lon * 1e7),
                          int(alt_rel * 1000), armed, mode_key, int(hdg * 100))
            sock.sendto(pkt, (BROADCAST_IP, BROADCAST_PORT))

            # GS Raporu
            tr_state = "takip" if mode_key == MODE_GUIDED else "stand by"
            pid_data = f"{pid_h_val:.2f} {pid_v_val:.2f}"
            t_str = datetime.datetime.now().strftime("%H:%M:%S")
            full_log = f"[{t_str}] {log_msg}"

            payload = f"{DRONE_ID} - {tr_state} - 1 - {full_log} - {mode_str} - 100 - 0 - {pid_data}"
            sock.sendto(payload.encode('utf-8'), (GS_IP, GS_PORT))

        except Exception:
            pass


threading.Thread(target=position_broadcaster, daemon=True).start()



# MAIN LOOP (GÜNCELLENMİŞ MANTIK)


def main():
    print("Sistem Basladi (Drone 20). Veriler bekleniyor...")

    while True:
        t0 = time.time()
        with lock:
            L_lat = state["leader_lat"]
            L_lon = state["leader_lon"]
            L_alt = state["leader_alt_rel"]
            L_mode = state["leader_mode_key"]
            L_hdg = state["leader_heading"]

            F_lat = state["follower_lat"]
            F_lon = state["follower_lon"]
            F_alt = state["follower_alt_rel"]

        have_leader = L_lat is not None and L_lon is not None
        have_follower = F_lat is not None and F_lon is not None

        if have_follower and L_mode == MODE_LAND:
            set_mode_land()
            with lock: state["log_msg"] = "Lider Iniste -> LAND"

        if have_leader and have_follower:
            # 1. Liderin bize göre konumu
            dx_real, dy_real = latlon_to_local_xy(F_lat, F_lon, L_lat, L_lon)

            # 2. Kuyruk Takibi (Tail Chasing)
            L_yaw_rad = math.radians(L_hdg)

            # Navigasyon koordinat düzeltmesi (Sin->Doğu, Cos->Kuzey)
            target_offset_x = -math.sin(L_yaw_rad) * DIST_SET
            target_offset_y = -math.cos(L_yaw_rad) * DIST_SET

            # 3. Hata Vektörü
            err_x = dx_real + target_offset_x
            err_y = dy_real + target_offset_y

            # Hedef noktaya (Liderin arkasındaki kutuya) olan mesafe
            dist_to_target = math.hypot(err_x, err_y)

            # Fazla yaklaşınca kaçış ve çarpma önleme algoritması çalşacak
            dist_to_leader = math.hypot(dx_real, dy_real)
            is_emergency = False

            if dist_to_leader < MIN_SAFE_DIST:
                is_emergency = True
                kacis_siddeti = (MIN_SAFE_DIST - dist_to_leader) * 2.0
                err_x = -dx_real * kacis_siddeti
                err_y = -dy_real * kacis_siddeti

            # 4. Hız Komutu (PID)
            vx = clamp(err_x * KP_POS, -VXY_MAX, VXY_MAX)
            vy = clamp(err_y * KP_POS, -VXY_MAX, VXY_MAX)

            alt_err = L_alt - F_alt
            vz = clamp(-KP_ALT * alt_err, -VZ_MAX, VZ_MAX)

            # 5. AKILLI YAW
            target_yaw = 0.0
            yaw_msg = ""

            print(f'lidere uzaklık = {dist_to_leader}')

            if dist_to_target > YAW_ALIGN_THRESH:
                # Hedef noktadan uzağız (>3m) -> Lidere Bak (Look-at)
                target_yaw = math.degrees(math.atan2(dx_real, dy_real))
                if target_yaw < 0: target_yaw += 360
                yaw_msg = "Lidere Bakiyor"
            else:
                # Hedef noktadayız (<3m) -> Liderle Hizalan (Align-to)
                target_yaw = L_hdg
                yaw_msg = "Hizalandi"

            send_velocity_yaw(vy, vx, 0, target_yaw)

            # Loglama
            dist_real = math.hypot(dx_real, dy_real)
            with lock:
                state["pid_h"] = math.hypot(vx, vy)
                state["pid_v"] = vz
                state["log_msg"] = f"Mesafe:{dist_real:.1f}m | Hata:{dist_to_target:.1f}m | {yaw_msg}"

        else:
            with lock:
                state["log_msg"] = "Drone 10 (Lider) verisi bekleniyor..."
                state["pid_h"] = 0.0

        dt = time.time() - t0
        if (sleep := DT - dt) > 0:
            time.sleep(sleep)


if __name__ == "__main__":
    main()