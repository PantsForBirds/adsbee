"""Mode S encoders for gen.py: CPR airborne positions (after github.com/lyusupov/ADSB-Out) and TC19 velocities."""
import math

def nl(lat: float) -> int:
    """
    Calculate NL function - number of longitude zones at a given latitude.
    Used for CPR encoding.
    """
    if abs(lat) >= 87.0:
        return 1

    # Number of latitude zones (NZ) for airborne CPR
    latz = 15.0

    return int(math.floor(
        (2.0 * math.pi) * math.acos(
            1.0 - (1.0 - math.cos(math.pi / (2.0 * latz))) /
            math.cos((math.pi / 180.0) * abs(lat)) ** 2
        ) ** -1
    ))


def cpr_encode(lat: float, lon: float, is_odd: bool) -> tuple[int, int]:
    """
    Encode latitude and longitude using CPR (Compact Position Reporting).

    Args:
        lat: Latitude in decimal degrees
        lon: Longitude in decimal degrees
        is_odd: True for odd frame (F=1), False for even frame (F=0)

    Returns:
        Tuple of (encoded_lat, encoded_lon) as 17-bit integers
    """
    # Airborne CPR uses NZ = 15
    latz = 15.0

    # Calculate dLat - latitude zone size
    if is_odd:
        dlat = 360.0 / (4.0 * latz - 1.0)  # Odd: 360/59
    else:
        dlat = 360.0 / (4.0 * latz)  # Even: 360/60

    # Calculate latitude index
    yz = math.floor((1 << 17) * ((lat % dlat) / dlat) + 0.5)
    rlat = dlat * ((yz / (1 << 17)) + math.floor(lat / dlat))

    # Calculate dLon - longitude zone size
    if is_odd:
        nl_val = nl(rlat) - 1
    else:
        nl_val = nl(rlat)

    if nl_val > 0:
        dlon = 360.0 / nl_val
    else:
        dlon = 360.0

    # Calculate longitude index
    xz = math.floor((1 << 17) * ((lon % dlon) / dlon) + 0.5)

    # Mask to 17 bits
    lat_cpr = int(yz) & 0x1FFFF
    lon_cpr = int(xz) & 0x1FFFF

    return (lat_cpr, lon_cpr)


def encode_altitude(alt_feet: int) -> int:
    """
    Encode altitude in feet to 12-bit Mode S format with Q-bit encoding.

    Args:
        alt_feet: Altitude in feet

    Returns:
        12-bit encoded altitude value
    """
    # Q-bit encoding: altitude = 25N - 1000 feet
    # Rearranging: N = (altitude + 1000) / 25
    enc_alt = int((alt_feet + 1000) / 25)

    # Split into upper and lower parts with Q-bit in the middle
    # Format: D2 D4 D1 D3 [Q=1] B1 B2 B4 A1 A2 A4 C1 C2 C4
    # We'll use a simplified encoding where Q=1 (bit 4 from right, 0-indexed)

    # Take the 11-bit value and insert Q=1 at bit position 4
    upper = (enc_alt & 0x7F0) << 1  # Upper 7 bits
    lower = enc_alt & 0x00F          # Lower 4 bits
    q_bit = 0x10                      # Q bit = 1

    return (upper | q_bit | lower) & 0xFFF


def mode_s_crc(msg_bits: str) -> str:
    """
    Calculate Mode S CRC-24 checksum.

    Args:
        msg_bits: Message as binary string (without CRC)

    Returns:
        24-bit CRC as binary string
    """
    # Mode S generator polynomial
    generator = "1111111111111010000001001"

    # Pad message with 24 zeros
    msg = msg_bits + "0" * 24
    msg_list = list(msg)

    # Perform polynomial division
    for i in range(len(msg_bits)):
        if msg_list[i] == '1':
            for j in range(len(generator)):
                if msg_list[i + j] == generator[j]:
                    msg_list[i + j] = '0'
                else:
                    msg_list[i + j] = '1'

    # Return last 24 bits (remainder)
    return ''.join(msg_list[-24:])


def encode_airborne_position(icao: str, lat: float, lon: float, alt_feet: int, is_odd: bool) -> str:
    """
    Encode a DF17 airborne position message.

    Args:
        icao: 6-character hex ICAO address (e.g., "ABC123")
        lat: Latitude in decimal degrees
        lon: Longitude in decimal degrees
        alt_feet: Altitude in feet
        is_odd: True for odd frame, False for even frame

    Returns:
        28-character hex string (112 bits / 14 bytes)
    """
    # DF = 17 (5 bits): 10001
    df = 17

    # CA = 6 (3 bits): capability (level 2+ transponder)
    ca = 6

    # ICAO address (24 bits)
    icao_int = int(icao, 16)

    # ME field (56 bits):
    # TC = 11 (5 bits): Type code for airborne position with barometric altitude
    tc = 11

    # SS = 0 (2 bits): Surveillance status (no alert, no SPI)
    ss = 0

    # NICsb = 0 (1 bit): NIC supplement-B
    nicsb = 0

    # Altitude (12 bits)
    alt_enc = encode_altitude(alt_feet)

    # T = 0 (1 bit): Time flag (not synchronized)
    t = 0

    # F = is_odd (1 bit): CPR format (0=even, 1=odd)
    f = 1 if is_odd else 0

    # CPR encoded position (17 bits each)
    lat_cpr, lon_cpr = cpr_encode(lat, lon, is_odd)

    # Build message bits (88 bits before CRC)
    msg_bits = ""
    msg_bits += f"{df:05b}"           # DF: 5 bits
    msg_bits += f"{ca:03b}"           # CA: 3 bits
    msg_bits += f"{icao_int:024b}"    # ICAO: 24 bits
    msg_bits += f"{tc:05b}"           # TC: 5 bits
    msg_bits += f"{ss:02b}"           # SS: 2 bits
    msg_bits += f"{nicsb:01b}"        # NICsb: 1 bit
    msg_bits += f"{alt_enc:012b}"     # ALT: 12 bits
    msg_bits += f"{t:01b}"            # T: 1 bit
    msg_bits += f"{f:01b}"            # F: 1 bit
    msg_bits += f"{lat_cpr:017b}"     # LAT-CPR: 17 bits
    msg_bits += f"{lon_cpr:017b}"     # LON-CPR: 17 bits

    # Calculate CRC (24 bits)
    crc = mode_s_crc(msg_bits)

    # Append CRC
    full_msg_bits = msg_bits + crc

    # Convert to hex string (112 bits = 28 hex chars)
    msg_int = int(full_msg_bits, 2)
    return f"{msg_int:028X}"




SITE = (37.29321, -121.95584)

def df17_velocity(icao_hex: str, ew_kt=-150, ns_kt=120, vr_fpm=-640) -> str:
    me = '10011' + '001' + '0' + '0' + '000'          # TC19, subtype 1, intent 0, IFR 0, NUCv 0
    me += ('1' if ew_kt < 0 else '0') + f'{abs(ew_kt) + 1:010b}'
    me += ('1' if ns_kt < 0 else '0') + f'{abs(ns_kt) + 1:010b}'
    me += '0' + ('1' if vr_fpm < 0 else '0') + f'{abs(vr_fpm) // 64 + 1:09b}'
    me += '00' + '0' + '0000000'
    bits = f'{17:05b}{5:03b}{int(icao_hex, 16):024b}' + me
    return f'{int(bits + mode_s_crc(bits), 2):028X}'
