"""Leçon 1 : décoder UNE trame LD06 à la main, octet par octet.

Lancer :  .venv/Scripts/python lecons/01_trame.py
La trame vient de la datasheet « LD06 Communication Protocol.pdf », page 8.
"""

import sys

# Trame officielle, page 8 du « LD06 Communication Protocol.pdf » (47 octets, CRC correct)
OFFICIELLE_HEX = (
    "54 2C 68 08 AB 7E E0 00 E4 DC 00 E2 D9 00 E5 D5 00 E3 D3 00 E4 D0 00 E9 CD 00 E4 "
    "CA 00 E2 C7 00 E9 C5 00 E5 C2 00 E5 C0 00 E5 BE 82 3A 1A 50"
)
# Trame du document de cours : imprimée avec 40 octets au lieu de 47 (incomplète)
COURS_HEX = (
    "54 2C 60 09 5A 15 "
    "D0 07 25 D0 07 2B D1 07 2F D2 07 30 D1 07 2F D1 07 2E "
    "D0 07 2B D0 07 28 D0 07 25 D1 07 60 15 3E 00 9A"
)


def u16(octets: bytes, pos: int) -> int:
    """Nombre sur 2 octets, LITTLE-ENDIAN : l'octet de poids faible arrive en premier."""
    return octets[pos] | (octets[pos + 1] << 8)


def crc8(octets: bytes) -> int:
    """CRC8 du LD06 (polynôme 0x4D), calculé sur tous les octets sauf le dernier."""
    crc = 0
    for b in octets:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


print(f"Trame du cours : {len(bytes.fromhex(COURS_HEX))} octets -> incomplète (il en faut 47) : on ne peut pas l'utiliser.")
trame = bytes.fromhex(OFFICIELLE_HEX)
print(f"Trame de la datasheet : {len(trame)} octets\n  {trame.hex(' ').upper()}")
if len(trame) != 47:
    sys.exit("longueur incorrecte")

print("\n--- En-tête ---")
print(f"octet 0  = 0x{trame[0]:02X}  -> header (doit valoir 0x54)")
print(f"octet 1  = 0x{trame[1]:02X}  -> version/taille (doit valoir 0x2C)")
vitesse = u16(trame, 2)
print(f"octets 2-3 = {trame[2]:02X} {trame[3]:02X} -> on lit 0x{vitesse:04X} = {vitesse} °/s "
      f"= {vitesse / 360:.2f} tours/s")
debut = u16(trame, 4)
print(f"octets 4-5 = {trame[4]:02X} {trame[5]:02X} -> angle début = {debut} centièmes = {debut / 100:.2f}°")

print("\n--- 12 mesures (3 octets chacune : distance sur 2 octets + intensité sur 1) ---")
fin = u16(trame, 42)
pas = ((fin - debut + 36000) % 36000) / 12
for n in range(12):
    pos = 6 + 3 * n
    dist = u16(trame, pos)
    inten = trame[pos + 2]
    angle_capteur = (debut + (n + 0.5) * pas) / 100   # sens horaire du LD06
    angle_math = (360 - angle_capteur) % 360          # converti en sens trigonométrique
    print(f"  point {n:2d} : octets {pos:2d}-{pos + 2:2d} = {trame[pos]:02X} {trame[pos+1]:02X} {trame[pos+2]:02X}"
          f"  -> {dist} mm, intensité {inten:3d}, angle LD06 {angle_capteur:6.2f}° -> math {angle_math:6.2f}°")

print("\n--- Fin de trame ---")
print(f"octets 42-43 -> angle fin = {fin / 100:.2f}°   (pas entre 2 points = {pas / 100:.4f}°)")
print(f"octets 44-45 -> timestamp = {u16(trame, 44)} ms")
print(f"octet 46     -> CRC reçu = 0x{trame[46]:02X}, CRC recalculé = 0x{crc8(trame[:46]):02X} -> trame valide")

abimee = bytearray(trame)
abimee[10] ^= 0x01  # on inverse UN seul bit d'une distance (parasite sur le fil)
print(f"\nExpérience : on change 1 bit de l'octet 10 -> CRC recalculé = 0x{crc8(bytes(abimee[:46])):02X}"
      f" ≠ 0x{abimee[46]:02X} -> trame rejetée")
