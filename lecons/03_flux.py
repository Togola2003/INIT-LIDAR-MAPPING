"""Leçon 3 : retrouver les trames dans un flux continu d'octets.

Le lidar envoie les octets en continu, sans pause entre les trames. L'ESP32 peut commencer
à écouter au milieu d'une trame et les octets arrivent par paquets de taille quelconque.
Ce script reproduit en Python la méthode de la bibliothèque LD06 (fonction readData de ld06.cpp).

Lancer :  .venv/Scripts/python lecons/03_flux.py
"""

TAILLE = 47
TRAME_DATASHEET = bytes.fromhex(
    "54 2C 68 08 AB 7E E0 00 E4 DC 00 E2 D9 00 E5 D5 00 E3 D3 00 E4 D0 00 E9 CD 00 E4 "
    "CA 00 E2 C7 00 E9 C5 00 E5 C2 00 E5 C0 00 E5 BE 82 3A 1A 50"
)


def crc8_table() -> list[int]:
    """Même table que ld06crc.h, construite à partir du polynôme 0x4D."""
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return table


TABLE = crc8_table()


def fabriquer_trame(angle_debut_centiemes: int) -> bytes:
    """Trame de la datasheet avec un autre angle de début, CRC recalculé."""
    t = bytearray(TRAME_DATASHEET)
    t[4], t[5] = angle_debut_centiemes & 0xFF, angle_debut_centiemes >> 8   # little-endian
    fin = (angle_debut_centiemes + 1043) % 36000
    t[42], t[43] = fin & 0xFF, fin >> 8
    crc = 0
    for b in t[:46]:
        crc = TABLE[crc ^ b]
    t[46] = crc
    return bytes(t)


class Lecteur:
    """Machine à états identique à LD06::readData : un index qui avance octet par octet."""

    def __init__(self):
        self.index = 0
        self.crc = 0
        self.tampon = bytearray(TAILLE)

    def recevoir(self, octet: int):
        """Traite UN octet. Renvoie (trame valide ou None, message)."""
        attendu_ok = (self.index > 1
                      or (self.index == 0 and octet == 0x54)
                      or (self.index == 1 and octet == 0x2C))
        if not attendu_ok:
            self.index, self.crc = 0, 0
            return None, f"0x{octet:02X} ignoré (on cherche 54 2C)"
        self.tampon[self.index] = octet
        if self.index < TAILLE - 1:
            self.crc = TABLE[self.crc ^ octet]   # CRC calculé au fil de l'eau
            self.index += 1
            return None, ""
        # 47e octet = le CRC envoyé par le lidar
        ok = self.crc == octet
        self.index, self.crc = 0, 0
        if ok:
            return bytes(self.tampon), "trame complète, CRC OK"
        return None, f"trame complète mais CRC FAUX -> jetée"


# ------------------------------------------------------------------ le flux simulé
a, b, c = fabriquer_trame(32427), fabriquer_trame(33470), fabriquer_trame(34513)
b_abimee = bytearray(b)
b_abimee[20] ^= 0x04                      # un parasite inverse 1 bit dans la trame B
flux = a[-7:] + a + bytes(b_abimee) + c   # on commence à écouter 7 octets avant la fin d'une trame

print(f"Flux reçu : {len(flux)} octets = 7 octets de fin d'une trame + trame A + trame B (abîmée) + trame C\n")

lecteur = Lecteur()
paquets = [5, 30, 13, 47, 40, 15, 200]    # le port série livre les octets par morceaux irréguliers
pos = 0
for taille in paquets:
    morceau = flux[pos:pos + taille]
    if not morceau:
        break
    print(f"-- paquet de {len(morceau)} octets (positions {pos} à {pos + len(morceau) - 1}) --")
    for k, octet in enumerate(morceau):
        trame, msg = lecteur.recevoir(octet)
        if trame:
            angle = (trame[4] | trame[5] << 8) / 100
            print(f"   octet {pos + k:3d} : {msg}  -> angle début {angle:.2f}°")
        elif msg:
            print(f"   octet {pos + k:3d} : {msg}")
    print(f"   (fin du paquet : le lecteur se souvient qu'il en est à l'octet {lecteur.index} d'une trame)")
    pos += len(morceau)

print("\nBilan : A et C sont récupérées, B est rejetée par le CRC, les 7 premiers octets sont ignorés.")
