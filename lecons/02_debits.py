"""Leçon 2 : le trajet des données et la question du débit.

    LD06 --(fil TX, UART 230400 bauds, 8N1)--> ESP32 --(câble USB, 115200 bauds)--> PC

Lancer :  .venv/Scripts/python lecons/02_debits.py
Tous les chiffres viennent de la datasheet (« LD06 Communication Protocol.pdf », p. 3 et 4).
"""

POINTS_PAR_S = 4500        # p. 3 : « measure 4500 times each second »
POINTS_PAR_TRAME = 12
OCTETS_PAR_TRAME = 47
BAUDS_LIDAR = 230400       # p. 4 : 230400, 8 bits de données, 1 bit de stop, pas de parité
BAUDS_USB = 115200         # Serial.begin(115200) dans le programme de l'ESP32
BITS_PAR_OCTET_SUR_FIL = 1 + 8 + 1   # bit de start + 8 bits + bit de stop (format « 8N1 »)

# ---------------------------------------------------------------- 1. un octet sur le fil
octet = 0x54  # le premier octet de chaque trame
bits = [(octet >> k) & 1 for k in range(8)]   # l'UART envoie le bit de poids FAIBLE en premier
print("1) Comment 0x54 voyage sur le fil TX du lidar")
print(f"   0x54 = {octet:08b} en binaire")
print(f"   sur le fil : start=0 | {' '.join(map(str, bits))} | stop=1   (10 bits, poids faible d'abord)")
duree_bit_us = 1e6 / BAUDS_LIDAR
print(f"   à {BAUDS_LIDAR} bauds, un bit dure {duree_bit_us:.2f} µs -> un octet {10 * duree_bit_us:.1f} µs")

# ---------------------------------------------------------------- 2. débit du lidar
trames_par_s = POINTS_PAR_S / POINTS_PAR_TRAME
bits_par_s = trames_par_s * OCTETS_PAR_TRAME * BITS_PAR_OCTET_SUR_FIL
print("\n2) Liaison LD06 -> ESP32")
print(f"   {POINTS_PAR_S} points/s ÷ {POINTS_PAR_TRAME} = {trames_par_s:.0f} trames/s"
      f" (une trame toutes les {1000 / trames_par_s:.2f} ms)")
print(f"   {trames_par_s:.0f} × {OCTETS_PAR_TRAME} octets × 10 bits = {bits_par_s:,.0f} bits/s"
      f" -> {100 * bits_par_s / BAUDS_LIDAR:.0f} % de la ligne à {BAUDS_LIDAR} bauds : ça passe")

# ---------------------------------------------------------------- 3. un tour complet
for vitesse in (2152, 3600):  # 2152 °/s = trame d'exemple de la datasheet ; 3600 °/s = 10 tours/s
    tours_par_s = vitesse / 360
    pts_tour = POINTS_PAR_S / tours_par_s
    print(f"\n3) Si le lidar tourne à {vitesse} °/s = {tours_par_s:.1f} tours/s :")
    print(f"   un tour = {1000 / tours_par_s:.0f} ms, ≈ {pts_tour:.0f} points, "
          f"un point tous les {360 / pts_tour:.2f}°")

# ---------------------------------------------------------------- 4. ESP32 -> PC en texte
ligne = "123,54.66,2000,228,1234,-567\n"   # une ligne printScanCSV (N,angle,dist,intensité,x,y)
bits_usb = POINTS_PAR_S * len(ligne) * BITS_PAR_OCTET_SUR_FIL
print("\n4) Liaison ESP32 -> PC (le programme envoie du TEXTE, pas des octets bruts)")
print(f"   un point = {ligne.strip()!r} = {len(ligne)} caractères (au lieu de 3 octets dans la trame)")
print(f"   tout envoyer demanderait {bits_usb:,.0f} bits/s, mais la liaison n'en offre que {BAUDS_USB:,}")
print(f"   -> {bits_usb / BAUDS_USB:.0f} fois trop : le PC ne reçoit donc qu'un scan de temps en temps.")
print("   Robot immobile : ce n'est pas grave, chaque scan reçu est complet (vérifié par le CRC + le nombre de points).")
