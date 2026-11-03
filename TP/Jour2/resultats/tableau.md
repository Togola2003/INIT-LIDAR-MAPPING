### Simulation (vérité exacte, 13 scans)
Taux de réussite selon l'erreur de la pose initiale (réussite : < 5 cm et < 2°)

| Méthode | 0.10 m / 5° | 0.25 m / 10° | 0.50 m / 20° | 1.00 m / 30° | Total | Erreur (réussis) | Temps médian |
|---|---|---|---|---|---|---|---|
| grid · exhaustive | 100% | 100% | 99% | 100% | **100%** | 1.9 cm / 0.00° | 81 ms |
| grid · multi-résolution | 100% | 100% | 100% | 100% | **100%** | 1.9 cm / 0.00° | 6 ms |
| grid · gradient | 100% | 100% | 40% | 0% | **60%** | 0.4 cm / 0.04° | 3 ms |
| grid · multi-résolution + gradient | 100% | 100% | 100% | 100% | **100%** | 0.3 cm / 0.04° | 6 ms |
| vecteur · Gauss-Newton | 100% | 100% | 54% | 1% | **64%** | 0.2 cm / 0.04° | 7 ms |
| vecteur · calcul direct | 100% | 74% | 5% | 0% | **45%** | 0.8 cm / 0.35° | 18 ms |
| vecteur · multi-résolution + Gauss-Newton | 100% | 100% | 100% | 100% | **100%** | 0.2 cm / 0.04° | 10 ms |
| scans · ICP point à point | 96% | 94% | 31% | 1% | **55%** | 0.4 cm / 0.12° | 8 ms |
| scans · ICP point à droite | 100% | 100% | 80% | 14% | **73%** | 0.1 cm / 0.02° | 3 ms |
| scans · multi-résolution + ICP | 100% | 100% | 100% | 100% | **100%** | 0.4 cm / 0.09° | 10 ms |

### Scans réels, dans la zone couverte par la carte (critère de l'énoncé)
Taux de réussite selon l'erreur de la pose initiale (réussite : < 5 cm et < 2°)

| Méthode | 0.10 m / 5° | 0.25 m / 10° | 0.50 m / 20° | 1.00 m / 30° | Total | Erreur (réussis) | Temps médian |
|---|---|---|---|---|---|---|---|
| grid · exhaustive | 98% | 100% | 97% | 95% | **98%** | 2.1 cm / 0.00° | 86 ms |
| grid · multi-résolution | 98% | 100% | 98% | 95% | **98%** | 2.1 cm / 0.00° | 6 ms |
| grid · gradient | 100% | 88% | 12% | 0% | **50%** | 0.8 cm / 0.09° | 3 ms |
| grid · multi-résolution + gradient | 100% | 100% | 100% | 95% | **99%** | 0.8 cm / 0.09° | 8 ms |
| vecteur · Gauss-Newton | 100% | 97% | 32% | 3% | **58%** | 1.6 cm / 0.21° | 22 ms |
| vecteur · calcul direct | 93% | 33% | 3% | 0% | **32%** | 2.8 cm / 0.72° | 27 ms |
| vecteur · multi-résolution + Gauss-Newton | 100% | 100% | 100% | 98% | **100%** | 1.4 cm / 0.21° | 20 ms |
| scans · ICP point à point | 98% | 82% | 20% | 0% | **50%** | 0.9 cm / 0.17° | 9 ms |
| scans · ICP point à droite | 100% | 100% | 53% | 10% | **66%** | 0.8 cm / 0.06° | 5 ms |
| scans · multi-résolution + ICP | 92% | 93% | 92% | 97% | **93%** | 1.0 cm / 0.15° | 13 ms |

### Scans réels, tous (y compris hors zone couverte)
Taux de réussite selon l'erreur de la pose initiale (réussite : < 5 cm et < 2°)

| Méthode | 0.10 m / 5° | 0.25 m / 10° | 0.50 m / 20° | 1.00 m / 30° | Total | Erreur (réussis) | Temps médian |
|---|---|---|---|---|---|---|---|
| grid · exhaustive | 80% | 82% | 77% | 78% | **79%** | 2.2 cm / 0.00° | 89 ms |
| grid · multi-résolution | 77% | 78% | 76% | 74% | **76%** | 2.1 cm / 0.00° | 6 ms |
| grid · gradient | 92% | 73% | 10% | 0% | **44%** | 0.9 cm / 0.09° | 3 ms |
| grid · multi-résolution + gradient | 81% | 78% | 77% | 74% | **78%** | 0.9 cm / 0.09° | 8 ms |
| vecteur · Gauss-Newton | 87% | 82% | 26% | 3% | **49%** | 1.6 cm / 0.21° | 22 ms |
| vecteur · calcul direct | 83% | 30% | 3% | 0% | **29%** | 2.7 cm / 0.69° | 28 ms |
| vecteur · multi-résolution + Gauss-Newton | 77% | 77% | 77% | 76% | **77%** | 1.4 cm / 0.21° | 22 ms |
| scans · ICP point à point | 83% | 73% | 18% | 0% | **44%** | 1.0 cm / 0.14° | 10 ms |
| scans · ICP point à droite | 100% | 96% | 46% | 8% | **62%** | 0.9 cm / 0.14° | 5 ms |
| scans · multi-résolution + ICP | 70% | 72% | 70% | 76% | **72%** | 1.0 cm / 0.15° | 13 ms |


Incertitude des références réelles (écart médian de chaque scan aux 12 autres) : médiane 2.4 cm, de 1.3 à 21.8 cm.
