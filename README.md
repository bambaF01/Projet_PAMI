# PAMI_PID

Contrôle PID de vitesse pour **2 moteurs DC avec 2 encodeurs** (Arduino Nano ATmega328P). Le sketch mesure les impulsions, calcule la vitesse (RPM) et ajuste le PWM via un PID (avec gain scheduling) pour suivre une consigne.

## Contenu du dépôt

- `PAMI_PID.ino` : sketch Arduino
- `videos_PAMI/` : vidéos de démonstration

## Matériel visé (d’après le code)

- Arduino Nano (ATmega328P)
- 2 moteurs DC + 2 encodeurs quadrature
- Driver moteur type H-bridge (2 canaux)

## Câblage (pins Arduino)

### Encodeurs

- Gauche A → `D2` (INT0)
- Gauche B → `D7`
- Droit A → `D3` (INT1)
- Droit B → `D8`

> Les entrées A doivent être sur `D2` / `D3` (interruptions). Les B peuvent être sur n’importe quelle pin digitale.

### Driver moteurs

- Gauche PWM → `D9`
- Gauche DIR → `D10`, `D6`
- Droit PWM → `D5`
- Droit DIR → `D11`, `D4`

## Utilisation

1. Ouvrir `PAMI_PID.ino` dans l’IDE Arduino.
2. Sélectionner la carte **Arduino Nano** et le bon port série.
3. Si le téléversement échoue, essayer **Tools > Processor > ATmega328P (Old Bootloader)**.
4. Téléverser.
5. Ouvrir le **Moniteur Série** à `9600` bauds.

### Commandes série

- `f` : avance
- `r` : recule
- `s` : stop
- `p` : toggle PID on/off
- `vXXX` : consigne RPM (pour les deux moteurs)
- `gXXX` : PWM manuel moteur gauche (PID off)
- `dXXX` : PWM manuel moteur droit (PID off)
- `+` / `-` : augmente / diminue `Kp` (selon cible)
- `i` / `o` : augmente / diminue `Ki` (selon cible)
- `kp++++` / `kp----` : ajuste `Kp` par pas de 0.1
- `ki+++` / `ki---` : ajuste `Ki` par pas de 0.05
- `L` : réglage gains moteur gauche uniquement
- `R` : réglage gains moteur droit uniquement
- `B` : réglage gains les deux moteurs (par défaut)
- `m` : mesure PPR (démarre puis arrête une mesure d'un tour)

### Sortie série (pour Serial Plotter)

Le sketch envoie :

- `rpmG`, `rpmD` (vitesse mesurée, rpmD en valeur absolue)
- `consG`, `consD` (consignes)
- `dtMs` (période de mesure)

## Réglages importants (dans le code)

- `PPR_MOTEUR` : impulsions par tour (ici `11`)
- `MAX_RPM` : limite de consigne (`60`)
- `FIXED_RPM` : consigne initiale (`60`)
- `periodeMesure` : période d'échantillonnage PID (`100ms`)
- Gains PID par plage : `Kp_low/mid/high`, `Ki_low/mid/high`
- Gains séparés pour chaque moteur (gauche/droit)
- Inversions de sens : `sensGlobal`, `sensDroit`
- Trim vitesse gauche : `trimG` (ajustement fin de vitesse)

## Schéma de câblage (ASCII)

```
Encodeur G A  -> D2 (INT0)        Encodeur D A  -> D3 (INT1)
Encodeur G B  -> D7               Encodeur D B  -> D8

Moteur G PWM  -> D9               Moteur D PWM  -> D5
Moteur G DIR  -> D10, D6          Moteur D DIR  -> D11, D4

Alim moteurs  -> H-bridge         Alim logique  -> 5V Arduino
GND commun    -> Arduino + H-bridge + encodeurs
```

## Procédure de tuning PID (rapide)

1. Mettre une consigne basse, ex: `v20`.
2. Mettre `Ki` à une valeur faible, et `Kd` à `0`.
3. Augmenter `Kp` jusqu’à obtenir une réponse rapide mais sans oscillations fortes.
4. Augmenter `Ki` petit à petit pour réduire l’erreur statique.
5. Tester à `v40` puis `v60` et ajuster les gains par plage (`*_low/mid/high`).
6. Si la sortie sature (`0..255`) ou oscille, réduire `Kp` ou `Ki`.

## Notes

- Le PID est **par moteur**, avec un **gain scheduling** selon la consigne (`RPM_LOW`=20, `RPM_MID`=40).
- La commande PWM est contrainte entre `0..255`.
- Les gains Kp et Ki sont séparés pour chaque moteur (gauche/droit) et ajustables en temps réel.
- L'intégrale est limitée (`iClamp`=200) pour éviter la saturation.
- Le trim `trimG` permet d'ajuster finement la vitesse du moteur gauche.
- Pour bien régler, commencer à basse vitesse et ajuster `Kp`, `Ki`.
