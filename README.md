# Roboto — Robot 2 roues (Arduino + Bluetooth) + Mapper PC

Petit projet de robot 2 moteurs DC piloté en **Bluetooth (HC-06)**, avec une **télémétrie** vers un PC Windows qui affiche une **mini carte (occupancy grid)**.

- Firmware Arduino : [roboto/roboto.ino](roboto/roboto.ino)
- Appli Windows (WinForms .NET) : [robot_mapper_pc/RobotMapper](robot_mapper_pc/RobotMapper)

---

## 1) Matériel

- Arduino Nano (ATmega328P)
- 2 moteurs DC (gauche/droite)
- Driver moteur : mini L298N (IN1..IN4)
- Bluetooth série : **HC-06** (SPP)
- Ultrason : **HC-SR04** (distance frontale)
- IMU I2C : MPU-9250 / MPU-6500 (adresse `0x68`) *(optionnel mais utilisé ici pour le yaw)*

---

## 2) Branchement (pins Arduino)

### Bluetooth HC-06 (SoftwareSerial)
Dans [roboto/roboto.ino](roboto/roboto.ino) :

- HC-06 `TXD` → Arduino `D7` (BT_RX)
- HC-06 `RXD` → Arduino `D8` (BT_TX)
- HC-06 `VCC` → `5V`
- HC-06 `GND` → `GND`

> Remarque : l’entrée RX du HC-06 est parfois annoncée “5V tolerant” selon les modules. Si tu veux être safe, mets un petit diviseur de tension sur `RXD` du HC-06.

### L298N (2 moteurs)
Pins par défaut (à garder si ton câblage suit IN1..IN4) :

- IN1 → `D2` (LEFT_IN1)
- IN2 → `D3` (LEFT_IN2)
- IN3 → `D4` (RIGHT_IN3)
- IN4 → `D5` (RIGHT_IN4)

Si un moteur tourne à l’envers, ajuste uniquement :

- `#define LEFT_INVERT  1/0`
- `#define RIGHT_INVERT 1/0`

### HC-SR04 (ultrason)
Dans le code actuel :

- TRIG → `D9`  (`US_TRIG_PIN`)
- ECHO → `D10` (`US_ECHO_PIN`)
- VCC → `5V`
- GND → `GND`

### IMU (MPU-9250/6500) en I2C
- SDA → `A4`
- SCL → `A5`
- VCC/GND selon ton module (souvent **3.3V**)

> Important : beaucoup de cartes IMU ne sont **pas 5V-safe** côté I2C. Si ton module n’a pas de régulateur/level-shifter intégré, utilise 3.3V et/ou un level-shifter.

---

## 3) Commandes Bluetooth

Le robot accepte des commandes ASCII. Certaines sont “1 lettre”, d’autres sont en CSV.

### Manuel (pilotage direct)
- `F` : forward (avance)
- `B` : backward (recule)
- `L` : turn left (pivot sur place)
- `R` : turn right (pivot sur place)
- `S` : stop

### Modes / debug distance
- `A` : toggle **Auto explore** (évite les obstacles avec l’ultrason)
- `M` : toggle **stream télémétrie**
- `P` : ping distance (one-shot)
  - répond `U,<echo_us>` puis `D,<dist_mm>`

### Réglages / objectif
- `V,<mm_per_s>` : règle l’estimation de vitesse linéaire (sans encodeurs) 
- `G,<x_mm>,<y_mm>` : définit une cible (mode **AUTO_GOTO**)

---

## 4) Format des messages envoyés par le robot

Quand le stream est activé (`M`), l’Arduino envoie périodiquement :

- `T,<ms>,<x_mm>,<y_mm>,<yaw_cdeg>,<dist_mm>,<mode>`

Où :
- `yaw_cdeg` = yaw en centi-degrés (ex: `1234` = 12.34°)
- `mode` = `0` MANUAL, `1` AUTO_EXPLORE, `2` AUTO_GOTO

Réponses “ACK” :
- `OK,A,0/1`
- `OK,M,0/1`
- `OK,V,<mm_per_s>`
- `OK,G,<x_mm>,<y_mm>`

---

## 5) Appli Windows (Robot Mapper)

Projet WinForms : [robot_mapper_pc/RobotMapper/RobotMapper.csproj](robot_mapper_pc/RobotMapper/RobotMapper.csproj)

### Lancer
Depuis la racine du workspace :

```powershell
dotnet run --project robot_mapper_pc\RobotMapper\RobotMapper.csproj
```

### Connexion Bluetooth
Sous Windows, le HC-06 en profil **SPP** apparaît comme un **port COM**.

- Appaire le HC-06 dans Windows
- Va dans **Bluetooth > Ports COM**
- Choisis le port **Outgoing** (important)
- Dans l’appli : `Refresh` → `Connect`

### Contrôles
- Boutons : `Stream (M)`, `Auto (A)`, `Ping (P)`
- Clavier :
  - `↑/↓/←/→` : avance / recule / gauche / droite
  - `S` : stop
  - `P` : ping distance

### Carte
- La carte est une grille (cellule = 50 mm)
- Clique sur la carte pour envoyer un objectif `G,x,y`

---

## 6) Dépannage rapide

### Distances à 0 (`dist_mm=0`)
- Utilise `Ping (P)` et regarde la trame `U,<echo_us>` :
  - `echo_us = 0` : pas d’écho → souvent câblage TRIG/ECHO inversé, GND manquant, alim, ou capteur HS
  - `echo_us > 0` mais `dist_mm=0` : cas rare (à investiguer)

### “Accès refusé COMxx”
- Ferme tout ce qui utilise le port (Arduino Serial Monitor, autre appli)
- Vérifie que tu as choisi le COM **Outgoing**

### Yaw instable / dérive
- Le yaw est obtenu par intégration du gyro Z : il peut dériver.
- Au démarrage, laisse le robot **immobile** pendant la calibration.

---

## 7) Limites connues (important)

- Sans encodeurs, `x/y` est une **estimation** basée sur une vitesse supposée.
- Avec un seul ultrason fixe à l’avant, la “map” reste une approximation.

---

## Licence
Projet perso / expérimentation.
