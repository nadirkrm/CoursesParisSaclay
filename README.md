# CoursesParisSaclay

# Cahier des Charges Fonctionnel – Projet Voiture Autonome (F1Tenth / Saclay)

---

# 1. Présentation générale du projet

L’objectif est de développer une **voiture autonome 1/10ᵉ** répondant aux exigences du concours **F1Tenth / Course Paris-Saclay** :

La voiture doit être capable de :

* percevoir son environnement (LiDAR + éventuellement caméra / IMU)
* éviter les obstacles
* suivre une trajectoire
* construire une carte et se localiser
* fonctionner en simulation puis en réel
* rouler **de manière totalement autonome** sur un circuit inconnu avant la course

Le projet doit être mené entre trois étudiants, en équipe structurée.

---

# 2. Besoins Fonctionnels

## 2.1 Fonctionnalités principales

| ID | Fonction                      | Description                                  |
| -- | ----------------------------- | -------------------------------------------- |
| F1 | **Perception environnement**  | Détection LiDAR avec portée ≥ 10 m           |
| F2 | **Conduite autonome basique** | Wall Following / évitement réactif           |
| F3 | **Localisation & SLAM**       | HectorSLAM ou slam_toolbox                   |
| F4 | **Planification**             | Pure Pursuit (min) / Stanley (optionnel)     |
| F5 | **Commande Ackermann**        | Direction + vitesse via ESC + servo          |
| F6 | **Téléopération**             | Joystick ou clavier                          |
| F7 | **Visualisation**             | RViz2 (LiDAR / map / odom / tf / commandes)  |
| F8 | **Déploiement embarqué**      | Exécution ROS2 sur Raspberry Pi ou autre SBC |

## 2.2 Fonctionnalités secondaires

| ID  | Fonction                | Description                       |
| --- | ----------------------- | --------------------------------- |
| F9  | Rosbag                  | Enregistrement données pour debug |
| F10 | Anti-collision avancé   | Safety node prioritaire           |
| F11 | Calibration automatique | Servo / ESC                       |
| F12 | Analyse performance     | Chronométrage + logs              |

---

# 3. Contraintes

## 3.1 Techniques

* OS : **Ubuntu 22.04**
* Framework : **ROS2 Humble**
* Contrôle bas niveau :

  * AckermannDrive
  * Servo PWM 50 Hz
  * ESC 2S/3S
* Latence cible : **≤ 80 ms**
* LiDAR 2D 360° 5–15 Hz
* SLAM temps réel (HectorSLAM conseillé)
* Pure Pursuit pour contrôle latéral
* Poids total ≤ 2.8 kg
* Autonomie ≥ 30 min

## 3.2 Sécurité

* Arrêt d’urgence mécanique + software
* Anti-collision obligatoire
* Vitesse limitée < 2 m/s au début
* Aucun câble apparent / gestion thermique

## 3.3 Organisationnelles

* Travail en équipe
* Versionning GitHub obligatoire
* Tests simulation → réel
* Documentation : README, schémas, architecture ROS2

## 3.4 Budget

**350–550 €** si extension matériel nécessaire.

---

# 4. Inventaire Matériel

## 4.1 Matériel déjà disponible (fourni par l’école)

✔ Voiture F1Tenth 1/10
✔ LiDAR (YDLIDAR ou RPLIDAR, préciser le modèle ensuite)
✔ Batteries + chargeur
✔ Raspberry Pi
✔ Plusieurs cartes STM32
✔ Caméras
✔ Accessoires divers (câbles Dupont, outillage)

 **À confirmer sur place :** alimentation, supports imprimés 3D, convertisseurs DC-DC, hub USB.

## 4.2 À commander potentiellement

| Élément                      | Utilité          | Prix     |
| ---------------------------- | ---------------- | -------- |
| Step-down 12V→5V (5A)        | Alimentation SBC | 10–15€   |
| Servo 20kg/cm                | Direction        | 20–30€   |
| ESC Hobbywing 1060           | Propulsion       | 30€      |
| IMU BMI088                   | Stabilisation    | 20–40€   |
| Jetson Orin Nano (optionnel) | Puissance GPU    | 250–400€ |

> **À décider ensemble** :
> Raspberry Pi vs Jetson vs autre SBC.
> STM32 suffira pour servo/ESC → un microcontrôleur est obligatoire dans tous les cas.

---

# 5. Architecture Logicielle (ROS2)

## Topics indispensables

| Topic  | Type                                 |
| ------ | ------------------------------------ |
| /scan  | sensor_msgs/LaserScan                |
| /drive | ackermann_msgs/AckermannDriveStamped |
| /odom  | nav_msgs/Odometry                    |
| /map   | nav_msgs/OccupancyGrid               |
| /tf    | tf2_msgs                             |

## Packages nécessaires

* ydlidar_ros2_driver / rplidar_ros
* hector_slam ou slam_toolbox
* ackermann_msgs
* nav2 (optionnel)
* f1tenth_simulator ou Webots R2025a

---

# 6. ✔️ Critères d’Acceptation

## Simulation

* Wall following stable
* Évitement obstacle (< 1 m)
* SLAM cohérent
* Pure Pursuit → suit la trajectoire sans contact

## Réel

* Calibration servo/ESC ok
* LiDAR propre et stable
* Anti-collision fonctionnel
* SLAM fonctionnel en environnement réel
* Voiture complète un circuit simple autonome

## Sécurité

* Arrêt d’urgence validé
* Vitesse limitée en early-stages
* Aucun câble dangereux

---

# 7. Organisation des Séances

Voici **la fusion complète** avec votre planning réel de séances.

| Date                     | Durée    | Objectifs                                                                |
| ------------------------ | -------- | ------------------------------------------------------------------------ |
| **17/11** (aujourd’hui)  | matin    | Présentation projet + création du GitHub + CDC final + répartition rôles |
| **20/11**                | 4h matin | Setup ROS2 + drivers LiDAR + test /scan + choix carte embarquée          |
| **01/12**                | 8h       | Simulation : Wall Following + Safety Node                                |
| **06/12** (peut changer) | 4h       | Intégration SLAM + SLAM en simulation                                    |
| **06/01**                | 4h       | Pure Pursuit + test navigation dans simulateur complet                   |
| **14/01**                | 8h       | Déploiement embarqué + calibration servo/ESC + premiers tests réels      |
| **16/01**                | 4h       | Tests réels + tuning + corrections                                       |
| **19/01**                | 4h       | Validation finale + répétitions pour la course                           |

---

# 8. 👥 Organisation du Travail en Équipe

## 8.1 Création d’un GitHub partagé

* Organisation "f1tenth-team-xxx"
* Repository principal :

  * `/src/` → tous les nodes ROS2 (perception / slam / control)
  * `/launch/` → fichiers launch
  * `/docs/` → architecture, schémas, notes
  * `/sim/` → Webots ou Gazebo
  * `/real/` → scripts de déploiement + config SBC

## 8.2 Répartition des rôles (exemple)

* **Nadir** → Perception + SLAM + orchestrateur projet
* **Membre 2** → Contrôle (Ackermann, Pure Pursuit, Safety Node)
* **Membre 3** → Embarqué (SBC, STM32, câblage, calibration)

> La répartition peut changer selon vos préférences.

---

# 9. Organigramme du Projet

1. Installation ROS2 & drivers
2. Test LiDAR & Wall Following
3. Safety Node
4. SLAM
5. Pure Pursuit / planification locale
6. Intégration complète en simulation
7. Déploiement embarqué
8. Tests réels
9. Optimisation pour la course

---

# 10. Conclusion

Ce cahier des charges complet regroupe :

* les contraintes F1Tenth / Saclay
* le matériel déjà disponible
* les fonctionnalités à implémenter
* les exigences logicielle ROS2
* un planning calibré sur vos **créneaux imposés de 4h / 8h**
* une organisation réaliste pour une équipe de 3

---
