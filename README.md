# CoursesParisSaclay

# Cahier des Charges Fonctionnel – Projet Voiture Autonome (F1Tenth / Saclay)

---

# 1. Présentation générale du projet

L’objectif est de développer une **voiture autonome 1/10ᵉ** répondant aux exigences du concours **F1Tenth / Course Paris-Saclay**.

La voiture doit être capable de :

* percevoir son environnement (LiDAR + éventuellement caméra / IMU)
* éviter les obstacles
* suivre une trajectoire
* construire une carte et se localiser
* fonctionner en simulation puis en réel
* rouler **de manière totalement autonome** sur un circuit inconnu avant la course

Le projet est mené par trois étudiants, en équipe structurée.

---

# 2. Besoins Fonctionnels

## 2.1 Fonctionnalités principales

| ID | Fonction                         | Description                                                              |
| -- | -------------------------------- | ------------------------------------------------------------------------ |
| F1 | **Perception environnement**     | Détection LiDAR avec portée ≥ 10 m                                       |
| F2 | **Conduite autonome basique**    | Wall Following / évitement réactif                                       |
| F3 | **Localisation & SLAM**          | HectorSLAM ou slam_toolbox                                               |
| F4 | **Planification**                | Pure Pursuit (min) / Stanley (optionnel)                                 |
| F5 | **Commande Ackermann**           | Direction + vitesse via ESC + servo                                      |
| F6 | **Démarrage / arrêt à distance** | Réception unique d’un signal start/stop conformément au règlement Saclay |
| F7 | **Visualisation**                | RViz2 (LiDAR / map / odom / tf / commandes)                              |
| F8 | **Déploiement embarqué**         | Exécution ROS2 sur Raspberry Pi ou autre SBC                             |

## 2.2 Fonctionnalités secondaires

| ID  | Fonction                | Description                          |
| --- | ----------------------- | ------------------------------------ |
| F9  | Rosbag                  | Enregistrement de données pour debug |
| F10 | Anti-collision avancé   | Safety node prioritaire              |
| F11 | Calibration automatique | Servo / ESC                          |
| F12 | Analyse performance     | Chronométrage + logs                 |

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
* LiDAR 2D 360° (5–15 Hz)
* SLAM temps réel (HectorSLAM conseillé)
* Pure Pursuit pour contrôle latéral
* Poids total ≤ 2.8 kg
* Autonomie ≥ 30 min

## 3.2 Sécurité

* Arrêt d’urgence mécanique + software
* Anti-collision obligatoire
* Vitesse limitée < 2 m/s au début
* Aucun câble apparent
* Gestion thermique Jetson / ESC / servo

## 3.3 Organisationnelles

* Travail en équipe
* Versionning GitHub obligatoire
* Tests d’abord en simulation
* Documentation complète (README, schémas, mapping topics)

---

## 4. Bilan d’Entrées / Sorties du Système

Ce bilan résume **les informations reçues (entrées)** et **les actions produites (sorties)** par la voiture autonome.
Il permet d’avoir une vision claire des interfaces du système ROS2, des capteurs et des commandes.

---

## 4.1 Entrées du système

### **Tableau des Entrées (Inputs)**

| Catégorie             | Entrée             | Description                               | Format / Topic           |
| --------------------- | ------------------ | ----------------------------------------- | ------------------------ |
| **Capteur principal** | LiDAR 2D           | Détection obstacles, SLAM, wall following | `/scan` — LaserScan      |
| **Capteur optionnel** | Caméra             | Aide visuelle / debug                     | `/image_raw`             |
| **Capteur optionnel** | IMU                | Orientation, stabilité                    | `/imu`                   |
| **Signal extérieur**  | Start/Stop         | Démarrage & arrêt réglementaire           | signal simple (GPIO/ROS) |
| **Simulation**        | Données virtuelles | LiDAR sim, odom, map                      | `/scan`, `/odom`, `/map` |

---

## 4.2 Sorties du système

### **Tableau des Sorties (Outputs)**

| Catégorie             | Sortie          | Description           | Format / Topic                   |
| --------------------- | --------------- | --------------------- | -------------------------------- |
| **Commande véhicule** | Ackermann Drive | Direction + vitesse   | `/drive` — AckermannDriveStamped |
| **Navigation**        | Odometry        | Pose estimée          | `/odom` — Odometry               |
| **SLAM**              | Carte occupée   | Map 2D                | `/map` — OccupancyGrid           |
| **TF2**               | Frames          | Transformations robot | `map → odom → base_link → laser` |
| **Logs**              | Rosbag          | Données enregistrées  | fichiers `.db3`                  |

---

# 5. Inventaire Matériel

## 5.1 Matériel déjà disponible (fourni par l’école)

* Voiture F1Tenth 1/10  
* LiDAR (YDLIDAR ou RPLIDAR)  
* Batteries + chargeur  
* Raspberry Pi  
* Plusieurs cartes STM32  
* Caméras  
* Accessoires divers (câbles Dupont, outillage)  

**À confirmer sur place :**  supports imprimés 3D, convertisseurs DC-DC, hub USB.

## 5.2 À commander potentiellement

| Élément                      | Utilité          | Prix     |
| ---------------------------- | ---------------- | -------- |
| Step-down 12V→5V (5A)        | Alimentation SBC | 10–15€   |
| Servo 20kg/cm                | Direction        | 20–30€   |
| ESC Hobbywing 1060           | Propulsion       | 30€      |
| IMU BMI088                   | Stabilisation    | 20–40€   |
| Jetson Orin Nano (optionnel) | Puissance GPU    | 250–400€ |

> Raspberry Pi vs Jetson à décider selon la charge CPU/GPU.
> STM32 obligatoire pour contrôler servo + ESC.

---

# 6. Architecture Logicielle (ROS2)

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

# 7. Critères d’Acceptation

## Simulation

* Wall following stable
* Évitement d’obstacle (< 1 m)
* SLAM cohérent
* Pure Pursuit : trajectoire suivie sans contact

## Réel

* Calibration servo / ESC réussie
* LiDAR stable
* Anti-collision fonctionnel
* SLAM valide dans un environnement réel
* Voiture complète un circuit simple en autonomie

## Sécurité

* Arrêt d’urgence obligatoire
* Vitesse limitée en early-stages
* Aucun câble dangereux

---

# 8. Organisation des Séances

| Date                    | Durée | Objectifs                                                                                            |
| ----------------------- | ----- | ---------------------------------------------------------------------------------------------------- |
| **17/11**               | matin | Présentation du projet + création GitHub + CDC initial + répartition rôles                           |
| **20/11**               | 4h    | Finalisation du CDC + montage complet du châssis TT-02 (roues, transmission, servo, carrosserie) |
| **01/12**               | 8h    | Simulation : Wall Following + Safety Node                                                            |
| **06/12** (à confirmer) | 4h    | Intégration SLAM + tests en simulation                                                               |
| **06/01**               | 4h    | Pure Pursuit + navigation simulée                                                                    |
| **14/01**               | 8h    | Déploiement embarqué + calibration + premiers tests réels                                            |
| **16/01**               | 4h    | Tests réels + tuning + corrections                                                                   |
| **19/01**               | 4h    | Validation finale + répétitions pour la course                                                       |

---

# 9. Organisation du Travail en Équipe

## GitHub

Structure recommandée :

```
/src/        → nodes ROS2 (perception, slam, control)
/launch/     → fichiers launch
/sim/        → Webots ou Gazebo
/real/       → scripts embarqués + config
/docs/       → schémas, CDC, architecture
```

---

# 10. Organigramme du Projet

1. Installation ROS2 & drivers
2. Wall Following
3. Safety Node
4. SLAM
5. Pure Pursuit
6. Intégration complète
7. Déploiement embarqué
8. Tests réels
9. Optimisation course

---

# 11. Conclusion

Ce cahier des charges regroupe :

* les contraintes Saclay
* le matériel déjà disponible
* les fonctionnalités à implémenter
* les exigences ROS2
* un planning réaliste basé sur vos créneaux
* une organisation adaptée à une équipe de trois


