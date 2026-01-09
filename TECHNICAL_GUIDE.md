# Guide Technique OpenVINS - VIO Expliqué

##  Vue d'ensemble

Ce document explique **comment fonctionne OpenVINS** au niveau technique : algorithmes, mathématiques, structure du code et concepts clés.

---

##  Table des matières

1. [Qu'est-ce que le VIO ?](#1-quest-ce-que-le-vio)
2. [Concepts mathématiques fondamentaux](#2-concepts-mathématiques-fondamentaux)
3. [Architecture d'OpenVINS](#3-architecture-dopenvins)
4. [Algorithme MSCKF détaillé](#4-algorithme-msckf-détaillé)
5. [Flux de données et pipeline](#5-flux-de-données-et-pipeline)
6. [Bibliothèques utilisées](#6-bibliothèques-utilisées)
7. [Structure du code](#7-structure-du-code)

---

## 1. Qu'est-ce que le VIO ?

### Définition

**VIO (Visual-Inertial Odometry)** = Estimer la trajectoire 3D d'un robot/drone/véhicule en fusionnant :
- 📷 **Caméra** : Images (position relative des features visuelles)
- ⚡ **IMU** : Accéléromètre + gyroscope (vitesse angulaire + accélération linéaire)

### Pourquoi fusionner caméra + IMU ?

| Capteur | Avantages | Inconvénients | Fréquence |
|---------|-----------|---------------|-----------|
| **Caméra** | Drift faible, position absolue | Lent (20 Hz), échoue si texture faible | 20 Hz |
| **IMU** | Rapide (200+ Hz), toujours actif | Drift rapide (biais), bruit | 200 Hz |
| **VIO (fusion)** | Précis + rapide + robuste | Complexe algorithmiquement | 200 Hz |

### Applications

- 🚁 Drones autonomes 
- 🤖 Robots mobiles
- 🥽 Réalité augmentée
- 🚗 Voitures autonomes (complément GPS)

---

## 2. Concepts mathématiques fondamentaux

### 2.1 Repères de coordonnées

OpenVINS utilise **3 repères** principaux :

```
┌─────────────────────────────────────────┐
│  Repère Global (G)                      │
│  - Fixe dans l'espace                   │
│  - z pointe vers le haut (opposé gravité) │
│  - Origine définie à l'initialisation  │
└─────────────────────────────────────────┘
                  ↓
           Transformation
           R_GtoI, p_IinG
                  ↓
┌─────────────────────────────────────────┐
│  Repère IMU (I)                         │
│  - Solidaire du capteur (bouge)        │
│  - Centre : position de l'IMU          │
│  - Axes : x=avant, y=gauche, z=haut    │
└─────────────────────────────────────────┘
                  ↓
           Transformation
           R_ItoC, p_CinI
                  ↓
┌─────────────────────────────────────────┐
│  Repère Caméra (C)                      │
│  - Solidaire de la caméra              │
│  - z = axe optique                     │
│  - Calibré par rapport à IMU           │
└─────────────────────────────────────────┘
```

### 2.2 Rotations : Quaternions vs Matrices

**Problème** : Représenter l'orientation 3D (3 degrés de liberté : roll, pitch, yaw)

#### Option 1 : Matrices de rotation (3×3)

```
R = [r11  r12  r13]
    [r21  r22  r23]
    [r31  r32  r33]
```

- ✅ Intuitif
- ❌ 9 paramètres (redondant)
- ❌ Contrainte : R^T · R = I (difficile à maintenir)

#### Option 2 : Quaternions (4 paramètres)

```
q = [qx, qy, qz, qw]  avec  ||q|| = 1
```

- ✅ Compact (4 paramètres)
- ✅ Pas de gimbal lock
- ✅ Interpolation facile (SLERP)
- ⚠️ Convention **JPL** dans OpenVINS (pas Hamilton !)

**Convention JPL** : `q_GtoI` signifie "rotation du repère Global vers IMU"

```cpp
// Fichier clé : ov_core/src/utils/quat_ops.h
Eigen::Vector4d quat_multiply(q1, q2);  // Composition de rotations
Eigen::Matrix3d quat_2_Rot(q);          // Quaternion → Matrice rotation
```

### 2.3 Vecteur d'état (State Vector)

L'état estimé par le filtre contient :

```
x = [
    // État IMU (16 DoF)
    q_GtoI      // Orientation (4) - quaternion
    p_IinG      // Position (3) - mètres
    v_IinG      // Vitesse (3) - m/s
    bg          // Biais gyroscope (3) - rad/s
    ba          // Biais accéléromètre (3) - m/s²
    
    // Calibration caméra-IMU (optionnel, 7 DoF par caméra)
    q_ItoC      // Rotation IMU → Caméra (4)
    p_CinI      // Position caméra dans IMU (3)
    
    // Clones (poses passées, sliding window)
    {q_GtoI_i, p_IinG_i}  pour i = 0..N  (N ≈ 11 clones)
    
    // Features SLAM (optionnel)
    {x_f, y_f, z_f}  pour chaque landmark 3D
]
```

**Taille typique** : 16 (IMU) + 7 (calib cam0) + 7 (calib cam1) + 11×7 (clones) = **107 dimensions**

### 2.4 Matrice de covariance

La **covariance** P représente l'**incertitude** de l'estimation :

```
P = E[(x - x̂)(x - x̂)ᵀ]  
```

- Dimension : 107×107 (pour l'état ci-dessus)
- Diagonale : variance de chaque paramètre
- Hors-diagonale : corrélations

```cpp
// Fichier : ov_msckf/src/state/State.h
Eigen::MatrixXd _Cov;  // Matrice de covariance complète
```

**Exemple d'interprétation** :
```
P(0,0) = 0.01   → σ_qx = 0.1 rad  (incertitude sur orientation)
P(4,4) = 0.001  → σ_px = 0.03 m   (incertitude sur position)
```

---

## 3. Architecture d'OpenVINS

### 3.1 Modules principaux

```
┌─────────────────────────────────────────────────────────────┐
│                      VioManager                              │
│  Orchestrateur principal (ov_msckf/src/core/VioManager.cpp) │
└─────────────────────────────────────────────────────────────┘
         │
         ├──> TrackKLT (ov_core/src/track/)
         │    - Détection features (FAST, Shi-Tomasi)
         │    - Suivi optique (Lucas-Kanade)
         │
         ├──> InertialInitializer (ov_init/src/)
         │    - Estime gravité, vitesse, biais IMU
         │    - Utilise ESKF (Error-State Kalman Filter)
         │
         ├──> Propagator (ov_msckf/src/state/)
         │    - Intègre mesures IMU
         │    - Propage état et covariance
         │
         ├──> UpdaterMSCKF (ov_msckf/src/update/)
         │    - Update basé sur features visuelles
         │    - Correction MSCKF classique
         │
         └──> UpdaterSLAM (ov_msckf/src/update/)
              - Update avec landmarks persistants
              - Optimisation bundle adjustment locale
```

### 3.2 Flux de traitement

```
                 SENSORS
                    │
        ┌───────────┴────────────┐
        │                        │
     📷 Images               IMU (200 Hz)
     (20 Hz)                     │
        │                        │
        v                        v
   ┌─────────┐          ┌──────────────┐
   │TrackKLT │          │  Propagator  │
   └─────────┘          └──────────────┘
        │                        │
        │  Features 2D           │  État propagé
        │  (x, y, id)            │  Covariance
        │                        │
        v                        v
   ┌──────────────────────────────────┐
   │   UpdaterMSCKF / UpdaterSLAM     │
   │  - Calcule innovations           │
   │  - Applique correction Kalman    │
   └──────────────────────────────────┘
                    │
                    v
            ┌──────────────┐
            │    State     │
            │ (pose + cov) │
            └──────────────┘
```

---

## 4. Algorithme MSCKF détaillé

**MSCKF** = Multi-State Constraint Kalman Filter

### 4.1 Principe

Au lieu de maintenir les **features 3D** dans l'état (comme SLAM), MSCKF :
1. Garde un **historique de poses** (clones)
2. Triangule les features **temporairement**
3. Utilise les contraintes géométriques pour corriger les poses
4. **Marginalise** les vieux clones

**Avantage** : État de taille fixe (mémoire constante) ✅

### 4.2 Étapes principales

#### Étape 1 : Propagation IMU

Quand une mesure IMU arrive à t+Δt :

```
Modèle IMU :
ω_m = ω + b_g + n_g   (gyroscope mesuré)
a_m = R_GtoI^T · (a - g) + b_a + n_a  (accéléromètre mesuré)

Propagation :
q_{t+Δt} = q_t ⊗ exp((ω_m - b_g) · Δt / 2)  (rotation)
p_{t+Δt} = p_t + v_t · Δt + 0.5 · a_t · Δt²  (position)
v_{t+Δt} = v_t + a_t · Δt                    (vitesse)
```

**Fichier** : `ov_msckf/src/state/Propagator.cpp`

```cpp
void Propagator::propagate_and_clone() {
    // 1. Intégrer IMU avec RK4
    // 2. Propager covariance (Jacobiens)
    // 3. Créer clone si nouvelle image
}
```

#### Étape 2 : Création de clones

À chaque image (20 Hz), on sauvegarde la pose actuelle :

```
Clone_i = {q_GtoI_i, p_IinG_i, timestamp_i}
```

**Sliding window** : On garde les 11 derniers clones (configurable)

#### Étape 3 : Suivi de features

**TrackKLT** détecte et suit des points d'intérêt :

```cpp
// ov_core/src/track/TrackKLT.cpp
void TrackKLT::feed_new_camera(CameraData &message) {
    // 1. Détection FAST corners dans grille 5×5
    // 2. Suivi Lucas-Kanade entre frames
    // 3. Élimination outliers (RANSAC, chi2)
}
```

**Output** : Liste de features avec historique d'observations

```
Feature #42:
  - Clone 0: (u=320, v=240)
  - Clone 1: (u=325, v=238)
  - Clone 2: (u=330, v=236)
  ...
```

#### Étape 4 : Triangulation

Pour chaque feature observée dans ≥2 clones, on triangule sa position 3D :

```
Méthode : SVD (décomposition en valeurs singulières)
Input : Observations 2D + poses caméra
Output : Position 3D du point (x_f, y_f, z_f)
```

**Fichier** : `ov_core/src/feat/FeatureInitializer.cpp`

#### Étape 5 : Update MSCKF

**Calcul de l'innovation** (différence mesure réelle vs prédite) :

```
z_i = [u_i, v_i]  (mesure réelle)
ẑ_i = π(R_CtoG · p_f + p_CinG)  (prédiction)

r_i = z_i - ẑ_i  (résidu)
```

**Jacobien** (sensibilité de la mesure à l'état) :

```
H = ∂h/∂x = [...matrice sparse...]
```

**Kalman Gain** :

```
K = P · H^T · (H · P · H^T + R)^{-1}
```

**Correction de l'état** :

```
x_{new} = x_{old} + K · r
P_{new} = (I - K · H) · P_{old}
```

**Fichier** : `ov_msckf/src/update/UpdaterMSCKF.cpp`

#### Étape 6 : Marginalisation

Quand un clone devient trop vieux, on le **marginalise** :

```
État avant :  [IMU, Clone_0, Clone_1, ..., Clone_11]
Marginalise Clone_0
État après :  [IMU, Clone_1, ..., Clone_11]
```

**Méthode** : Schur complement (garde les corrélations)

```cpp
// ov_msckf/src/state/StateHelper.cpp
void StateHelper::marginalize(State *state, Type *marg_variable);
```

---

## 5. Flux de données et pipeline

### 5.1 Cycle complet

```
t=0.000s : ─────────────────────────────────────
           IMU[0.000] → Propagate
t=0.005s : IMU[0.005] → Propagate
t=0.010s : IMU[0.010] → Propagate
t=0.015s : IMU[0.015] → Propagate
t=0.020s : IMU[0.020] → Propagate
t=0.025s : IMU[0.025] → Propagate
           ...
t=0.050s : IMG[0.050] + IMU[0.050] → Clone + Track + Update
           ├─ Propagate jusqu'à t=0.050
           ├─ Créer Clone_i
           ├─ Détecter/suivre features
           ├─ Trianguler features
           ├─ Calculer update MSCKF
           └─ Corriger état
t=0.055s : IMU[0.055] → Propagate
           ...
t=0.100s : IMG[0.100] + IMU[0.100] → Clone + Track + Update
           ...
```

### 5.2 Ordre d'exécution (code)

```cpp
// Fichier : ov_msckf/src/core/VioManager.cpp

void VioManager::feed_measurement_imu(ImuData &imu) {
    // 1. Buffer IMU
    propagator->feed_imu(imu);
}

void VioManager::feed_measurement_camera(CameraData &cam) {
    // 2. Propager jusqu'au timestamp caméra
    propagator->propagate_and_clone(state, cam.timestamp);
    
    // 3. Détecter features
    trackFEATS->feed_new_camera(cam);
    
    // 4. Check initialisation
    if (!is_initialized) {
        is_initialized = try_to_initialize(cam);
        return;
    }
    
    // 5. Update MSCKF
    do_feature_propagate_update(cam);
}

void VioManager::do_feature_propagate_update(CameraData &cam) {
    // 6. Récupérer features trackées
    auto features = trackFEATS->get_feature_database();
    
    // 7. Update avec MSCKF
    updaterMSCKF->update(state, features);
    
    // 8. Update avec SLAM (optionnel)
    if (updaterSLAM) {
        updaterSLAM->update(state, features);
    }
    
    // 9. Marginaliser vieux clones
    StateHelper::marginalize_old_clone(state);
}
```

---

## 6. Bibliothèques utilisées

### 6.1 Eigen (algèbre linéaire)

**Rôle** : Matrices, vecteurs, décompositions

```cpp
#include <Eigen/Dense>

Eigen::Vector3d position;           // Vecteur 3D
Eigen::Matrix3d rotation;           // Matrice 3×3
Eigen::MatrixXd covariance(107,107); // Matrice dynamique

// Opérations
Eigen::Matrix3d inv = rotation.inverse();
Eigen::VectorXd result = matrix * vector;
```

**Utilisé dans** : Tous les fichiers (état, propagation, update)

### 6.2 Ceres Solver (optimisation non-linéaire)

**Rôle** : Initialisation dynamique, calibration

```cpp
#include <ceres/ceres.h>

// Définir un cost function
class ReprojectionError {
    template <typename T>
    bool operator()(const T* const camera, ...) {
        // Calcul résidu
    }
};

// Optimiser
ceres::Problem problem;
problem.AddResidualBlock(new ReprojectionError(), ...);
ceres::Solve(options, &problem, &summary);
```

**Utilisé dans** : `ov_init/src/dynamic/DynamicInitializer.cpp`

**Note** : Nous avons patché pour Ceres 2.2.0 (Manifold API au lieu de LocalParameterization)

### 6.3 OpenCV (vision par ordinateur)

**Rôle** : Détection features, suivi optique

```cpp
#include <opencv2/opencv.hpp>

// Détection FAST corners
std::vector<cv::KeyPoint> keypoints;
cv::FAST(image, keypoints, threshold);

// Suivi Lucas-Kanade
std::vector<cv::Point2f> next_pts;
cv::calcOpticalFlowPyrLK(prev_img, curr_img, prev_pts, next_pts, ...);
```

**Utilisé dans** : `ov_core/src/track/TrackKLT.cpp`

### 6.4 Boost (utilitaires C++)

**Rôle** : Filesystem, chrono, threads

```cpp
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace fs = boost::filesystem;
if (fs::exists(path)) { ... }

auto t1 = boost::posix_time::microsec_clock::local_time();
```

**Utilisé dans** : Logging, gestion fichiers, timers

---

## 7. Structure du code

### 7.1 Organisation des dossiers

```
open_vins/
├── ov_core/              # Modules génériques (tracking, types)
│   ├── src/track/        # TrackKLT, TrackDescriptor
│   ├── src/feat/         # FeatureInitializer (triangulation)
│   ├── src/types/        # IMU, PoseJPL, Landmark
│   └── src/utils/        # quat_ops, print, colors
│
├── ov_init/              # Initialisation du système
│   ├── src/static/       # Initialisation statique (IMU seul)
│   ├── src/dynamic/      # Initialisation dynamique (Ceres)
│   └── src/ceres/        # State_JPLQuatLocal (Manifold Ceres)
│
└── ov_msckf/             # Filtre MSCKF principal
    ├── src/core/         # VioManager (orchestrateur)
    ├── src/state/        # State, Propagator, StateHelper
    ├── src/update/       # UpdaterMSCKF, UpdaterSLAM, UpdaterZUPT
    └── src/sim/          # Simulator (génère données synthétiques)
```

### 7.2 Fichiers clés à connaître

| Fichier | Rôle | Lignes | Complexité |
|---------|------|--------|------------|
| `VioManager.cpp` | Chef d'orchestre | 700 | ⭐⭐⭐ |
| `State.h` | Définition état | 200 | ⭐⭐ |
| `Propagator.cpp` | Intégration IMU | 600 | ⭐⭐⭐⭐ |
| `UpdaterMSCKF.cpp` | Correction Kalman | 800 | ⭐⭐⭐⭐⭐ |
| `TrackKLT.cpp` | Suivi features | 500 | ⭐⭐⭐ |
| `DynamicInitializer.cpp` | Init avec Ceres | 900 | ⭐⭐⭐⭐ |

### 7.3 Classes principales

```cpp
// 1. Gestion de l'état
class State {
    std::shared_ptr<IMU> _imu;               // Pose IMU actuelle
    std::map<double, PoseJPL*> _clones_IMU;  // Historique poses
    Eigen::MatrixXd _Cov;                    // Covariance
};

// 2. Propagation
class Propagator {
    void propagate_and_clone();  // Intègre IMU + crée clone
    void predict_and_compute();  // Propage covariance
};

// 3. Update
class UpdaterMSCKF {
    void update(State*, FeatureDatabase);  // Correction Kalman
    void get_feature_jacobian_full();      // Calcul Jacobiens
};

// 4. Tracking
class TrackKLT {
    void feed_new_camera(CameraData);  // Détecte + suit features
    void perform_detection_monocular(); // FAST corners
};

// 5. Manager
class VioManager {
    void feed_measurement_imu(ImuData);    // Entrée IMU
    void feed_measurement_camera(CameraData); // Entrée caméra
    State* get_state();                    // Récupère état
};
```

---

## 8. Concepts scientifiques clés

### 8.1 Filtre de Kalman étendu (EKF)

**Problème** : Estimer un état caché x à partir de mesures bruitées z

**Équations** :

```
Prédiction :
  x̂_{k|k-1} = f(x̂_{k-1}, u_k)
  P_{k|k-1} = F_k P_{k-1} F_k^T + Q_k

Correction :
  K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R_k)^{-1}
  x̂_k = x̂_{k|k-1} + K_k (z_k - h(x̂_{k|k-1}))
  P_k = (I - K_k H_k) P_{k|k-1}
```

**Vocabulaire** :
- `F` : Jacobien de la dynamique (comment l'état évolue)
- `H` : Jacobien de la mesure (comment la mesure dépend de l'état)
- `Q` : Bruit de processus (incertitude IMU)
- `R` : Bruit de mesure (incertitude features)
- `K` : Gain de Kalman (pondération prédiction vs mesure)

### 8.2 Modèle pinhole (caméra)

**Projection 3D → 2D** :

```
Point 3D : P = [X, Y, Z]^T (dans repère caméra)
Point 2D : p = [u, v]^T (pixels)

u = f_x · (X/Z) + c_x
v = f_y · (Y/Z) + c_y

Matrice intrinsèque :
K = [f_x   0   c_x]
    [ 0   f_y  c_y]
    [ 0    0    1 ]
```

**Distorsion** (radiale + tangentielle) :

```
Radiale : r' = r · (1 + k1·r² + k2·r⁴)
Tangentielle : [p1, p2] (décentrage lentille)
```

### 8.3 Triangulation (géométrie épipolaire)

**Objectif** : Trouver position 3D d'un point vu dans 2+ images

```
Image 1 : p1 = K · [R1 | t1] · P
Image 2 : p2 = K · [R2 | t2] · P

Méthode : SVD de la matrice A:
A · P = 0  avec A construit depuis p1, p2, R1, R2, t1, t2

Solution : Vecteur singulier de plus petite valeur singulière
```

**Fichier** : `ov_core/src/feat/FeatureInitializer::single_triangulation_1d()`

### 8.4 Marginalisation (Schur complement)

**Objectif** : Enlever une variable de l'état en gardant les corrélations

```
État x = [x_a, x_m]  (a=à garder, m=à marginaliser)
Cov P = [P_aa  P_am]
        [P_ma  P_mm]

P_new = P_aa - P_am · P_mm^{-1} · P_ma
```

**Physiquement** : On projette l'information de x_m sur x_a avant de l'enlever

---

## 9. Notions mathématiques pré-requises

### Niveau minimum requis

| Domaine | Concepts nécessaires |
|---------|---------------------|
| **Algèbre linéaire** | Matrices, vecteurs, inverse, déterminant, SVD, valeurs propres |
| **Probabilités** | Gaussiennes, covariance, loi normale multivariée |
| **Géométrie 3D** | Rotations, translations, transformations homogènes |
| **Calcul différentiel** | Dérivées partielles, Jacobiens, Taylor 1er ordre |
| **Optimisation** | Moindres carrés, gradient, Gauss-Newton |

### Ressources recommandées

📚 **Livres** :
- *State Estimation for Robotics* (Barfoot) - **LE** livre de référence
- *Multiple View Geometry* (Hartley & Zisserman) - Vision 3D
- *Probabilistic Robotics* (Thrun) - Filtres probabilistes

🎓 **Cours en ligne** :
- SLAM Course (Cyrill Stachniss) - YouTube
- Visual Odometry (Davide Scaramuzza) - UZH

📄 **Papers fondateurs** :
- MSCKF original : Mourikis & Roumeliotis (ICRA 2007)
- Concepts théoriques : [docs.openvins.com](https://docs.openvins.com/)  
- Paper original : [Geneva et al., ICRA 2020](https://udel.edu/~ghuang/iros19-vins-workshop/papers/06.pdf)

---

## 10. FAQ Technique

### Q1 : Pourquoi quaternions et pas angles d'Euler ?

**Réponse** : Les angles d'Euler (roll, pitch, yaw) souffrent du **gimbal lock** (singularité quand pitch = ±90°). Les quaternions n'ont pas ce problème et sont plus efficaces pour l'interpolation.

### Q2 : Pourquoi MSCKF et pas bundle adjustment complet ?

**Réponse** : MSCKF = **temps réel** (complexité O(n²) au lieu de O(n³)). On marginalise les vieux clones au lieu de tout ré-optimiser. Bon compromis précision/vitesse.

### Q3 : Quelle est la différence entre MSCKF et SLAM ?

| Critère | MSCKF | SLAM |
|---------|-------|------|
| État | Poses (clones) | Poses + landmarks 3D |
| Features | Marginalisées | Persistantes dans l'état |
| Complexité | O(n²) | O(n³) |
| Mémoire | Fixe (sliding window) | Croissante |
| Précision | Bonne | Excellente |
| Temps réel | ✅ Oui | ⚠️ Difficile |

### Q4 : Pourquoi initialiser avec Ceres ?

**Réponse** : L'initialisation résout un problème **non-linéaire** (estimer gravité, vitesse, biais simultanément). Ceres utilise Gauss-Newton optimisé, plus robuste qu'un EKF simple.

### Q5 : C'est quoi le "sliding window" ?

**Réponse** : On garde seulement les **N dernières poses** (N=11 typiquement). Les vieilles sont marginalisées. Cela limite la mémoire et la complexité tout en gardant les corrélations récentes.

---

## 11. Pour aller plus loin

### Code important à lire dans l'ordre

1. ✅ `examples_integration/euroc_reader_example.cpp` - Votre code (simple)
2. ✅ `ov_msckf/src/core/VioManager.cpp` - Vue d'ensemble
3. ⭐ `ov_core/src/types/IMU.h` - Structure de base
4. ⭐ `ov_msckf/src/state/State.h` - Définition état
5. ⭐⭐ `ov_msckf/src/state/Propagator.cpp` - Intégration IMU
6. ⭐⭐⭐ `ov_core/src/track/TrackKLT.cpp` - Suivi features
7. ⭐⭐⭐⭐ `ov_msckf/src/update/UpdaterMSCKF.cpp` - Cœur de l'algorithme

### Expériences à faire

1. **Modifier les paramètres** dans `estimator_config.yaml` :
   - `max_clones` : 5 vs 15 (impact mémoire/précision)
   - `num_pts` : 50 vs 500 (features trackées)
   - `init_window_time` : 1s vs 5s (temps initialisation)

2. **Activer les logs** :
   - Modifier `verbosity: "DEBUG"` pour voir les détails

3. **Comparer algorithmes** :
   - Désactiver SLAM : `max_slam: 0`
   - Tester update fréquence : `track_frequency`

---

**Dernière mise à jour** : 30 novembre 2025
