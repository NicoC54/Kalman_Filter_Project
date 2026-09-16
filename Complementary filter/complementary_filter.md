# Fiche de synthèse : Filtre complémentaire et fusion de capteurs (IMU)

---

## 1. Principe fondamental du filtre complémentaire

Le filtre complémentaire fusionne deux sources de données aux propriétés inverses :
* **Gyroscope (vitesse angulaire $\omega$) :** très précis et rapide à court terme, mais dérive dans le temps à cause de l'intégration numérique (bruit basse fréquence / *drift*).
* **Accéléromètre / Magnétomètre (référence absolue) :** bruité à court terme (vibrations mécaniques, accélérations parasites), mais sans dérive à long terme.

### Équation temporelle générale
$$\theta_t = \alpha \cdot (\theta_{t-1} + \omega \cdot dt) + (1 - \alpha) \cdot \theta_{\text{ref}}$$

* **$\alpha$ (gain de filtrage, souvent entre $0{,}95$ et $0{,}98$) :** passe-haut appliqué au gyroscope.
* **$1 - \alpha$ :** passe-bas appliqué à la référence absolue.
* **$dt$ :** temps d'échantillonnage fixe d'un seul cycle (ex. $0{,}01\text{ s}$ pour $100\text{ Hz}$).

---

## 2. Rôle des capteurs et angles associés

| Capteur | Mesure physique | Rôle dans l'estimation | Angle associé |
| :--- | :--- | :--- | :--- |
| **Gyroscope** | Vitesse de rotation ($\text{rad/s}$ ou $^\circ/\text{s}$) | Intégration dynamique rapide sur $X, Y, Z$ | $\Delta\phi, \Delta\theta, \Delta\psi$ |
| **Accéléromètre** | Accélération spécifique ($g$ ou $\text{m/s}^2$) | Référence absolue via le vecteur gravité $\vec{g}$ | Roulis ($\phi$) et Tangage ($\theta$) |
| **Magnétomètre** | Champ magnétique ($\mu\text{T}$ ou Gauss) | Boussole absolue via le Nord magnétique | Lacet / Cap ($\psi$) |
| **Baromètre** | Pression atmosphérique ($\text{hPa}$) | Altitude verticale absolue (1D, aucun angle) | Hauteur $Z$ (en mètres) |

> **Règle clé :** Le baromètre ne sert jamais au calcul d'angles d'attitude. Il s'associe uniquement à l'accéléromètre vertical ($a_z$) pour stabiliser une altitude en mètres.

---

## 3. Conventions aéronautiques (Drones et robotique)

* **Repère sol :** convention **NED** (*North-East-Down*)
  * $X$ : Nord géographique
  * $Y$ : Est
  * $Z$ : Bas (vers le centre de la Terre)
* **Repère drone :** convention **FRD** (*Forward-Right-Down*)
  * $X_b$ : Avant / Nez
  * $Y_b$ : Latéral droit
  * $Z_b$ : Dessous / Plancher
* **Angles de Tait-Bryan :** séquence de rotation intrinsèque $Z - Y' - X''$
  1. Lacet (**Yaw / $\psi$**) autour de $Z$
  2. Tangage (**Pitch / $\theta$**) autour de $Y'$
  3. Roulis (**Roll / $\phi$**) autour de $X''$

---

## 4. Démonstration mathématique (Matrice de rotation)

Soit le vecteur gravité au repos dans le repère terrestre $R_0$ :
$$\vec{g}_0 = \begin{bmatrix} 0 \\ 0 \\ g \end{bmatrix}$$

La matrice de passage globale est $R = R_x(\phi) \cdot R_y(\theta) \cdot R_z(\psi)$.  
La projection mesurée par l'accéléromètre $\vec{a} = R \cdot \vec{g}_0$ donne le système exact :

$$\begin{cases} 
a_x = -g \sin\theta \\ 
a_y = g \sin\phi \cos\theta \\ 
a_z = g \cos\phi \cos\theta 
\end{cases}$$

### A. Roulis ($\phi$)
En divisant la deuxième équation par la troisième :
$$\frac{a_y}{a_z} = \frac{g \sin\phi \cos\theta}{g \cos\phi \cos\theta} = \frac{\sin\phi}{\cos\phi} = \tan\phi \implies \phi = \text{atan2}(a_y, a_z)$$

### B. Tangage ($\theta$)
Pour éliminer $\phi$, on utilise l'identité $\sin^2\phi + \cos^2\phi = 1$ :
$$\sqrt{a_y^2 + a_z^2} = \sqrt{g^2 \cos^2\theta (\sin^2\phi + \cos^2\phi)} = g \cos\theta$$

En divisant $-a_x$ par cette racine :
$$\frac{-a_x}{\sqrt{a_y^2 + a_z^2}} = \frac{g \sin\theta}{g \cos\theta} = \tan\theta \implies \theta = \text{atan2}\left(-a_x, \sqrt{a_y^2 + a_z^2}\right)$$

---

## 5. Formules de calcul des angles absolus

### Formules symétriques recommandées (NXP / Freescale AN3447)
Pour une robustesse accrue lors d'inclinaisons simultanées sur les deux axes :

$$\phi_{\text{acc}} = \text{atan2}\left(a_y, \sqrt{a_x^2 + a_z^2}\right)$$
$$\theta_{\text{acc}} = \text{atan2}\left(-a_x, \sqrt{a_y^2 + a_z^2}\right)$$

### Cap magnétique (Lacet / $\psi$)
* **Cas capteur à plat ($\phi \approx 0, \theta \approx 0$) :**
  $$\psi_{\text{mag}} = \text{atan2}(m_y, m_x)$$
* **Cas capteur incliné (*tilt compensation*) :**
  $$X_h = m_x \cos\theta + m_y \sin\phi \sin\theta + m_z \cos\phi \sin\theta$$
  $$Y_h = m_y \cos\phi - m_z \sin\phi$$
  $$\psi_{\text{mag}} = \text{atan2}(Y_h, X_h)$$

---

## 6. Implémentation C++ complète

```cpp
#include <iostream>
#include <cmath>

// Remplacer ces prototypes par les lectures réelles du bus I2C/SPI
double getAccelx(); double getAccely(); double getAccelz();
double getGyroVx(); double getGyroVy(); double getGyroVz();
double getMagx();   double getMagy();   double getMagz();

int main() {
    // 1. Déclaration de l'état persistant HORS de la boucle
    double roll  = 0.0; // theta_x (rad)
    double pitch = 0.0; // theta_y (rad)
    double yaw   = 0.0; // theta_z (rad)

    // Paramètres du filtre
    const double alpha = 0.98;
    const double dt = 0.01; // Cycle à 100 Hz (10 ms)
    const double RAD_TO_DEG = 180.0 / M_PI;

    while (true) {
        // 2. Récupération des données brutes
        double ax = getAccelx();
        double ay = getAccely();
        double az = getAccelz();

        double gx = getGyroVx(); // rad/s
        double gy = getGyroVy(); // rad/s
        double gz = getGyroVz(); // rad/s

        double mx = getMagx();
        double my = getMagy();
        double mz = getMagz();

        // 3. Calcul des angles de référence (Accéléromètre)
        double roll_acc  = std::atan2(ay, std::hypot(ax, az));
        double pitch_acc = std::atan2(-ax, std::hypot(ay, az));

        // 4. Compensation d'inclinaison pour le magnétomètre (Tilt Compensation)
        double x_h = mx * std::cos(pitch) 
                   + my * std::sin(roll) * std::sin(pitch) 
                   + mz * std::cos(roll) * std::sin(pitch);
        double y_h = my * std::cos(roll) 
                   - mz * std::sin(roll);

        double yaw_mag = std::atan2(y_h, x_h);

        // 5. Mise à jour du filtre complémentaire
        roll  = alpha * (roll  + gx * dt) + (1.0 - alpha) * roll_acc;
        pitch = alpha * (pitch + gy * dt) + (1.0 - alpha) * pitch_acc;
        yaw   = alpha * (yaw   + gz * dt) + (1.0 - alpha) * yaw_mag;

        // 6. Sortie télémétrie en degrés
        std::cout << "Roll: "  << roll  * RAD_TO_DEG << "° | "
                  << "Pitch: " << pitch * RAD_TO_DEG << "° | "
                  << "Yaw: "   << yaw   * RAD_TO_DEG << "°\n";
    }

    return 0;
}
```