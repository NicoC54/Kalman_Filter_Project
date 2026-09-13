# Fiche Mémo : Poser les équations dynamiques continues $\dot{x} = Fx + Gu$

---

### La méthode universelle en 3 étapes

1. **Définir l'état $x$** : regrouper les grandeurs à estimer et leurs dérivées (ex. $x = \begin{bmatrix} z & \dot{z} \end{bmatrix}^T$).
2. **Exprimer la physique** : écrire la loi régissant la dérivée la plus élevée ($\sum F = m\ddot{z}$, $\sum C = J\ddot{\theta}$, circuit $\frac{di}{dt}$, etc.).
3. **Réduire à l'ordre 1** : découper chaque variable en écrivant sa dérivée première, puis identifier les coefficients devant $x$ (matrice $F$) et devant $u$ (matrice $G$).

---

### Exemple 1 : Cinématique pure (véhicule, piéton, drone basique)

On s'intéresse uniquement au mouvement sans modéliser les forces ni la masse. La commande $u(t)$ est directement l'accélération demandée (ou mesurée par un accéléromètre).

* **Loi physique :**
  $$\ddot{z}(t) = u(t)$$

* **Découpage d'ordre 1 ($x = \begin{bmatrix} z \\ \dot{z} \end{bmatrix}$) :**
  * $\dot{z} = \mathbf{0} \cdot z + \mathbf{1} \cdot \dot{z} + \mathbf{0} \cdot u$
  * $\ddot{z} = \mathbf{0} \cdot z + \mathbf{0} \cdot \dot{z} + \mathbf{1} \cdot u$

* **Matrices continues :**
  $$F = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}, \quad G = \begin{bmatrix} 0 \\ 1 \end{bmatrix}$$

---

### Exemple 2 : Dynamique newtonienne avec frottement fluide (mobile avec traînée)

Un mobile de masse $m$ subit une force de poussée motrice $u(t)$ et un frottement de l'air proportionnel à sa vitesse : $F_{\text{frottement}} = -k_v \dot{z}$.

* **Loi physique (PFD : $\sum F = m a$) :**
  $$m \ddot{z}(t) = u(t) - k_v \dot{z}(t) \implies \ddot{z}(t) = -\frac{k_v}{m}\dot{z}(t) + \frac{1}{m}u(t)$$

* **Découpage d'ordre 1 ($x = \begin{bmatrix} z \\ \dot{z} \end{bmatrix}$) :**
  * $\dot{z} = \mathbf{0} \cdot z + \mathbf{1} \cdot \dot{z} + \mathbf{0} \cdot u$
  * $\ddot{z} = \mathbf{0} \cdot z - \mathbf{\frac{k_v}{m}} \cdot \dot{z} + \mathbf{\frac{1}{m}} \cdot u$

* **Matrices continues :**
  $$F = \begin{bmatrix} 0 & 1 \\ 0 & -\frac{k_v}{m} \end{bmatrix}, \quad G = \begin{bmatrix} 0 \\ \frac{1}{m} \end{bmatrix}$$

---

### Exemple 3 : Système oscillant amorti (masse-ressort-amortisseur)

Une masse fixée à un ressort de raideur $k$, avec amortisseur $c$, excitée par une force extérieure $u(t)$.

* **Loi physique :**
  $$m \ddot{z}(t) + c \dot{z}(t) + k z(t) = u(t) \implies \ddot{z}(t) = -\frac{k}{m}z(t) - \frac{c}{m}\dot{z}(t) + \frac{1}{m}u(t)$$

* **Découpage d'ordre 1 ($x = \begin{bmatrix} z \\ \dot{z} \end{bmatrix}$) :**
  * $\dot{z} = \mathbf{0} \cdot z + \mathbf{1} \cdot \dot{z} + \mathbf{0} \cdot u$
  * $\ddot{z} = -\mathbf{\frac{k}{m}} \cdot z - \mathbf{\frac{c}{m}} \cdot \dot{z} + \mathbf{\frac{1}{m}} \cdot u$

* **Matrices continues :**
  $$F = \begin{bmatrix} 0 & 1 \\ -\frac{k}{m} & -\frac{c}{m} \end{bmatrix}, \quad G = \begin{bmatrix} 0 \\ \frac{1}{m} \end{bmatrix}$$

---

### Exemple 4 : Estimation d'angle et biais gyroscope (fusion IMU)

On cherche l'angle $\theta$ via la vitesse angulaire mesurée $\omega_{\text{mes}} = u$. Le gyroscope présente un biais instrumental $b$ qui dérive lentement dans le temps (marche aléatoire $\dot{b} \approx 0$).

* **Lois physiques :**
  * $\dot{\theta}(t) = u(t) - b(t)$
  * $\dot{b}(t) = 0$

* **Découpage d'ordre 1 ($x = \begin{bmatrix} \theta \\ b \end{bmatrix}$) :**
  * $\dot{\theta} = \mathbf{0} \cdot \theta - \mathbf{1} \cdot b + \mathbf{1} \cdot u$
  * $\dot{b} = \mathbf{0} \cdot \theta + \mathbf{0} \cdot b + \mathbf{0} \cdot u$

* **Matrices continues :**
  $$F = \begin{bmatrix} 0 & -1 \\ 0 & 0 \end{bmatrix}, \quad G = \begin{bmatrix} 1 \\ 0 \end{bmatrix}$$