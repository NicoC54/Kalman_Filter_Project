# Méthodologie : Poser un filtre de Kalman (KF) avant de coder

---

### 1. Définir les variables et leurs dimensions

Poser les trois dimensions fondamentales du système :
* **$n$** : nombre de variables dans l'état.
* **$p$** : nombre d'entrées de commande.
* **$m$** : nombre de mesures capteurs indépendantes.

Définir les vecteurs associés :
* **Vecteur d'état ($x$)** de taille $n \times 1$ : grandeurs physiques à estimer (ex. position $z$, vitesse $\dot{z}$).
* **Vecteur de commande ($u$)** de taille $p \times 1$ : entrées déterministes connues appliquées au système (ex. accélération moteur $a$).
* **Vecteur de mesure ($y$)** de taille $m \times 1$ : valeurs brutes issues des capteurs (ex. altitude radar).

---

### 2. Poser la physique continue ($F$, $G$ et bruit $w(t)$)

Formuler les lois physiques (Newton, Lagrange) sous forme d'espace d'état en temps continu en intégrant le vecteur de bruit continu $w(t)$ de dimension $n \times 1$ :

$$\dot{x}(t) = F x(t) + G u(t) + w(t)$$

#### Exemple cinématique (position $z$, vitesse $\dot{z}$, accélération $u$)
La perturbation d'accélération parasite continue $w_a(t)$ n'agit directement que sur la dérivée de la vitesse :
1. $\dot{z} = \mathbf{0} \cdot z + \mathbf{1} \cdot \dot{z} + \mathbf{0} \cdot u + \mathbf{0}$
2. $\ddot{z} = \mathbf{0} \cdot z + \mathbf{0} \cdot \dot{z} + \mathbf{1} \cdot u + \mathbf{w_a(t)}$

Par identification directe ligne par ligne :

$$F = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}, \quad G = \begin{bmatrix} 0 \\ 1 \end{bmatrix}, \quad w(t) = \begin{bmatrix} 0 \\ w_a(t) \end{bmatrix}$$

---

### 3. Discrétiser la dynamique avec le pas $T_s$ ($F_d$ et $G_d$)

L'algorithme de Kalman s'exécute par pas discrets $T_s$ selon :

$$x_{k+1} = F_d x_k + G_d u_k + w_k$$

#### Matrice de transition discrète $F_d$ ($n \times n$)
Traduit l'évolution libre du système par inertie. Calculée par l'exponentielle de matrice ou son approximation de Taylor au premier ordre :

$$F_d = e^{F T_s} \approx I + F T_s$$

$$F_d = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix} + \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} T_s = \begin{bmatrix} 1 & T_s \\ 0 & 1 \end{bmatrix}$$

#### Matrice de commande discrète $G_d$ ($n \times p$)
Traduit l'action cumulée de la commande $u$ maintenue constante durant l'intervalle $T_s$ (bloqueur d'ordre zéro) :

$$G_d = \int_0^{T_s} e^{F \tau} G \, d\tau \approx G T_s + \frac{1}{2} F G T_s^2$$

$$G_d = \begin{bmatrix} 0 \\ 1 \end{bmatrix} T_s + \frac{1}{2} \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} \begin{bmatrix} 0 \\ 1 \end{bmatrix} T_s^2 = \begin{bmatrix} \frac{1}{2} T_s^2 \\ T_s \end{bmatrix}$$

> **Méthode alternative directe par identification cinématique :**
> * $z_{k+1} = \mathbf{1} \cdot z_k + \mathbf{T_s} \cdot \dot{z}_k + \mathbf{\frac{1}{2}T_s^2} \cdot u_k$
> * $\dot{z}_{k+1} = \mathbf{0} \cdot z_k + \mathbf{1} \cdot \dot{z}_k + \mathbf{T_s} \cdot u_k$
> 
> L'identification des coefficients devant $(z_k, \dot{z}_k)$ donne directement $F_d$, et ceux devant $u_k$ donnent $G_d$.

---

### 4. Poser la matrice d'observation ($H$)

Relier les mesures capteurs $y$ aux variables de l'état $x$ selon l'équation linéaire :

$$y = H x$$

* **Matrice d'observation $H$ ($m \times n$)** :
  * Si l'état est $\begin{bmatrix} z \\ \dot{z} \end{bmatrix}$ et que le capteur ne mesure que la position ($y = z$) :
    $$y = \mathbf{1} \cdot z + \mathbf{0} \cdot \dot{z} \implies H = \begin{bmatrix} 1 & 0 \end{bmatrix}$$
  * Si un second capteur mesure la vitesse ($y_1 = z$, $y_2 = \dot{z}$) :
    $$H = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}$$

---

### 5. Quantifier les incertitudes et bruits ($R$, $Q_d$, $P_0$)

* **$R$ ($m \times m$) — Bruit de mesure (déjà discret)** : variance issue des spécifications constructeur du capteur ($\sigma_{\text{mesure}}^2$).
  > **Pourquoi ne discrétise-t-on pas $R$ ?**  
  > La mesure est un événement **instantané et ponctuel** à l'instant $t_k$ : le capteur lit une valeur déjà échantillonnée avec son erreur $v_k \sim \mathcal{N}(0, R)$. Contrairement au bruit de modèle $w(t)$ qui s'accumule en continu sur toute la durée $T_s$ (nécessitant l'intégrale de Van Loan pour obtenir $Q_d$), $R$ ne dépend pas de $T_s$ et s'utilise directement telle quelle.

* **$Q_d$ ($n \times n$) — Bruit de processus discrétisé** : déduit de la matrice spectrale continue $Q_c$ via l'intégrale exacte de discrétisation.

#### Construction de $Q_c$ à partir de $w(t)$
Soit $q_c$ la densité spectrale du bruit blanc sur l'accélération, telle que $\mathbb{E}[w_a(t) w_a(\tau)] = q_c \cdot \delta(t - \tau)$ :

$$Q_c = \begin{bmatrix} 0 & 0 \\ 0 & q_c \end{bmatrix}$$

#### Calcul exact de $Q_d$ (Formule de Van Loan)

$$Q_d = \int_0^{T_s} e^{F \tau} Q_c \, (e^{F \tau})^T \, d\tau$$

Avec $e^{F \tau} = \begin{bmatrix} 1 & \tau \\ 0 & 1 \end{bmatrix}$ :

$$e^{F \tau} Q_c (e^{F \tau})^T = \begin{bmatrix} 1 & \tau \\ 0 & 1 \end{bmatrix} \begin{bmatrix} 0 & 0 \\ 0 & q_c \end{bmatrix} \begin{bmatrix} 1 & 0 \\ \tau & 1 \end{bmatrix} = q_c \begin{bmatrix} \tau^2 & \tau \\ \tau & 1 \end{bmatrix}$$

En intégrant terme à terme de $0$ à $T_s$ :

$$Q_d = q_c \begin{bmatrix} \frac{1}{3} T_s^3 & \frac{1}{2} T_s^2 \\ \frac{1}{2} T_s^2 & T_s \end{bmatrix}$$

* **$P_0$ ($n \times n$) — Covariance initiale de l'état** :
  * État initial connu avec certitude : valeurs très faibles sur la diagonale.
  * État initial totalement inconnu : valeurs très élevées (ex. $10^4 \cdot I$) pour forcer le filtre à converger dès la première mesure reçue.

---

### 6. Tableau de correspondance et déclarations Eigen

Règle de dimensionnement : toute matrice ou vecteur est déclaré via `Eigen::Matrix<double, Lignes, Colonnes>`.

| Symbole | Rôle | Taille générale | Exemple ($n=2, m=1, p=1$) | Déclaration C++ (Eigen) |
| :--- | :--- | :--- | :--- | :--- |
| **$x$** | Vecteur d'état | **$n \times 1$** | $2 \times 1$ | `Eigen::Matrix<double, 2, 1>` |
| **$y$** | Mesure capteur | **$m \times 1$** | $1 \times 1$ | `Eigen::Matrix<double, 1, 1>` |
| **$u$** | Commande extérieure | **$p \times 1$** | $1 \times 1$ | `Eigen::Matrix<double, 1, 1>` |
| **$F$** | Matrice d'état continue | **$n \times n$** | $2 \times 2$ | `Eigen::Matrix<double, 2, 2>` |
| **$G$** | Matrice de commande continue | **$n \times p$** | $2 \times 1$ | `Eigen::Matrix<double, 2, 1>` |
| **$w(t)$** | Vecteur de bruit continu | **$n \times 1$** | $2 \times 1$ | *(Formulation théorique)* |
| **$Q_c$** | Densité spectrale continue | **$n \times n$** | $2 \times 2$ | *(Formulation théorique)* |
| **$F_d$** | Matrice d'état discrétisée | **$n \times n$** | $2 \times 2$ | `Eigen::Matrix<double, 2, 2>` |
| **$G_d$** | Matrice de commande discrétisée | **$n \times p$** | $2 \times 1$ | `Eigen::Matrix<double, 2, 1>` |
| **$Q_d$** | Covariance du bruit discrétisée | **$n \times n$** | $2 \times 2$ | `Eigen::Matrix<double, 2, 2>` |
| **$H$** | Observation capteur | **$m \times n$** | $1 \times 2$ | `Eigen::Matrix<double, 1, 2>` |
| **$P$** | Covariance de l'état | **$n \times n$** | $2 \times 2$ | `Eigen::Matrix<double, 2, 2>` |
| **$R$** | Bruit de mesure | **$m \times m$** | $1 \times 1$ | `Eigen::Matrix<double, 1, 1>` |
| **$K$** | Gain de Kalman | **$n \times m$** | $2 \times 1$ | `Eigen::Matrix<double, 2, 1>` |

---

### 7. Algorithme d'exécution (Boucle temps réel)

#### Phase 1 : Correction (Mise à jour à chaque mesure $y$ reçue)
1. **Innovation :** $\tilde{y} = y - H \hat{x}$  
   $$(m \times 1) - (m \times n)(n \times 1) \implies \mathbf{m \times 1}$$
2. **Covariance de l'innovation :** $S = H P H^T + R$  
   $$(m \times n)(n \times n)(n \times m) + (m \times m) \implies \mathbf{m \times m}$$
3. **Gain de Kalman :** $K = P H^T S^{-1}$  
   $$(n \times n)(n \times m)(m \times m) \implies \mathbf{n \times m}$$
4. **Mise à jour de l'état :** $\hat{x} = \hat{x} + K \tilde{y}$  
   $$(n \times 1) + (n \times m)(m \times 1) \implies \mathbf{n \times 1}$$
5. **Mise à jour de la covariance :** $P = (I - KH) P$  
   $$[(n \times n) - (n \times m)(m \times n)](n \times n) \implies \mathbf{n \times n}$$

#### Phase 2 : Prédiction (Projection sur le pas $T_s$)
1. **Prédiction de l'état :** $\hat{x} = F_d \hat{x} + G_d u$  
   $$(n \times n)(n \times 1) + (n \times p)(p \times 1) \implies \mathbf{n \times 1}$$
2. **Prédiction de la covariance :** $P = F_d P F_d^T + Q_d$  
   $$(n \times n)(n \times n)(n \times n) + (n \times n) \implies \mathbf{n \times n}$$