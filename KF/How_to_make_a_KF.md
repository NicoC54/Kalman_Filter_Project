# Méthodologie : Poser un filtre de Kalman (KF) avant de coder

---

### 1. Définir les variables et leurs dimensions

* **Vecteur d'état ($x$)** de taille $n \times 1$ : grandeurs physiques à estimer (ex. position, vitesse, angle).
* **Vecteur de commande ($u$)** de taille $p \times 1$ : entrées extérieures déterministes et connues appliquées au système (ex. poussée, commande moteur, gravité).
* **Vecteur de mesure ($y$)** de taille $m \times 1$ : valeurs brutes issues des capteurs (ex. altitude radar, position GPS).

---

### 2. Poser la physique continue et la discrétiser ($F$ et $G$)

À partir de la dynamique continue $\dot{x}(t) = A x(t) + B u(t)$, discrétiser le système avec le pas de temps $\Delta t$ :

* **Matrice de transition $F$ ($n \times n$)** : projection par inertie de l'état sans commande.
  $$p_k = p_{k-1} + v_{k-1}\Delta t \implies F = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}$$
* **Matrice de commande $G$ ($n \times p$)** : impact de l'accélération $u$ durant $\Delta t$.
  $$G = \begin{bmatrix} \frac{1}{2}\Delta t^2 \\ \Delta t \end{bmatrix}$$

---

### 3. Poser la matrice d'observation ($H$)

Relier les mesures capteurs $y$ aux composantes de l'état $x$ selon l'équation $y = H x$ :

* **Matrice d'observation $H$ ($m \times n$)** :
  * Si l'état est $\begin{bmatrix} \text{position} \\ \text{vitesse} \end{bmatrix}$ et que le capteur ne mesure que la position :
  $$y = 1 \cdot \text{position} + 0 \cdot \text{vitesse} \implies H = \begin{bmatrix} 1 & 0 \end{bmatrix}$$

---

### 4. Quantifier les incertitudes et bruits ($R$, $Q$, $P_0$)

* **$R$ ($m \times m$) — Bruit de mesure** : variance issue des spécifications capteurs ($\sigma_{\text{mesure}}^2$).
* **$Q$ ($n \times n$) — Bruit de processus** : incertitude du modèle physique (rafales, frottements négligés), souvent propagée à partir d'une variance d'accélération $\sigma_a^2$.
* **$P_0$ ($n \times n$) — Covariance initiale** :
  * Point de départ bien connu : valeurs faibles sur la diagonale.
  * Point de départ inconnu : valeurs très élevées (ex. $10^5$ ou $10^9$) pour faire confiance à la première mesure.

---

### 5. Vérifier la cohérence des dimensions

| Étape | Formule | Analyse dimensionnelle | Dimension finale |
| :--- | :--- | :--- | :--- |
| **Gain $K$** | $S = H P H^T + R$ | $(m \times n)(n \times n)(n \times m) + (m \times m)$ | $m \times m$ |
| | $K = P H^T S^{-1}$ | $(n \times n)(n \times m)(m \times m)$ | **$n \times m$** |
| **Correction** | $\hat{x} = \hat{x} + K(y - H\hat{x})$ | $(n \times 1) + (n \times m)[(m \times 1) - (m \times n)(n \times 1)]$ | **$n \times 1$** |
| | $P = (I - KH)P$ | $[(n \times n) - (n \times m)(m \times n)](n \times n)$ | **$n \times n$** |
| **Prédiction** | $\hat{x} = F\hat{x} + Gu$ | $(n \times n)(n \times 1) + (n \times p)(p \times 1)$ | **$n \times 1$** |
| | $P = F P F^T + Q$ | $(n \times n)(n \times n)(n \times n) + (n \times n)$ | **$n \times n$** |