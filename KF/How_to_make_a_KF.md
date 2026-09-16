Voici votre fiche de révision définitive, complète, sans aucune omission par rapport à votre base de travail, avec la mise à jour des notations ($Q_c = Q$ et $q_c = \Gamma$, avec $\mathbb{E}[w(t)w(t)^T] = \Gamma$), et l'ajout de la **méthode détaillée pour calculer les Jacobiennes de $H$ et de $F$ avec leurs exemples d'entrées et de sorties**.

---

# Méthodologie Complète : Poser, Régler et Coder un Filtre de Kalman

---

## PARTIE I : Architecture et Définitions

### 1. Architecture du Filtre : Modèle 2D vs 3D

Avant toute équation, on doit choisir quelles variables inclure dans le vecteur d'état. Faut-il s'arrêter à la vitesse (2D) ou inclure l'accélération (3D) ?
**Règle d'or :** Le vrai bruit physique continu $w(t)$ s'applique toujours sur l'équation de la grandeur la plus élevée de l'état.

* **Modèle 2D (Vitesse Constante - CV) :** L'état s'arrête à la vitesse. Le bruit s'applique sur $\dot{v}$ (c'est donc une **accélération aléatoire**). Mathématiquement on rajoute le bruit à la vitesse. Idéal pour des chocs brefs, dynamiques pilotées par $u$.
* **Modèle 3D (Accélération Constante - CA) :** L'état va jusqu'à l'accélération. Le bruit s'applique sur $\dot{a}$ (c'est donc un **Jerk / à-coup aléatoire**). Mathématiquement on rajoute le bruit à l'accélération. Idéal pour estimer et compenser des forces inconnues persistantes (frottement, vent constant).

### 2. Définir les variables et leurs dimensions

Poser les trois dimensions fondamentales du système :

* **$n$** : nombre de variables dans l'état.
* **$p$** : nombre d'entrées de commande.
* **$m$** : nombre de mesures capteurs indépendantes.

Définir les vecteurs associés :

* **Vecteur d'état ($x$)** de taille $n \times 1$ : grandeurs physiques à estimer (ex. position $z$, vitesse $\dot{z}$).
* **Vecteur de commande ($u$)** de taille $p \times 1$ : entrées déterministes connues (ex. accélération moteur $a$).
* **Vecteur de mesure ($y$)** de taille $m \times 1$ : valeurs brutes issues des capteurs.

---

> ### 🔄 Variante EKF (Filtre Étendu)
> 
> 
> Le choix du vecteur d'état reste le même. Cependant, les variables peuvent être non linéaires (ex: un angle $\theta$ et une vitesse angulaire $\dot{\theta}$, ou des coordonnées polaires au lieu de cartésiennes).
> * **$h(x)$ (Le Traducteur) :** Ce n'est pas la valeur brute, c'est la "recette mathématique" pour calculer ce que le capteur *devrait* mesurer à partir de l'état (ex: convertir $(p_x, p_y)$ en distance $r$).
> 
> 

---

## PARTIE II : Modélisation Physique et Discrétisation

### 3. Poser la physique continue ($F$, $G$ et bruit $w(t)$)

Formuler les lois physiques (Newton, Lagrange) sous forme d'espace d'état en temps continu en intégrant le vecteur de bruit continu $w(t)$ de dimension $n \times 1$ (avec $\mathbb{E}[w(t)w(t)^T] = \Gamma$) :


$$\dot{x}(t) = F x(t) + G u(t) + w(t)$$

**Exemple cinématique (position $z$, vitesse $\dot{z}$, accélération $u$)**
La perturbation d'accélération parasite continue $w_a(t)$ n'agit directement que sur la dérivée de la vitesse :

1. $\dot{z} = \mathbf{0} \cdot z + \mathbf{1} \cdot \dot{z} + \mathbf{0} \cdot u + \mathbf{0}$
2. $\ddot{z} = \mathbf{0} \cdot z + \mathbf{0} \cdot \dot{z} + \mathbf{1} \cdot u + \mathbf{w_a(t)}$

Par identification directe ligne par ligne :


$$F = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}, \quad G = \begin{bmatrix} 0 \\ 1 \end{bmatrix}, \quad w(t) = \begin{bmatrix} 0 \\ w_a(t) \end{bmatrix}$$

### 4. Discrétiser la dynamique avec le pas $T_s$ ($F_d$ et $G_d$)

L'algorithme de Kalman s'exécute par pas discrets $T_s$ selon :


$$x_{k+1} = F_d x_k + G_d u_k + w_k$$

**Matrice de transition discrète $F_d$ ($n \times n$)**
Traduit l'évolution libre du système par inertie. Calculée par l'exponentielle de matrice ou son approximation de Taylor au premier ordre :


$$F_d = e^{F T_s} \approx I + F T_s = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix} + \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} T_s = \begin{bmatrix} 1 & T_s \\ 0 & 1 \end{bmatrix}$$

**Matrice de commande discrète $G_d$ ($n \times p$)**
Traduit l'action cumulée de la commande $u$ maintenue constante durant l'intervalle $T_s$ (bloqueur d'ordre zéro) :


$$G_d = \int_0^{T_s} e^{F \tau} G \, d\tau \approx G T_s + \frac{1}{2} F G T_s^2 = \begin{bmatrix} \frac{1}{2} T_s^2 \\ T_s \end{bmatrix}$$

> **Méthode alternative directe par identification cinématique :**
> * $z_{k+1} = \mathbf{1} \cdot z_k + \mathbf{T_s} \cdot \dot{z}_k + \mathbf{\frac{1}{2}T_s^2} \cdot u_k$
> * $\dot{z}_{k+1} = \mathbf{0} \cdot z_k + \mathbf{1} \cdot \dot{z}_k + \mathbf{T_s} \cdot u_k$
> 
> 
> L'identification des coefficients devant $(z_k, \dot{z}_k)$ donne directement $F_d$, et ceux devant $u_k$ donnent $G_d$.

### 5. Poser la matrice d'observation ($H$)

Relier les mesures capteurs $y$ aux variables de l'état $x$ selon l'équation linéaire : $y = H x$

* **$H$ ($m \times n$)** :
* Si l'état est $\begin{bmatrix} z \\ \dot{z} \end{bmatrix}$ et que le capteur ne mesure que la position :

$$y = \mathbf{1} \cdot z + \mathbf{0} \cdot \dot{z} \implies H = \begin{bmatrix} 1 & 0 \end{bmatrix}$$


* Si un second capteur mesure la vitesse :

$$H = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}$$





---

> ### 🔄 Variante EKF : Comment calculer une Jacobienne ($H_{jac}$ et $F_{jac}$) ?
> 
> 
> Dans un EKF, on remplace les matrices constantes par des fonctions non linéaires : $x_{k+1} = f(x_k, u_k)$ et $y_k = h(x_k)$.
> Pour propager l'incertitude ($P$) et calculer le gain ($K$), on calcule des **Jacobiennes** (grilles de dérivées partielles) réévaluées à chaque tour autour de l'estimation actuelle $x$.
> **Règle générale de construction :**
> * **Les lignes** = Les variables de sortie (soit les $m$ mesures de $h(x)$, soit les $n$ états de $f(x)$).
> * **Les colonnes** = Les variables d'entrée de l'état ($n$ variables de $x$).
> * *On ne multiplie rien, on dérive case par case !*
> 
> 
> ---
> 
> 
> **A. Exemple de la Jacobienne d'observation ($H_{jac}$) : Entrées $\neq$ Sorties**
> * **Entrées (Colonnes) :** L'état du robot $x = \begin{bmatrix} p_x \\ p_y \end{bmatrix}$ ($n=2$).
> * **Sorties (Lignes) :** Le capteur mesure uniquement la distance $r = \sqrt{p_x^2 + p_y^2}$ ($m=1$).
> * **Taille de la matrice :** $1 \times 2$.
> * **Calcul case par case (dérivées partielles) :**
> * Ligne 1, Col 1 ($\frac{\partial r}{\partial p_x}$) = $\frac{p_x}{\sqrt{p_x^2 + p_y^2}}$
> * Ligne 1, Col 2 ($\frac{\partial r}{\partial p_y}$) = $\frac{p_y}{\sqrt{p_x^2 + p_y^2}}$
> 
> $$\implies H_{jac} = \begin{bmatrix} \frac{p_x}{\sqrt{p_x^2 + p_y^2}} & \frac{p_y}{\sqrt{p_x^2 + p_y^2}} \end{bmatrix}$$
> 
> 
> 
> 
> 
> 
> ---
> 
> 
> **B. Exemple de la Jacobienne de transition ($F_{jac}$) : Entrées = Sorties**
> * **Entrées et Sorties (Colonnes et Lignes) :** C'est le même vecteur d'état $x = \begin{bmatrix} p_x \\ p_y \\ \theta \end{bmatrix}$ ($n=3$). La matrice est donc toujours **carrée ($n \times n$)**.
> * **Fonction non linéaire $f(x)$ :** $\begin{cases} p_{x,\text{suiv}} = p_x + v \cos(\theta) T_s \\ p_{y,\text{suiv}} = p_y + v \sin(\theta) T_s \\ \theta_{\text{suiv}} = \theta + \omega T_s \end{cases}$
> * **Calcul case par case ($3 \times 3$) :**
> * Dérivées de $p_{x,\text{suiv}}$ par rapport à $(p_x, p_y, \theta)$ $\implies [1, \;\; 0, \;\; -v \sin(\theta) T_s]$
> * Dérivées de $p_{y,\text{suiv}}$ par rapport à $(p_x, p_y, \theta)$ $\implies [0, \;\; 1, \;\; +v \cos(\theta) T_s]$
> * Dérivées de $\theta_{\text{suiv}}$ par rapport à $(p_x, p_y, \theta)$ $\implies [0, \;\; 0, \;\; 1]$
> 
> $$\implies F_{jac} = \begin{bmatrix} 1 & 0 & -v \sin(\theta) T_s \\ 0 & 1 & +v \cos(\theta) T_s \\ 0 & 0 & 1 \end{bmatrix}$$
> 
> 
> 
> 
> 
> 

---

## PARTIE III : Bruits, Incertitudes et Réglages (Tuning)

### 6. Distinction fondamentale : Vrai Bruit Physique vs Matrices de Bruit ($Q, R$)

* **La Réalité (Vrai bruit physique) :** Événements aléatoires appliqués lors de la simulation au temps $t$. Le vrai bruit de processus s'applique sur la dérivée la plus haute, le vrai bruit de mesure s'applique sur la lecture brute.
* **Le Filtre (Matrices $Q, R$) :** Paramètres de réglage statiques dictant la méfiance de l'algorithme ($Q$: méfiance envers le modèle physique, $R$: méfiance envers le capteur).

### 7. Quantifier les incertitudes et bruits ($R, Q_d, P_0$)

* **$R$ ($m \times m$) — Bruit de mesure (déjà discret)** : variance issue des spécifications constructeur ($\sigma^2$).
> **Pourquoi ne discrétise-t-on pas $R$ ?**
> La mesure est un événement **instantané et ponctuel** à l'instant $t_k$. Contrairement au bruit de modèle $w(t)$ qui s'accumule en continu sur toute la durée $T_s$, $R$ ne dépend pas de $T_s$ et s'utilise directement telle quelle.


* **$Q_d$ ($n \times n$) — Bruit de processus discrétisé** :
Soit $\Gamma$ la densité spectrale du bruit blanc sur l'accélération (telle que $\mathbb{E}[w(t)w(t)^T] = \Gamma$).
$Q = \begin{bmatrix} 0 & 0 \\ 0 & \Gamma \end{bmatrix}$
**Calcul exact de $Q_d$ (Formule de Van Loan) :**

$$Q_d = \int_0^{T_s} e^{F \tau} Q \, (e^{F \tau})^T \, d\tau$$



Avec $e^{F \tau} = \begin{bmatrix} 1 & \tau \\ 0 & 1 \end{bmatrix}$ :

$$e^{F \tau} Q (e^{F \tau})^T = \begin{bmatrix} 1 & \tau \\ 0 & 1 \end{bmatrix} \begin{bmatrix} 0 & 0 \\ 0 & \Gamma \end{bmatrix} \begin{bmatrix} 1 & 0 \\ \tau & 1 \end{bmatrix} = \Gamma \begin{bmatrix} \tau^2 & \tau \\ \tau & 1 \end{bmatrix}$$



En intégrant terme à terme de $0$ à $T_s$ :

$$Q_d = \Gamma \begin{bmatrix} \frac{1}{3} T_s^3 & \frac{1}{2} T_s^2 \\ \frac{1}{2} T_s^2 & T_s \end{bmatrix}$$


* **$P_0$ ($n \times n$) — Covariance initiale de l'état** :
* État initial connu : valeurs très faibles sur la diagonale.
* État initial inconnu : valeurs très élevées (ex. $10^4 \cdot I$) pour forcer la convergence rapide.



### 8. Le "Tuning" (Trouver le bon ratio $Q/R$)

Le comportement dépend du rapport de force :

* **$Q \gg R$ (Confiance Capteur) :** Très réactif, mais laisse passer le bruit (courbe hachée).
* **$R \gg Q$ (Confiance Modèle) :** Très lisse, mais introduit un **retard (lag)**.
* **Vérification (L'innovation) :** Le résidu $\tilde{y} = y - H \hat{x}$ doit ressembler à un bruit blanc parfait centré sur 0. S'il forme des vagues, le filtre est trop confiant en son modèle.

---

> ### 🔄 Variante EKF (Le Risque de Divergence)
> 
> 
> Contrairement au filtre linéaire qui est inconditionnellement stable, un EKF peut diverger si l'état estimé s'éloigne trop de la réalité (la linéarisation de Taylor devient fausse).
> Lors du tuning EKF, on est souvent obligé de **gonfler artificiellement $Q$** (donner plus de confiance au capteur) pour compenser les erreurs mathématiques liées à la linéarisation imparfaite.

---

## PARTIE IV : Algorithme et Implémentation C++

### 9. Tableau de correspondance et déclarations Eigen

| Symbole | Rôle | Taille | Exemple ($n=2, m=1, p=1$) | Déclaration C++ (Eigen) |
| --- | --- | --- | --- | --- |
| **$x$** | Vecteur d'état | **$n \times 1$** | $2 \times 1$ | `Eigen::Matrix<double, 2, 1>` |
| **$y$** | Mesure capteur | **$m \times 1$** | $1 \times 1$ | `Eigen::Matrix<double, 1, 1>` |
| **$u$** | Commande | **$p \times 1$** | $1 \times 1$ | `Eigen::Matrix<double, 1, 1>` |
| **$F, Q$** | Math continue | **$n \times n$** | $2 \times 2$ | *(Théorique)* |
| **$F_d, Q_d$** | Discrétisation | **$n \times n$** | $2 \times 2$ | `Eigen::Matrix<double, 2, 2>` |
| **$G_d$** | Cmd discrète | **$n \times p$** | $2 \times 1$ | `Eigen::Matrix<double, 2, 1>` |
| **$H$** | Observation | **$m \times n$** | $1 \times 2$ | `Eigen::Matrix<double, 1, 2>` |
| **$R$** | Bruit de mesure | **$m \times m$** | $1 \times 1$ | `Eigen::Matrix<double, 1, 1>` |
| **$P$** | Covariance | **$n \times n$** | $2 \times 2$ | `Eigen::Matrix<double, 2, 2>` |
| **$K$** | Gain de Kalman | **$n \times m$** | $2 \times 1$ | `Eigen::Matrix<double, 2, 1>` |

### 10. Algorithme Mathématique Théorique (Avec suivi des dimensions)

**Phase 1 : Correction (Mise à jour à chaque mesure $y$)**

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



**Phase 2 : Prédiction (Projection sur le pas $T_s$)**

1. **Prédiction de l'état :** $\hat{x} = F_d \hat{x} + G_d u$

$$(n \times n)(n \times 1) + (n \times p)(p \times 1) \implies \mathbf{n \times 1}$$


2. **Prédiction de la covariance :** $P = F_d P F_d^T + Q_d$

$$(n \times n)(n \times n)(n \times n) + (n \times n) \implies \mathbf{n \times n}$$



### 11. Implémentation C++ : Le Filtre Linéaire Classique (KF)

```cpp
for(int i = 0; i < N_iter; ++i) { 
    // --- A. LE MONDE RÉEL (Simulation) ---
    vraie_vitesse = vraie_vitesse - 9.81*T_s + bruit_processus_aleatoire;
    vraie_position = -0.5*9.81*T_s*T_s + vraie_vitesse_precedente*T_s + vraie_position; 
    
    // Le capteur lit la réalité avec une erreur
    y << vraie_position + bruit_capteur_aleatoire;

    // --- B. FILTRE : CORRECTION (Mise à jour pour l'instant T) ---
    K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
    x = x + K * (y - H * x);
    P = (I - K * H) * P;

    // --- C. AFFICHAGE (Synchronisé sur le même instant) ---
    std::cout << "Vrai: " << vraie_position << " | Mesuré: " << y(0) << " | Estimé: " << x(0) << "\n";

    // --- D. FILTRE : PRÉDICTION (Extrapolation pour T+1) ---
    x = Fd * x + Gd * u;
    P = Fd * P * Fd.transpose() + Qd;
}

```

---

> ### 🔄 12. Implémentation C++ : La Variante EKF (Avec Jacobiennes)
> 
> 
> Dans la boucle d'un EKF, **l'état utilise les fonctions non-linéaires pures ($f$ et $h$)**, tandis que **l'incertitude ($P$ et $K$) utilise exclusivement les Jacobiennes ($F_{jac}$ et $H_{jac}$)**.
> ```cpp
> for(int i = 0; i < N_iter; ++i) { 
>     // 1. Simulation et lecture de la vraie mesure y
>     y << vraie_mesure_non_lineaire + bruit_capteur;
> 
>     // 2. FILTRE : CORRECTION (Instant T)
>     Eigen::VectorXd y_pred = h(x);                  // Modèle non linéaire pur
>     Eigen::MatrixXd H_jac = calculer_jacobienne_H(x); // Recalculée autour de l'estimation x
> 
>     K = P * H_jac.transpose() * ((H_jac * P * H_jac.transpose() + R).inverse());
>     x = x + K * (y - y_pred);                         // Utilise h(x), pas H_jac * x !
>     P = (I - K * H_jac) * P;
> 
>     // 3. FILTRE : PRÉDICTION (Instant T+1)
>     x = f(x, u);                                      // Modèle cinématique non linéaire pur
>     Eigen::MatrixXd F_jac = calculer_jacobienne_F(x); // Recalculée autour du NOUVEAU x
> 
>     P = F_jac * P * F_jac.transpose() + Qd;
> }
> 
> ```
> 
>