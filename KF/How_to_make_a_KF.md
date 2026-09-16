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

---

### 8. Distinction fondamentale : Vrai Bruit Physique vs Matrices de Bruit ($Q$ et $R$)

Lorsqu'on simule et code un filtre de Kalman, une erreur classique consiste à confondre les perturbations de l'environnement avec les réglages mathématiques de l'algorithme. Il faut impérativement séparer votre programme en **deux mondes hermétiques**.

#### A. Le Monde Réel (La Vérité Terrain et la Simulation)

C'est la physique réelle de votre système. Le vrai monde est chaotique et les capteurs sont imparfaits. Ces bruits sont **des événements aléatoires générés à chaque instant $t$** (ex: via `std::normal_distribution`).

* **Le vrai bruit de processus ($w_k$) :** C'est une perturbation physique réelle (bourrasque de vent, frottement imprévisible, nid-de-poule).
* *Où s'applique-t-il ?* Uniquement sur la cinématique réelle du système, généralement **sur la dérivée la plus élevée** (la vitesse ou l'accélération). La physique (l'intégration) se charge ensuite de faire ruisseler cette erreur sur la position.


* **Le vrai bruit de mesure ($v_k$) :** C'est le tremblement électronique ou l'imprécision physique instantanée du capteur.
* *Où s'applique-t-il ?* Directement sur la grandeur physique mesurée, **avant** d'être envoyée au filtre.
* *Code :* `mesure = vraie_position + erreur_aleatoire`


#### B. Le Cerveau (L'algorithme de Kalman)

Le filtre est un observateur "aveugle". Il reçoit une mesure polluée, mais **il ignore quelle est la part de vérité et la part de bruit**. Les matrices $Q$ et $R$ ne sont pas des nombres aléatoires, mais des **paramètres de réglage statiques** qui dictent au filtre son niveau de méfiance.

* **La matrice $Q$ (Méfiance envers le modèle) :** Elle dit au filtre : *"Attention, les équations physiques parfaites ($F$ et $G$) que j'ai programmées ne reflètent pas la réalité à 100%. Garde toujours une part de doute sur tes prédictions, car il peut y avoir du vent."*
* *Règle :* Si on augmente $Q$, le filtre a moins confiance en sa prédiction mathématique et va chercher à "croire" davantage le capteur.


* **La matrice $R$ (Méfiance envers le capteur) :** Elle dit au filtre : *"Attention, le capteur que tu utilises a une variance de $\sigma^2$. Tiens-en compte pour ne pas réagir excessivement au moindre petit pic de mesure."*
* *Règle :* Si on augmente $R$, le filtre fait moins confiance au capteur et va avoir tendance à lisser massivement la trajectoire en se reposant sur sa prédiction mathématique.


#### C. Le rôle de l'Ingénieur : Le "Tuning"

Toute la magie du filtre de Kalman réside dans le calcul du **Gain de Kalman ($K$)**. L'algorithme calcule $K$ en effectuant en permanence le ratio entre l'incertitude du modèle ($P$, nourrie par $Q$) et l'incertitude du capteur ($R$).
Trouver le bon équilibre entre les valeurs que l'on met dans $Q$ et dans $R$ s'appelle le **tuning** du filtre.

---
### 9. Le "Tuning" d'un Filtre de Kalman (Régler $Q$ et $R$)

Bien que les équations du filtre de Kalman soient mathématiquement parfaites, son comportement réel dépend entièrement du choix des matrices $Q$ et $R$. Ce processus itératif s'appelle le "tuning". L'objectif est de trouver le curseur idéal entre la réactivité (suivre la réalité) et la fluidité (lisser le bruit).

#### A. Fixer $R$ : La réalité du matériel (Le plus facile)

La matrice de covariance du bruit de mesure $R$ ne se devine pas, elle se déduit physiquement des capteurs.

* **Méthode Datasheet :** Le fabricant du capteur fournit souvent une précision ou un écart-type $\sigma$. (Ex: GPS précis à $\pm 3$ mètres $\implies R = 3^2 = 9$).
* **Méthode Expérimentale :** Placez le capteur en position statique, enregistrez 1000 mesures, et calculez mathématiquement la variance de cet échantillon.
* *Règle :* On touche très peu à $R$ une fois qu'elle est fixée, car elle représente une vérité physique mesurable.

#### B. Fixer $Q$ : Modéliser l'inconnu (Le plus difficile)

La matrice $Q$ représente ce que votre modèle cinématique ignore (vent, vibrations, nids-de-poule, action imprévisible d'un pilote). C'est ici que réside le véritable "tuning".

* **L'approche par la variance d'accélération ($q_c$) :** On estime l'accélération parasite maximale que le système peut subir entre deux instants. Par exemple, si une voiture peut au maximum subir un à-coup de $2 \text{ m/s}^2$ à cause d'une bosse, on utilise cette valeur pour construire $q_c$ (la densité spectrale), qui va ensuite remplir $Q_d$ via l'intégrale de Van Loan.
* *Règle :* C'est ce paramètre $q_c$ (ou `gamma` dans votre code) que l'ingénieur va augmenter ou diminuer itérativement lors des essais.

#### C. La Règle d'or : Tout est une question de Ratio ($Q / R$)

Le comportement du filtre ne dépend pas tant des valeurs absolues de $Q$ et $R$, mais du rapport de force entre les deux.

| Scénario | Comportement du Filtre | Conséquence |
| --- | --- | --- |
| **$Q$ grand $\gg R$** | **Fait confiance au Capteur** | Le filtre est très **réactif**. Il capte les vrais mouvements brusques instantanément, mais laisse passer beaucoup de bruit (courbe hachée). |
| **$R$ grand $\gg Q$** | **Fait confiance au Modèle** | Le filtre est très **lisse**. Il efface parfaitement le bruit, mais introduit un **retard (lag)** important. Si l'objet tourne brutalement, l'estimation mettra du temps à rattraper la réalité. |

#### D. Comment vérifier mathématiquement son réglage ? (L'Innovation)

Dans l'industrie, on ne règle pas un filtre uniquement "à l'œil". On analyse mathématiquement **l'innovation** (ou résidu) : $\tilde{y} = y - H \hat{x}$.
L'innovation représente la surprise du filtre à chaque nouvelle mesure.

* **Filtre parfaitement réglé :** La courbe de l'innovation doit ressembler à un bruit blanc parfait. Elle doit être centrée sur zéro, sans aucune tendance, ni cycle, ni vague.
* **Filtre mal réglé :** Si l'innovation est biaisée (reste au-dessus de zéro) ou forme des vagues lors de manœuvres, cela prouve que le filtre est "trop confiant" en son modèle ($Q$ trop faible) et n'arrive pas à suivre la vraie dynamique du système.

---