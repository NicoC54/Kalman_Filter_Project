#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>


int main(){
float dt = 0.1;
Eigen::Matrix2d F;
F << 1, dt, 
     0, 1;

Eigen::Matrix2d I = Eigen::Matrix2d::Identity();

Eigen::Matrix2d Q;
Q << 0.1, 0.0,
     0.0, 1.0;

Eigen::Matrix<double,1,1> R;
R << 300;

Eigen::Matrix2d P;
P << 1e9, 0, 0, 1e9;

Eigen::Matrix<double,1,2> H;
H << 1.0, 0;

Eigen::Vector2d G;
G << 0.5 * dt * dt, 
     dt;

Eigen::Matrix<double, 1, 1> input_vector;
input_vector << 0.0; // u = 0 si aucune accélération commandée


Eigen::Vector2d state_vector;
state_vector << 0, 0;


Eigen::Matrix<double,1,1> mesure_vector;

int cycle = 0;

// En dehors de la boucle (initialisation du générateur)
std::default_random_engine generator;
std::normal_distribution<double> noise(0.0, std::sqrt(300.0)); // sigma = sqrt(R)



while(cycle<10){

    cycle++;

    double position_reelle = 5.0 * cycle;

    mesure_vector << position_reelle + noise(generator);

    Eigen::Matrix<double,1,1> S = H * P * H.transpose() + R;

    Eigen::Vector2d kalman_gain = P* H.transpose() * S.inverse();

    //Mise à jour de la mesure
    Eigen::Matrix<double,1,1> innovation = mesure_vector - (H*state_vector);
    state_vector = state_vector + kalman_gain * innovation(0,0);
    P = (I - kalman_gain*H)*P;

    //extrapolation du pas suivant
    state_vector = F*state_vector + G*input_vector;
    P = F*P*F.transpose() + Q;


    std::cout << "numéro de cycle: " << cycle;

    std::cout << "position_estimée: " <<state_vector[0];
    std::cout << "vitesse_estimée: " <<state_vector[1];




}

return 0;
}