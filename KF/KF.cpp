#include <Eigen/Dense>
#include <iostream>
#include <random> // Pour le bruit aléatoire
#include <cmath>  // Pour std::sqrt()

void KalmanFilter(double dt){

Eigen::Matrix<double,2,2> Fd;

Fd << 1,dt,0,1;

Eigen::Matrix<double,2,1> x;
x << 18000, -1800;

Eigen::Matrix<double,2,1> Gd;
Gd << 0.5*dt*dt, dt;

Eigen::Matrix<double,1,1> u;
u << -9.81;

Eigen::Matrix<double,1,2> H;
H << 1,0;

Eigen::Matrix<double,1,1> mesure;
mesure << 0;

Eigen::Matrix<double,2,2> P;
P << 1e4,0,0,1e4;

Eigen::Matrix<double,2,1> Kg;
Kg << 0,0;

double gamma = 1;

Eigen::Matrix<double,2,2> Qd;
Qd << gamma*dt*dt, gamma *dt, gamma * dt, gamma * 1;

Eigen::Matrix<double,1,1> R;
R << 300;

Eigen::Matrix<double,2,2> I = Eigen::Matrix2d::Identity();


//initialisation de x0 et po déja faite plus haut

//calcul du gain de kalman;

std::random_device rd; //seed random
std::mt19937 generator(rd()) // creation du generateur qui prend une seed random rd

double vraie_vitesse = -1750;
double vraie_position = 18000;

std::normal_distribution<double> distribution_bruit(0.0, std::sqrt(300));
std::normal_distribution<double> dist_bruit_vent(0.0, std::sqrt(20));

for(int i = 0; i < 20; ++i) { 

double erreur_capteur = distribution_bruit(generateur);
double rafale_vent = dist_bruit_vent(generateur);

vraie_position = -0.5*9.81*dt*dt + vraie_vitesse*dt + vraie_position;
vraie_vitesse = vraie_vitesse - 9.81*dt + rafale_vent;

mesure << vraie_position + erreur_capteur;

Kg = P*H.transpose()*(( H*P*H.transpose() + R).inverse());

//Mise a jour de la covariance P et du vecteur d'état;

P = (I-Kg*H)*P;
x = x + Kg*(mesure-H*x);


std::cout << "estimation de mesure de position: " << x(0) << " vs estimation de vraie mesure de position: " << vraie_position << std::endl;
std::cout << "estimation de mesure de vitesse" << x(1) << " vs estimation de vraie mesure de vitesse: " << vraie_vitesse << std::endl;

//Extrapolation de P et x;

x = Fd*x + Gd*u;
P = Fd*P*(Fd.transpose()) + Qd;

}
}

int main(){

KalmanFilter(0.5);
return 0;


}
