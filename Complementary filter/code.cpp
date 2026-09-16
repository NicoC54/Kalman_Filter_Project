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