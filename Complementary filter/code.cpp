#include <iostream>
#include <cmath>




int main(){

    double complementary_filter(double ax, double ay, double az, double dt, double Gain, double gyro_vx, double gyro_vy, double gyro_vz){

        while (1){
        
        double gyro_x = atan2(ay,az);
        theta_x = Gain*(theta_x + gyro_vx*dt) + (1-Gain) * gyro_x

        double gyro_y = atan2(-ax,std::hypot(ay,az));
        theta_y = Gain*(theta_y + gyro_vy*dt) + (1-Gain) * gyro_y;

        std::cout << theta_x;
        std::cout << theta_y;
    }




    return 0;
}
}