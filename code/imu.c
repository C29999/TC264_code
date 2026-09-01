 #include "imu.h"

float z_angle=0;
float gyro_z=0;
void imu_init(void)
{
    if (imu660rb_init() != 0)
    {
       show_center("IMU660RB Error");
    }
    else 
    {
        show_center("IMU660RB Success");
    }
}
void imu_update(void)
{
    imu660rb_get_gyro();
    gyro_z=imu660rb_gyro_transition(imu660rb_gyro_z);
    z_angle += gyro_z * 0.002f; 
}