namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            struct QuaternionUpdate
            {
                int16_t q1;
                int16_t q2;
                int16_t q3;
                int16_t q4;
                int16_t accel_x;
                int16_t accel_y;
                int16_t accel_z;
                int16_t mag_x;
                int16_t mag_y;
                int16_t mag_z;
                float   temp_c;
            };
        } // navx
    }
}