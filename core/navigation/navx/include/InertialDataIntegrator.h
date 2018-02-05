#ifndef SRC_INERTIALDATAINTEGRATOR_H_
#define SRC_INERTIALDATAINTEGRATOR_H_


namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class InertialDataIntegrator {
                float last_velocity[2];
                float displacement[2];

            public:
                InertialDataIntegrator();
                void updateDisplacement(float accel_x_g, float accel_y_g,
                                         int update_rate_hz, bool is_moving);
                void resetDisplacement();
                float getVelocityX();
                float getVelocityY();
                float getVelocityZ();
                float getDisplacementX();
                float getDisplacementY();
                float getDisplacementZ() ;
            };
        }
    }
}
#endif /* SRC_INERTIALDATAINTEGRATOR_H_ */
