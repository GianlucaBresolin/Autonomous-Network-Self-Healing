#pragma once
#include "../../interfaces/velocity_command.h"

class VelocityCommand : public VelocityCommandInterface {
    public: 
        VelocityCommand(
            float vx_value = 0.0,
            float vy_value = 0.0,
            float vz_value = 0.0
        ) : 
            vx(vx_value),
            vy(vy_value),
            vz(vz_value)
        { }
    
    private:
        float vx;
        float vy;
        float vz;
};