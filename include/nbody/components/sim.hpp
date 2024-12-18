#ifndef COMPONENTS_SIMULATOR_STATE_HPP
#define COMPONENTS_SIMULATOR_STATE_HPP

namespace Components {
    struct SimulatorState {
        double baseTimeAcceleration;
        double timeScale;
        
        SimulatorState() 
            : baseTimeAcceleration(1.0)
            , timeScale(1.0) {}
    };
}

#endif