#pragma once

namespace Components {
    struct SimulatorState {
        double baseTimeAcceleration;
        double timeScale;
        
        SimulatorState() 
            : baseTimeAcceleration(1.0)
            , timeScale(1.0) {}
    };
}