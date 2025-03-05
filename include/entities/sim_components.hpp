#pragma once

namespace Components {
    struct SimulatorState {
        double baseTimeAcceleration = 1.0;
        double timeScale = 1.0;
        
        SimulatorState(double bta = 1.0, double ts = 1.0) 
            : baseTimeAcceleration(bta)
            , timeScale(ts) {}
    };
}