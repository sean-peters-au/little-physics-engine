#ifndef THERMODYNAMICS_SYSTEM_H
#define THERMODYNAMICS_SYSTEM_H

#include <entt/entt.hpp>
#include "components.h"
#include "simulator_constants.h"

namespace Systems {
    class ThermodynamicsSystem {
    public:
        static void update(entt::registry& registry) {
            auto view = registry.view<
                Components::Temperature, 
                Components::Density,
                Components::Volume,
                Components::Mass
            >();
            
            for (auto entity : view) {
                auto& temp = view.get<Components::Temperature>(entity);
                auto& density = view.get<Components::Density>(entity);
                const auto& volume = view.get<Components::Volume>(entity);
                const auto& mass = view.get<Components::Mass>(entity);
                
                // PV = nRT -> T = PV/nR
                // where n = mass (in kg) * 1000 (to get moles)
                double moles = mass.value * 1e3;
                temp.value = density.value * volume.value / 
                    (moles * SimulatorConstants::GasConst);
            }
        }
    };
}

#endif 