/**
 * @file event_manager.cpp
 * @brief Implementation of EventManager.
 */
#include "event_manager.hpp"

// Include headers for the singletons EventManager needs to call
#include "sim_manager.hpp"
#include "presentation_manager.hpp"
#include "sim.hpp" // For ECSSimulator

#include <iostream> // For temporary debug output

// --- Singleton Implementation ---
EventManager& EventManager::getInstance() {
    static EventManager instance;
    return instance;
}

EventManager::EventManager() {
    // Initialization if needed
}

// --- Public Methods ---

void EventManager::processEvent(const sf::Event& event) {
    switch (event.type) {
        case sf::Event::MouseButtonPressed:
            if (event.mouseButton.button == sf::Mouse::Left) {
                handleMouseButtonPressed(event.mouseButton.x, event.mouseButton.y);
            }
            break;
        case sf::Event::MouseMoved:
            handleMouseMoved(event.mouseMove.x, event.mouseMove.y);
            break;
        case sf::Event::KeyPressed:
            handleKeyPressed(event.key.code);
            break;
        // Add other event types if needed (e.g., Closed handled by PresentationManager)
        default:
            break;
    }
}

void EventManager::updateLayout(const std::vector<UIButton>& currentButtonLayout) {
    uiLayout = currentButtonLayout;
    // TODO: Maybe recalculate highlights here based on current mouse position?
}

// --- Private Helper Methods ---

void EventManager::handleMouseButtonPressed(int x, int y) {
    const UIButton* clickedButton = getButtonAt(x, y);

    if (clickedButton) {
        // --- Dispatch actions based on ButtonID ---
        switch (clickedButton->id) {
            case ButtonID::PAUSE_PLAY:
                SimManager::getInstance().togglePause();
                break;
            case ButtonID::NEXT_FRAME:
                 // Need to check if paused - maybe SimManager provides this?
                 // Or pass paused state into processEvent?
                 // For now, assume SimManager handles the check internally
                 SimManager::getInstance().stepOnce();
                break;
            case ButtonID::RESET:
                ECSSimulator::getInstance().reset();
                SimManager::getInstance().resetPause(); // Also reset pause state
                break;
            case ButtonID::DEBUG_TOGGLE:
                PresentationManager::getInstance().toggleDebugVisualization();
                break;
            case ButtonID::SPEED_0_25X:
            case ButtonID::SPEED_0_5X:
            case ButtonID::SPEED_1X:
                SimManager::getInstance().setTimeScale(clickedButton->speedMultiplier);
                break;
            case ButtonID::COLOR_DEFAULT:
                PresentationManager::getInstance().setColorScheme(ColorScheme::DEFAULT);
                break;
            case ButtonID::COLOR_SLEEP:
                 PresentationManager::getInstance().setColorScheme(ColorScheme::SLEEP);
                break;
            case ButtonID::COLOR_TEMP:
                 PresentationManager::getInstance().setColorScheme(ColorScheme::TEMPERATURE);
                break;
            case ButtonID::SCENARIO_BUTTON:
                SimManager::getInstance().selectScenario(clickedButton->scenario);
                break;
            case ButtonID::UNKNOWN:
            default:
                 std::cerr << "Warning: Clicked button with unknown or unhandled ID." << std::endl;
                break;
        }
    }
}

void EventManager::handleMouseMoved(int x, int y) {
    const UIButton* hoveredButton = getButtonAt(x, y);
    ButtonID newHighlight = hoveredButton ? hoveredButton->id : ButtonID::UNKNOWN;

    if (newHighlight != highlightedButton) {
        highlightedButton = newHighlight;
    }
}

void EventManager::handleKeyPressed(sf::Keyboard::Key key) {
    // Handle global keyboard shortcuts related to UI actions if desired
     switch (key) {
        // Example: Scenario switching (Could be moved from SimManager)
         case sf::Keyboard::Num1: SimManager::getInstance().selectScenario(SimulatorConstants::SimulationType::KEPLERIAN_DISK); break;
         case sf::Keyboard::Num2: SimManager::getInstance().selectScenario(SimulatorConstants::SimulationType::RANDOM_POLYGONS); break;
         // ... add other number keys for scenarios ...

        // Example: Toggle Pause (Could be moved from SimManager)
         case sf::Keyboard::P: SimManager::getInstance().togglePause(); break;

         // Example: Reset (Could be moved from SimManager)
         case sf::Keyboard::R: ECSSimulator::getInstance().reset(); SimManager::getInstance().resetPause(); break;

         default:
             break; // Ignore other keys
     }
}

// Helper for hit testing
const UIButton* EventManager::getButtonAt(int x, int y) {
    for (const auto& button : uiLayout) {
        if (button.rect.contains(x, y)) {
            return &button;
        }
    }
    return nullptr; // No button found at coordinates
} 