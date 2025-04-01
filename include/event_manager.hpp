/**
 * @file event_manager.hpp
 * @brief Handles UI interaction logic and dispatches actions.
 */
#pragma once

#include <vector>
#include <SFML/Window/Event.hpp>

#include "renderer_types.hpp" // For UIButton

// Forward declaration if needed
// class PresentationManager;
// class SimManager;
// class ECSSimulator;

class EventManager {
public:
    // Singleton access
    static EventManager& getInstance();

    // Delete copy/move constructors and assignment operators
    EventManager(const EventManager&) = delete;
    EventManager& operator=(const EventManager&) = delete;
    EventManager(EventManager&&) = delete;
    EventManager& operator=(EventManager&&) = delete;

    /**
     * @brief Processes a single SFML event for UI interactions.
     * @param event The SFML event to process.
     */
    void processEvent(const sf::Event& event);

    /**
     * @brief Updates the internal layout data used for hit detection.
     * @param currentButtonLayout A vector containing the latest button layouts.
     */
    void updateLayout(const std::vector<UIButton>& currentButtonLayout);

    // --- Make MouseMoved public --- 
    void handleMouseMoved(int x, int y);

    // Add getter for highlight state (needed by PresentationManager::renderUI)
    ButtonID getHighlightedButtonID() const { return highlightedButton; }

private:
    // Private constructor/destructor for singleton
    EventManager();
    ~EventManager() = default;

    // Store the current UI layout for hit detection
    std::vector<UIButton> uiLayout;

    // State needed for interaction logic (e.g., highlights)
    ButtonID highlightedButton = ButtonID::UNKNOWN;
    // Add other states if needed (e.g., highlightedScenario)

    // Helper methods for handling specific event types
    void handleMouseButtonPressed(int x, int y);
    void handleKeyPressed(sf::Keyboard::Key key);

    // Helper for hit testing
    const UIButton* getButtonAt(int x, int y);
}; 