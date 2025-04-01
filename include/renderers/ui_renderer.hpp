/**
 * @file ui_renderer.hpp
 * @brief Handles drawing UI elements like buttons and text.
 */
#pragma once

#include <SFML/Graphics.hpp>
#include <string>
#include <vector>

#include "renderer_types.hpp" // For UIButton definition

class UIRenderer {
public:
    /**
     * @brief Constructor.
     * @param font The font to use for rendering text.
     */
    explicit UIRenderer(const sf::Font& font);

    /**
     * @brief Draws a standard UI button.
     * @param target Where to draw.
     * @param button The button data (rect, label, ID).
     * @param fillColor Background color.
     * @param textColor Text color.
     */
    void drawButton(sf::RenderTarget& target, const UIButton& button, sf::Color fillColor, sf::Color textColor = sf::Color::White);

    /**
     * @brief Renders text directly to the target.
     * @param target Where to draw.
     * @param text The string to render.
     * @param x Screen X coordinate.
     * @param y Screen Y coordinate.
     * @param color Text color.
     * @param size Font size.
     */
    void renderText(sf::RenderTarget& target, const std::string& text, int x, int y, sf::Color color, unsigned int size = 12);

    // We might need a method later for EventManager to get layout data
    // void setLayoutData(const std::vector<UIButton>& buttons); // Option 1: Receive layout
    // const std::vector<UIButton>& getLayoutData() const;       // Option 2: Provide stored layout

private:
    const sf::Font& font; // Reference to the font owned by PresentationManager

    // Optional: Store layout data if needed for getLayoutData()
    // std::vector<UIButton> currentButtonLayout;
}; 