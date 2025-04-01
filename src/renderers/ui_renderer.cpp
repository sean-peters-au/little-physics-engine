/**
 * @file ui_renderer.cpp
 * @brief Implementation of UIRenderer.
 */

#include "renderers/ui_renderer.hpp"

// Constructor
UIRenderer::UIRenderer(const sf::Font& fontRef)
    : font(fontRef) // Store reference
{}

// Draw Button Implementation (Moved from PresentationManager)
void UIRenderer::drawButton(sf::RenderTarget& target, const UIButton& button, sf::Color fillColor, sf::Color textColor)
{
    sf::RectangleShape shape(sf::Vector2f(static_cast<float>(button.rect.width),
                                          static_cast<float>(button.rect.height)));
    shape.setPosition(static_cast<float>(button.rect.left),
                      static_cast<float>(button.rect.top));
    shape.setFillColor(fillColor);
    shape.setOutlineColor(sf::Color::White);
    shape.setOutlineThickness(1.f);
    target.draw(shape);

    if (!button.label.empty()) {
        // Call the UIRenderer's own renderText method
        renderText(target, button.label, button.rect.left + 5, button.rect.top + 3, textColor);
    }
}

// Render Text Implementation (Moved from PresentationManager)
void UIRenderer::renderText(sf::RenderTarget& target, const std::string &text, int x, int y, sf::Color color, unsigned int size)
{
    sf::Text sfText;
    sfText.setFont(font); // Use the member font reference
    sfText.setString(text);
    sfText.setCharacterSize(size);
    sfText.setFillColor(color);
    sfText.setPosition(static_cast<float>(x), static_cast<float>(y));
    target.draw(sfText);
} 