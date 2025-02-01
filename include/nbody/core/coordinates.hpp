/**
 * @file coordinates.hpp
 * @brief Coordinate system for screen-space to universe-space conversion
 *
 * This system handles:
 * - Screen panning controls
 * - Coordinate space conversion between:
 *   - Screen space (pixels, origin at top-left)
 *   - Universe space (meters, origin at center)
 */

#ifndef COORDINATES_H
#define COORDINATES_H

#include "nbody/math/vector_math.hpp"

/**
 * @brief Manages viewport position and coordinate transformations
 * 
 * Provides camera-like functionality for panning the view and converting
 * between screen coordinates (pixels) and universe coordinates (meters).
 */
class CoordinateSystem
{
  public:
  Position screenPos;  ///< Current screen offset in pixels

  /** @brief Creates coordinate system at screen origin */
  CoordinateSystem();

  /**
   * @brief Creates coordinate system at specified screen position
   * @param screenPos Initial screen offset in pixels
   */
  CoordinateSystem(Position screenPos);

  /** @brief Pans view up by 10 pixels */
  void moveUp();

  /** @brief Pans view down by 10 pixels */
  void moveDown();

  /** @brief Pans view left by 10 pixels */
  void moveLeft();

  /** @brief Pans view right by 10 pixels */
  void moveRight();

  /**
   * @brief Converts screen coordinates to universe coordinates
   * @param pos Position in screen space (pixels)
   * @return Position in universe space (meters)
   */
  Position screenToUniverse(Position pos) const;

  /**
   * @brief Converts universe coordinates to screen coordinates
   * @param pos Position in universe space (meters)
   * @return Position in screen space (pixels)
   */
  Position universeToScreen(Position pos) const;
};

#endif
