#ifndef DRAWER_H
#define DRAWER_H

#include "simulator.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

class Drawer {
	public:
	ParticleSimulator simulator;
  TTF_Font* Sans;
  CoordinateSystem* coordSystem;

	Drawer();
	Drawer(ParticleSimulator& simulator, CoordinateSystem* coordSystem);

	void draw(SDL_Renderer* renderer, unsigned int steps);
};

#endif
