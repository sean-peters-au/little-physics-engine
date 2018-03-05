#ifndef DRAWER_H
#define DRAWER_H

#include "simulator.h"

#include <SDL2/SDL.h>
#include <SDL2_ttf/SDL_ttf.h>

class Drawer {
	public:
	ParticleSimulator simulator;
  TTF_Font* Sans;

	Drawer(ParticleSimulator& simulator);

	void draw(SDL_Renderer* renderer, unsigned int steps);
};

#endif
