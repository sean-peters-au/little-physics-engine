#ifndef DRAWER_H
#define DRAWER_H

#include "simulator.h"

#include <SDL2/SDL.h>

class Drawer {
	public:
	ParticleSimulator simulator;

	Drawer(ParticleSimulator& simulator);

	void draw(SDL_Renderer* renderer);
};

#endif
