#include "drawer.h"
#include "particle.h"
#include "simulator.h"
#include "coordinates.h"

#include <iostream>
#include <SDL2/SDL.h>

Drawer::Drawer(ParticleSimulator::ParticleSimulator simulator) : simulator(simulator) {}

void Drawer::draw(SDL_Renderer* renderer) {
	SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);

	for (Particle particle : simulator.particles) {
    Position pixel = universeToScreen(particle.pos);
	  if(SDL_RenderDrawPoint(renderer, pixel.x, pixel.y) != 0) {
			std::cout << SDL_GetError() << std::endl;
		}
	}
}
