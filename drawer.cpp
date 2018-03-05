#include "drawer.h"
#include "particle.h"
#include "simulator.h"
#include "coordinates.h"

#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2_ttf/SDL_ttf.h>

Drawer::Drawer(ParticleSimulator::ParticleSimulator& simulator) : simulator(simulator) {
  Sans = TTF_OpenFont("OpenSans-Regular.ttf", 10);
}

void Drawer::draw(SDL_Renderer* renderer, unsigned int steps) {
  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
  SDL_RenderClear(renderer); 

	SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);

  SDL_Color White = {255, 255, 255};
  char text[BUFSIZ];
  sprintf(text, "%u days", steps);
  SDL_Surface* surfaceMessage = TTF_RenderText_Solid(Sans, text, White);
  SDL_Texture* Message = SDL_CreateTextureFromSurface(renderer, surfaceMessage);
  SDL_Rect Message_rect;
  Message_rect.x = 0;
  Message_rect.y = 0;
  Message_rect.w = 60;
  Message_rect.h = 20;
  SDL_RenderCopy(renderer, Message, NULL, &Message_rect);

	SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
		
	for (Particle* particle : simulator.particles) {
    Position pixel = universeToScreen(particle->pos);
	  if(SDL_RenderDrawPoint(renderer, pixel.x, pixel.y) != 0) {
			std::cout << SDL_GetError() << std::endl;
		}
	}
}

