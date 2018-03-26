#include "drawer.h"
#include "particle.h"
#include "simulator.h"
#include "coordinates.h"

#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2_ttf/SDL_ttf.h>

Drawer::Drawer(ParticleSimulator& simulator, CoordinateSystem* coordSystem) {
  this->simulator = simulator;
  this->coordSystem = coordSystem;
  Sans = TTF_OpenFont("OpenSans-Regular.ttf", 10);
}

void Drawer::draw(SDL_Renderer* renderer, unsigned int steps) {
  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
  SDL_RenderClear(renderer); 

	SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);

  SDL_Color White = {255, 255, 255};
  char text[BUFSIZ];
  sprintf(text, "%u ticks", steps);
  SDL_Surface* surfaceMessage = TTF_RenderText_Solid(Sans, text, White);
  SDL_Texture* Message = SDL_CreateTextureFromSurface(renderer, surfaceMessage);
  SDL_Rect Message_rect;
  Message_rect.x = 0;
  Message_rect.y = 0;
  Message_rect.w = 60;
  Message_rect.h = 20;
  SDL_RenderCopy(renderer, Message, NULL, &Message_rect);
  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
  //SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_ADD);
  //SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_MOD);

	for (Particle* particle : simulator.particles) {
    // 1.15^255 is the density of the core of the sun. So this calculation gives us a 
    // transparency exponential curve that where centre of sun is 100% opaque)
    unsigned char alpha = (unsigned char)std::max(log2(particle->density)/log2(1.15), 80.0);
    
    SDL_SetRenderDrawColor(renderer, 255, 255, 0, alpha);
    Position pixel = coordSystem->universeToScreen(particle->pos);
	  if(SDL_RenderDrawPoint(renderer, pixel.x, pixel.y) != 0) {
			std::cout << SDL_GetError() << std::endl;
		}
	}
}
