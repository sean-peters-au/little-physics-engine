#include "simulator.h"
#include "simulator_constants.h"
#include "drawer.h"

#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2_ttf/SDL_ttf.h>

int main(int, char**){
	if (SDL_Init(SDL_INIT_VIDEO) != 0) {
		std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
		return 1;
	}
  if (TTF_Init() != 0) {
		std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
		return 1;
  }

	// Initialise SDL
	SDL_Window *window;
	SDL_Renderer *renderer;
	SDL_CreateWindowAndRenderer(SimulatorConstants::ScreenLength, SimulatorConstants::ScreenLength, 0, &window, &renderer);

	// Initialise Simulation	
	ParticleSimulator simulator = ParticleSimulator();
	simulator.init();
	Drawer drawer = Drawer(simulator);
	

	bool running = true;
	unsigned int lastTicks = 0, nowTicks;
  unsigned int steps = 0;

	while(running) {
		SDL_Event events;
		while(SDL_PollEvent(&events)) {
			if(events.type == SDL_QUIT)
				running = false;
		}

		nowTicks = SDL_GetTicks();
		if(nowTicks - lastTicks < 1000 / SimulatorConstants::StepsPerSecond) {
			SDL_Delay(nowTicks - lastTicks);
      lastTicks = nowTicks;
		}

		simulator.tick();
		drawer.draw(renderer, steps);
		
	  SDL_RenderPresent(renderer); 
    steps++;
	}

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
	SDL_Quit();
	return 0;
}
