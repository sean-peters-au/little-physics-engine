#include "ecs_simulator.h"
#include "simulator_constants.h"
#include "systems/barnes_hut_system.h"

#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <sstream>
#include <iomanip>

int main(int, char**) {
	// Initialize simulation constants for gas-to-planet formation
	SimulatorConstants::initializeConstants(SimulatorConstants::SimulationType::CELESTIAL_GAS);

	if (SDL_Init(SDL_INIT_VIDEO) != 0) {
		std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
		return 1;
	}
	if (TTF_Init() != 0) {
		std::cout << "TTF_Init Error: " << SDL_GetError() << std::endl;
		return 1;
	}

	SDL_Window *window;
	SDL_Renderer *renderer;
	SDL_CreateWindowAndRenderer(SimulatorConstants::ScreenLength, 
								SimulatorConstants::ScreenLength, 0, &window, &renderer);

	// Initialize font for FPS counter
	TTF_Font* font = TTF_OpenFont("/System/Library/Fonts/Helvetica.ttc", 16);
	if (!font) {
		std::cout << "TTF_OpenFont Error: " << TTF_GetError() << std::endl;
		return 1;
	}

	CoordinateSystem coordSystem;
	ECSSimulator simulator(&coordSystem);
	simulator.init();

	const int FPS = 60;
	const int frameDelay = 1000 / FPS;
	Uint32 frameStart;
	int frameTime;
	float avgFPS = 0;
	int frameCount = 0;
	Uint32 fpsTimer = SDL_GetTicks();

	bool running = true;
	while(running) {
		frameStart = SDL_GetTicks();

		SDL_Event event;
		while(SDL_PollEvent(&event)) {
			if(event.type == SDL_QUIT) {
				running = false;
			}
		}

		simulator.tick();
		
		// Basic rendering - just draw white dots for now
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
		SDL_RenderClear(renderer);
		
		auto& registry = simulator.getRegistry();
		auto view = registry.view<Components::Position>();

		// Draw particles with size based on density
		SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
		for (auto [entity, pos] : view.each()) {
			// Draw a 2x2 pixel dot for better visibility
			SDL_Rect point = {
				static_cast<int>(pos.x) - 1,
				static_cast<int>(pos.y) - 1,
				2, 2
			};
			SDL_RenderFillRect(renderer, &point);
		}
		
		// Calculate and display FPS
		frameTime = SDL_GetTicks() - frameStart;
		frameCount++;
		
		if (SDL_GetTicks() - fpsTimer >= 1000) {
			avgFPS = frameCount / ((SDL_GetTicks() - fpsTimer) / 1000.0f);
			
			// Render FPS text
			std::stringstream ss;
			ss << std::fixed << std::setprecision(1) << avgFPS << " FPS";
			SDL_Color textColor = {255, 255, 255, 255};
			SDL_Surface* surface = TTF_RenderText_Solid(font, ss.str().c_str(), textColor);
			SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
			
			SDL_Rect dstRect = {10, 10, surface->w, surface->h};
			SDL_RenderCopy(renderer, texture, NULL, &dstRect);
			
			SDL_FreeSurface(surface);
			SDL_DestroyTexture(texture);
			
			frameCount = 0;
			fpsTimer = SDL_GetTicks();
		}
		
		SDL_RenderPresent(renderer);
		
		if (frameDelay > frameTime) {
			SDL_Delay(frameDelay - frameTime);
		}
	}

	TTF_CloseFont(font);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	TTF_Quit();
	SDL_Quit();

	return 0;
}
