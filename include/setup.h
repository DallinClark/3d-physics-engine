#pragma once

#include "rigid_body.h"
#include "world.h"
#include "texture.h"
#include "camera.h"

// A class for creating simulations, it places objects and sets the camera

class Setup {
public:
	static void makeBallFalling(std::shared_ptr<World> engine, Texture& crateTexture, Texture& squareTexture);
	static void makeJenga(std::shared_ptr<World> engine, Texture& groundTexture, Texture& boxTexture, Texture& sphereTexture);
	static void makeAngryBirds(std::shared_ptr<World> engine, Texture& groundTexture, Texture& boxTexture, Texture& sphereTexture, std::shared_ptr<Camera> camera);
	static void makeBallPit(std::shared_ptr<World> engine, Texture& groundTexture, Texture& boxTexture, Texture& sphereTexture, std::shared_ptr<Camera> camera);
};