#pragma once

#include "../physics/rigid_body.h"
#include "../physics/world.h"
#include "../rendering/texture.h"
#include "../rendering/camera.h"

// A class for creating simulations, it places objects and sets the camera

class Setup {
public:
	static void makeBallFalling(std::shared_ptr<World> engine, Texture& crateTexture, Texture& squareTexture);
	static void makeJenga(std::shared_ptr<World> engine, Texture& groundTexture, Texture& boxTexture, Texture& sphereTexture);
	static void makeAngryBirds(std::shared_ptr<World> engine, Texture& groundTexture, Texture& boxTexture, Texture& sphereTexture, std::shared_ptr<Camera> camera);
	static void makeBallPit(std::shared_ptr<World> engine, Texture& groundTexture, Texture& boxTexture, Texture& sphereTexture, std::shared_ptr<Camera> camera);
	static void makeDiamondLauncher(std::shared_ptr<World> engine, Texture& groundTexture, Texture& boxTexture, Texture& sphereTexture, Texture& diamondTexture, std::shared_ptr<Camera> camera);
	static void makeDiamond(std::shared_ptr<World> engine, Texture& groundTexture, Texture& boxTexture, Texture& sphereTexture, Texture& diamondTexture, std::shared_ptr<Camera> camera);
	};