#pragma once
#include <vector>
#include <list>
#include "PhysicsStructures.h"
#include "SFML/Graphics.hpp"
#include <algorithm>

class Simulation
{
    int window_length;
    int frames_per_second;
    float delta_time;
    float SCALE;
    std::vector<MassPoint> MassPoints;
    std::vector<Spring> Springs;
    std::vector<PressureSpringMassModel> SpringMassModels;

public:
    Simulation();

    void render(int WindowLength, int FPS, float speed);

private:
    Vector2 toScreenSpace(Vector2 point);

    Vector2 toSimulationSpace(Vector2 point);

    void drawMassPoints(sf::RenderWindow* pWindow);

    void applySprings(float dt, sf::RenderWindow* pWindow);

    void applyMassPointCollision(MassPoint* mass_point);

    void applyMassPointPhysics(float dt);

    void applyPressureMassPhysiscs(float dt);

    void generateSprings(PressureSpringMassModel& model);
};

