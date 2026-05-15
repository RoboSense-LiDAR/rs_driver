#pragma once

#include <vector>

struct SimplePoint
{
    float x, y, z;
    float intensity;
};

// Viewer API
void startViewer();
void updatePointCloud(const std::vector<SimplePoint>& pts);

// NEU: globale Intensität
//extern float g_min_intensity;
//extern float g_max_intensity;
