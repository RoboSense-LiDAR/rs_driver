#pragma once

#include <vector>
#include <atomic>
#include <string>

struct SimplePoint
{
    float x, y, z;
    float intensity;
};

// Viewer API
void startViewer();
void updatePointCloud(const std::vector<SimplePoint>& pts);

// Aufnahme-Steuerung
extern std::atomic<bool> g_recording;
extern std::string g_output_folder;
extern uint64_t g_frame_counter;

