#include <iostream>
#include "simple_viewer.h"

int main()
{
    std::cout << "Starting LiDAR Playback Viewer..." << std::endl;

    startViewer();

    // Viewer läuft in eigenem Thread → Main bleibt einfach offen
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
