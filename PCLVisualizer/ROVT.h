#ifndef ROVT_H
#define ROVT_H

#include <pcl/visualization/pcl_visualizer.h>

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer);
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer);
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,	void* nothing);
#endif
