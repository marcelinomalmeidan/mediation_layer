#ifndef THREADS_H_
#define THREADS_H_

#include "mediation_layer/globals.h"

namespace threads {

// Thread that runs the mediation layer algorithm
void MediationLayerThread();

// Thread for publishing static objects. These are republished with
// low frequency just in case RVIZ is restarted
void StaticObjectsVisualizationThread();

// Thread for publishing moving objects (high frequency update)
void VisualizationThread();

}  // namespace threads

#endif  // THREADS_H_