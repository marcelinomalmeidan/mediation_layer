#ifndef THREADS_H_
#define THREADS_H_

#include "mediation_layer/globals.h"

namespace threads {

// Thread that runs the mediation layer algorithm
void MediationLayerThread(const double &rate);

// Thread for verifying heartbeat of quad references and position
void HeartbeatThread();

// Thread for publishing the game state
void GameStatePubThread(const double &rate);

// Thread for publishing static objects. These are republished with
// low frequency just in case RVIZ is restarted
void StaticObjectsVisualizationThread();

// Thread for publishing moving objects (high frequency update)
void VisualizationThread(const double &rate);

}  // namespace threads

#endif  // THREADS_H_