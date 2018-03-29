#ifndef STRUCTS_H_
#define STRUCTS_H_

#include "mediation_layer/ml_class.h"
#include <thread>

struct globalVariables {
    // Mutex protected variables
    MediationLayer obj_mid_layer;
    std::vector<Balloon> balloons;
};

class mutexStruct {
 public:
    pthread_mutex_t m_ml_class;
    pthread_mutex_t m_balloons;

    // Methods
    mutexStruct() {
        pthread_mutex_init(&m_ml_class, NULL);
        pthread_mutex_init(&m_balloons, NULL);
    }
    void destroy() {
        pthread_mutex_destroy(&m_ml_class);
        pthread_mutex_destroy(&m_balloons);
    }
};

#endif  // STRUCTS_H_