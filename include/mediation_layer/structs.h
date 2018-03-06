#ifndef STRUCTS_H_
#define STRUCTS_H_

#include "mediation_layer/ml_class.h"
#include <thread>

struct globalVariables {
    // Mutex protected variables
    MediationLayer obj_mid_layer;
};

class mutexStruct {
 public:
    pthread_mutex_t m_ml_class;

    // Methods
    mutexStruct() {
        pthread_mutex_init(&m_ml_class, NULL);
    }
    void destroy() {
        pthread_mutex_destroy(&m_ml_class);
    }
};

#endif  // STRUCTS_H_