#ifndef FastSimulation_TrackingRecHitProducer_TrackingRecHitPipe_H
#define FastSimulation_TrackingRecHitProducer_TrackingRecHitPipe_H

#include "FastSimulation/TrackingRecHitProducer/interface/TrackingRecHitAlgorithm.h"
#include "FastSimulation/TrackingRecHitProducer/interface/TrackingRecHitProduct.h"

#include <vector>

class TrackingRecHitPipe
{
    protected:
        std::vector<TrackingRecHitAlgorithm*> _algorithms;
        
    public:
        TrackingRecHitPipe()
        {
        }
        
        void produce(TrackingRecHitProductPtr product) const
        {
            for (unsigned int ialgo = 0; ialgo < _algorithms.size(); ++ialgo)
            {
                product = _algorithms[ialgo]->process(product);
            }
        }
        
        inline void addAlgorithm(TrackingRecHitAlgorithm* algorithm)
        {
            _algorithms.push_back(algorithm);
        }
        
};

#endif

