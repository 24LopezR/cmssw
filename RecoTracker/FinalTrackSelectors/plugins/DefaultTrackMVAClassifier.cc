#ifndef RecoTracker_FinalTrackSelectors_mva_h
#define RecoTracker_FinalTrackSelectors_mva_h

#include "RecoTracker/FinalTrackSelectors/interface/TrackMVAClassifier.h"


#include "DataFormats/TrackReco/interface/Track.h"

namespace {
  
template<bool PROMPT>
struct mva {
  float operator()(reco::Track const & trk,
		   reco::BeamSpot const & beamSpot,
		   reco::VertexCollection const & vertices,
		   GBRForest const & forest) const {

    auto tmva_ndof_ = trk.ndof();
    auto tmva_nlayers_ = trk.hitPattern().trackerLayersWithMeasurement();
    auto tmva_nlayers3D_ = trk.hitPattern().pixelLayersWithMeasurement()
        + trk.hitPattern().numberOfValidStripLayersWithMonoAndStereo();
    auto tmva_nlayerslost_ = trk.hitPattern().trackerLayersWithoutMeasurement(reco::HitPattern::TRACK_HITS);
    float chi2n =  trk.normalizedChi2();
    float chi2n_no1Dmod = chi2n;
    
    int count1dhits = 0;
    /*
    auto ith = trk.extra()->firstRecHit();
    auto  edh = ith + trk.recHitsSize();
    for (; ith<edh; ++ith) {
      const TrackingRecHit & hit = srcHits[ith];
      if (hit.dimension()==1) ++count1dhits;
    }
    */
    if (count1dhits > 0) {
      float chi2 = trk.chi2();
      float ndof = trk.ndof();
      chi2n = (chi2+count1dhits)/float(ndof+count1dhits);
    }
    auto tmva_chi2n_ = chi2n;
    auto tmva_chi2n_no1dmod_ = chi2n_no1Dmod;
    auto tmva_eta_ = trk.eta();
    auto tmva_relpterr_ = float(trk.ptError())/std::max(float(trk.pt()),0.000001f);
    auto tmva_nhits_ = trk.numberOfValidHits();
    int lostIn = trk.hitPattern().numberOfLostTrackerHits(reco::HitPattern::MISSING_INNER_HITS);
    int lostOut = trk.hitPattern().numberOfLostTrackerHits(reco::HitPattern::MISSING_OUTER_HITS);
    auto tmva_minlost_ = std::min(lostIn,lostOut);
    auto tmva_lostmidfrac_ = trk.numberOfLostHits() / (trk.numberOfValidHits() + trk.numberOfLostHits());

    float gbrVals_[11];
    gbrVals_[0] = tmva_lostmidfrac_;
    gbrVals_[1] = tmva_minlost_;
    gbrVals_[2] = tmva_nhits_;
    gbrVals_[3] = tmva_relpterr_;
    gbrVals_[4] = tmva_eta_;
    gbrVals_[5] = tmva_chi2n_no1dmod_;
    gbrVals_[6] = tmva_chi2n_;
    gbrVals_[7] = tmva_nlayerslost_;
    gbrVals_[8] = tmva_nlayers3D_;
    gbrVals_[9] = tmva_nlayers_;
    gbrVals_[10] = tmva_ndof_;


    return forest.GetClassifier(gbrVals_);
    
  }

};

using TrackMVAClassifierDetached = TrackMVAClassifier<mva<false>>;
// typedef TrackMVAClassifier<mva<false>> TrackMVAClassifierDetached;
  
}

#include "FWCore/PluginManager/interface/ModuleDef.h"
#include "FWCore/Framework/interface/MakerMacros.h"

DEFINE_FWK_MODULE(TrackMVAClassifierDetached);


#endif
