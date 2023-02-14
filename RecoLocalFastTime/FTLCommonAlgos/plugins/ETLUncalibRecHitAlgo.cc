#include "RecoLocalFastTime/FTLCommonAlgos/interface/MTDUncalibratedRecHitAlgoBase.h"
#include "FWCore/MessageLogger/interface/MessageLogger.h"

#include "CommonTools/Utils/interface/FormulaEvaluator.h"

class ETLUncalibRecHitAlgo : public ETLUncalibratedRecHitAlgoBase {
public:
  /// Constructor
  ETLUncalibRecHitAlgo(const edm::ParameterSet& conf, edm::ConsumesCollector& sumes)
      : MTDUncalibratedRecHitAlgoBase<ETLDataFrame>(conf, sumes),
        adcNBits_(conf.getParameter<uint32_t>("adcNbits")),
        adcSaturation_(conf.getParameter<double>("adcSaturation")),
        adcLSB_(adcSaturation_ / (1 << adcNBits_)),
        toaLSBToNS_(conf.getParameter<double>("toaLSB_ns")),
        tocLSBToNS_(conf.getParameter<double>("tocLSB_ns")),
        tofDelay_(conf.getParameter<double>("tofDelay")),
        timeError_(conf.getParameter<std::string>("timeResolutionInNs")),
        timeCorr_p0_(conf.getParameter<double>("timeCorr_p0")),
        timeCorr_p1_(conf.getParameter<double>("timeCorr_p1")),
        timeCorr_p2_(conf.getParameter<double>("timeCorr_p2")) {}

  /// Destructor
  ~ETLUncalibRecHitAlgo() override {}

  /// get event and eventsetup information
  void getEvent(const edm::Event&) final {}
  void getEventSetup(const edm::EventSetup&) final {}

  /// make the rec hit
  FTLUncalibratedRecHit makeRecHit(const ETLDataFrame& dataFrame) const final;

private:
  const uint32_t adcNBits_;
  const double adcSaturation_;
  const double adcLSB_;
  const double toaLSBToNS_;
  const double tocLSBToNS_;
  const double tofDelay_;
  const reco::FormulaEvaluator timeError_;
  const double timeCorr_p0_;
  const double timeCorr_p1_;
  const double timeCorr_p2_;
};

FTLUncalibratedRecHit ETLUncalibRecHitAlgo::makeRecHit(const ETLDataFrame& dataFrame) const {
  constexpr int iSample = 2;  //only in-time sample
  const auto& sample = dataFrame.sample(iSample);

  std::pair<float, float> amplitude(0., 0.);
  std::pair<float, float> time(0., 0.);

  amplitude.first = double(sample.data()) * adcLSB_;
  double time_of_arrival = double(sample.toa()) * toaLSBToNS_ - tofDelay_;
  double time_of_crossing = double(sample.toc()) * tocLSBToNS_ - tofDelay_;
  double time_over_threshold = time_of_crossing - time_of_arrival;
  unsigned char flag = 0;

  // Time-walk correction for toa
  time.first -= timeCorr_p0_ * pow(time_over_threshold, timeCorr_p1_) + timeCorr_p2_;
  flag |= 0x1;

  LogDebug("ETLUncalibRecHit") << "ADC+: set the charge to: " << amplitude.first << ' ' << sample.data() << ' ' << adcLSB_
                               << ' ' << std::endl;
  LogDebug("ETLUncalibRecHit") << "ADC+: set the time to: " << time.first << ' ' << sample.toa() << ' ' << toaLSBToNS_ << ' '
                               << std::endl;
  LogDebug("ETLUncalibRecHit") << "Final uncalibrated amplitude : " << amplitude.first << std::endl;

  // NB: Here amplitudeV is defined as an array in order to be used
  //     below as an input to FormulaEvaluator::evaluate.
  const std::array<double, 1> amplitudeV = {{amplitude.first}};
  const std::array<double, 1> emptyV = {{0.}};
  double timeError = timeError_.evaluate(amplitudeV, emptyV);

  return FTLUncalibratedRecHit(dataFrame.id(),
                               dataFrame.row(),
                               dataFrame.column(),
                               amplitude,
                               time,
                               timeError,
                               -1.f,
                               -1.f,
                               flag);
}

#include "FWCore/Framework/interface/MakerMacros.h"
DEFINE_EDM_PLUGIN(ETLUncalibratedRecHitAlgoFactory, ETLUncalibRecHitAlgo, "ETLUncalibRecHitAlgo");
