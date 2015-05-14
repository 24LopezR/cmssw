#ifndef RecoTracker_FinalTrackSelectors_ClassifierMerger_h
#define RecoTracker_FinalTrackSelectors_ClassifierMerger_h

#include "FWCore/Framework/interface/Frameworkfwd.h"
#include "FWCore/Framework/interface/global/EDProducer.h"

#include "FWCore/Framework/interface/Event.h"
#include "FWCore/ParameterSet/interface/ParameterSet.h"
#include "FWCore/ParameterSet/interface/ParameterSetDescription.h"
#include "FWCore/ParameterSet/interface/ConfigurationDescriptions.h"


#include<vector>
#include<memory>

namespace {
  class ClassifierMerger final : public edm::global::EDProducer<> {
   public:
    explicit ClassifierMerger(const edm::ParameterSet& conf) {
      for (auto const & it : conf.getParameter<std::vector<std::string> >("inputClassifiers")) {
	srcMVAs.push_back(consumes<MVACollection>(edm::InputTag(it,"MVAVals")));
	srcQuals.push_back(consumes<QualityMaskCollection>(edm::InputTag(it,"QualityMasks")));
      }

      produces<MVACollection>("MVAVals");
      produces<QualityMaskCollection>("QualityMasks");

    }
      
    static void  fillDescriptions(edm::ConfigurationDescriptions& descriptions) {
      edm::ParameterSetDescription desc;
      desc.add<std::vector<std::string> >("inputClassifiers",std::vector<std::string>());
      descriptions.add("ClassifierMerger", desc);
    }

   private:

      using MVACollection = std::vector<float>;
      using QualityMaskCollection = std::vector<unsigned char>;

      virtual void produce(edm::StreamID, edm::Event& evt, const edm::EventSetup&) const override {

	// get Master
	edm::Handle<MVACollection> hmva;
	evt.getByToken(srcMVAs[0], hmva);
	auto size = (*hmva).size();

	edm::Handle<QualityMaskCollection> hqual;
	evt.getByToken(srcQuals[0], hqual);

	
	// products
	std::unique_ptr<MVACollection> mvas(new MVACollection((*hmva)));
	std::unique_ptr<QualityMaskCollection> quals(new QualityMaskCollection(*hqual));
	
	for (auto i=1U; i<srcQuals.size(); ++i) {
	  evt.getByToken(srcQuals[i], hqual);
	  auto const & iq = *hqual;
	  assert(iq.size()==size);
	  for (auto j=0U; j!=size; ++j) (*quals)[j] |= iq[j];
	}

	evt.put(std::move(mvas),"MVAValues");
	evt.put(std::move(quals),"QualityMasks");
	
      }
      
      std::vector<edm::EDGetTokenT<MVACollection>> srcMVAs;
      std::vector<edm::EDGetTokenT<QualityMaskCollection>> srcQuals;
      

  };
}


#include "FWCore/PluginManager/interface/ModuleDef.h"
#include "FWCore/Framework/interface/MakerMacros.h"

DEFINE_FWK_MODULE(ClassifierMerger);


#endif
