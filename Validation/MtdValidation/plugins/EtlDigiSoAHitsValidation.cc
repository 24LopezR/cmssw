// -*- C++ -*-
//
// Package:    Validation/MtdValidation
// Class:      EtlDigiSoAHitsValidation
//
/**\class EtlDigiSoAHitsValidation EtlDigiSoAHitsValidation.cc Validation/MtdValidation/plugins/EtlDigiSoAHitsValidation.cc

 Description: ETL DIGI hits validation

*/

#include <string>

#include "FWCore/Framework/interface/Frameworkfwd.h"
#include "FWCore/Framework/interface/Event.h"
#include "FWCore/Framework/interface/MakerMacros.h"
#include "FWCore/ParameterSet/interface/ParameterSet.h"

#include "DQMServices/Core/interface/DQMEDAnalyzer.h"
#include "DQMServices/Core/interface/DQMStore.h"

#include "DataFormats/Common/interface/ValidHandle.h"
#include "DataFormats/ForwardDetId/interface/ETLDetId.h"
#include "DataFormats/FTLDigiSoA/interface/ETLDigiHostCollection.h"

#include "Geometry/Records/interface/MTDDigiGeometryRecord.h"
#include "Geometry/Records/interface/MTDTopologyRcd.h"
#include "Geometry/MTDGeometryBuilder/interface/MTDGeometry.h"
#include "Geometry/MTDGeometryBuilder/interface/MTDTopology.h"

#include "Geometry/MTDGeometryBuilder/interface/ProxyMTDTopology.h"
#include "Geometry/MTDGeometryBuilder/interface/RectangularMTDTopology.h"

#include "Geometry/MTDCommonData/interface/MTDTopologyMode.h"

class EtlDigiSoAHitsValidation : public DQMEDAnalyzer {
public:
  explicit EtlDigiSoAHitsValidation(const edm::ParameterSet&);
  ~EtlDigiSoAHitsValidation() override;

  static void fillDescriptions(edm::ConfigurationDescriptions& descriptions);

private:
  void bookHistograms(DQMStore::IBooker&, edm::Run const&, edm::EventSetup const&) override;

  void analyze(const edm::Event&, const edm::EventSetup&) override;

  // ------------ member data ------------

  const std::string folder_;
  const bool optionalPlots_;

  edm::EDGetTokenT<etldigi::ETLDigiHostCollection> etlDigiHitsToken_;

  edm::ESGetToken<MTDGeometry, MTDDigiGeometryRecord> mtdgeoToken_;
  edm::ESGetToken<MTDTopology, MTDTopologyRcd> mtdtopoToken_;

  // --- histograms declaration

  MonitorElement* meNhits_;

  MonitorElement* meHitCharge_;
  MonitorElement* meHitToA_;
  MonitorElement* meHitToT_;
  MonitorElement* meHitCAL_;

  MonitorElement* meOccupancy_;

  //local position monitoring
  MonitorElement* meLocalOccupancy_;
  MonitorElement* meHitXlocal_;
  MonitorElement* meHitYlocal_;
  MonitorElement* meHitZlocal_;

  MonitorElement* meHitX_;
  MonitorElement* meHitY_;
  MonitorElement* meHitZ_;
  MonitorElement* meHitPhi_;
  MonitorElement* meHitEta_;

  MonitorElement* meHitToAVsQ_;
  MonitorElement* meHitToTVsQ_;
  MonitorElement* meHitCALVsQ_;
  MonitorElement* meHitQvsPhi_;
  MonitorElement* meHitQvsEta_;
  MonitorElement* meHitQvsZ_;
  MonitorElement* meHitToAVsPhi_;
  MonitorElement* meHitToTVsPhi_;
  MonitorElement* meHitCALVsPhi_;
  MonitorElement* meHitToAVsEta_;
  MonitorElement* meHitToTVsEta_;
  MonitorElement* meHitCALVsEta_;
  MonitorElement* meHitToAVsZ_;
  MonitorElement* meHitToTVsZ_;
  MonitorElement* meHitCALVsZ_;
};

// ------------ constructor and destructor --------------
EtlDigiSoAHitsValidation::EtlDigiSoAHitsValidation(const edm::ParameterSet& iConfig)
    : folder_(iConfig.getParameter<std::string>("folder")),
      optionalPlots_(iConfig.getParameter<bool>("optionalPlots")) {
  etlDigiHitsToken_ = consumes<etldigi::ETLDigiHostCollection>(iConfig.getParameter<edm::InputTag>("inputTag"));
  mtdgeoToken_ = esConsumes<MTDGeometry, MTDDigiGeometryRecord>();
  mtdtopoToken_ = esConsumes<MTDTopology, MTDTopologyRcd>();
}

EtlDigiSoAHitsValidation::~EtlDigiSoAHitsValidation() {}

// ------------ method called for each event  ------------
void EtlDigiSoAHitsValidation::analyze(const edm::Event& iEvent, const edm::EventSetup& iSetup) {
  using namespace edm;

  auto geometryHandle = iSetup.getTransientHandle(mtdgeoToken_);
  const MTDGeometry* geom = geometryHandle.product();

  auto topologyHandle = iSetup.getTransientHandle(mtdtopoToken_);
  const MTDTopology* topology = topologyHandle.product();

  auto etlDigiHitsHandle = makeValid(iEvent.getHandle(etlDigiHitsToken_));

  // --- Loop over the BTL DIGI hits

  unsigned int n_digi_etl = 0;
  const auto etlDigiView = etlDigiHitsHandle->view();
  for (int i = 0; i < etlDigiView.metadata().size(); i++) {
    ETLDetId detId = etldigi::rawId(etlDigiView, i);
    DetId geoId = detId.geographicalId();
    const MTDGeomDet* thedet = geom->idToDet(geoId);
    if (thedet == nullptr)
      throw cms::Exception("EtlDigiSoAHitsValidation") << "GeographicalID: " << std::hex << geoId.rawId() << " ("
                                                       << detId.rawId() << ") is invalid!" << std::dec << std::endl;

    const PixelTopology& topo = static_cast<const PixelTopology&>(thedet->topology());

    Local3DPoint local_point(topo.localX(etldigi::rowID(etlDigiView, i)), topo.localY(etldigi::colID(etlDigiView, i)), 0.);
    const auto& global_point = thedet->toGlobal(local_point);

    uint16_t adc = etldigi::charge(etlDigiView, i);
    uint16_t toa = etldigi::ToAdata(etlDigiView, i);
    uint16_t tot = etldigi::ToTdata(etlDigiView, i);
    uint16_t cal = etldigi::CALdata(etlDigiView, i);

    meHitCharge_->Fill(adc);
    meHitToA_->Fill(toa);
    meHitToT_->Fill(tot);
    meHitCAL_->Fill(cal);

    meOccupancy_->Fill(global_point.z(), global_point.phi());

    if (optionalPlots_) {
      meLocalOccupancy_->Fill(local_point.x(), local_point.y());
      meHitXlocal_->Fill(local_point.x());
      meHitYlocal_->Fill(local_point.y());
      meHitZlocal_->Fill(local_point.z());
    }

    meHitX_->Fill(global_point.x());
    meHitY_->Fill(global_point.y());
    meHitZ_->Fill(global_point.z());
    meHitPhi_->Fill(global_point.phi());
    meHitEta_->Fill(global_point.eta());

    meHitToAVsQ_->Fill(adc, toa);
    meHitToTVsQ_->Fill(adc, tot);
    meHitCALVsQ_->Fill(adc, cal);

    meHitQvsPhi_->Fill(global_point.phi(), adc);
    meHitToAVsPhi_->Fill(global_point.phi(), toa);
    meHitToTVsPhi_->Fill(global_point.phi(), tot);
    meHitCALVsPhi_->Fill(global_point.phi(), cal);

    meHitQvsEta_->Fill(global_point.eta(), adc);
    meHitToAVsEta_->Fill(global_point.eta(), toa);
    meHitToTVsEta_->Fill(global_point.eta(), tot);
    meHitCALVsEta_->Fill(global_point.eta(), cal);

    meHitQvsZ_->Fill(global_point.z(), adc);
    meHitToAVsZ_->Fill(global_point.z(), toa);
    meHitToTVsZ_->Fill(global_point.z(), tot);
    meHitCALVsZ_->Fill(global_point.z(), cal);

    n_digi_etl++;

  }  // dataFrame loop

  if (n_digi_etl > 0)
    meNhits_->Fill(log10(n_digi_etl));
}

// ------------ method for histogram booking ------------
void EtlDigiSoAHitsValidation::bookHistograms(DQMStore::IBooker& ibook,
                                              edm::Run const& run,
                                              edm::EventSetup const& iSetup) {
  ibook.setCurrentFolder(folder_);
  // --- histograms booking

  meNhits_ = ibook.book1D("EtlNhits", "Number of ETL DIGI hits;log_{10}(N_{DIGI})", 100, 0., 5.25);

  meHitCharge_ = ibook.book1D("EtlHitCharge", "ETL DIGI hits charge;Q_{DIGI} [ADC counts]", 100, 0., 1024.);
  meHitToA_    = ibook.book1D("EtlHitToA", "ETL DIGI hits Time of Arrival;ToA_{DIGI} [# clk cycles]", 100, 0., 1024.);
  meHitToT_    = ibook.book1D("EtlHitToT", "ETL DIGI hits Time over Threshold;ToT_{DIGI} [# clk cycles]", 100, 0., 1024.);
  meHitCAL_    = ibook.book1D("EtlHitCAL", "ETL DIGI hits Calibration code;CAL_{DIGI}", 100, 0., 1024.);

  meOccupancy_ = ibook.book2D("EtlOccupancy",
                                 "ETL DIGI hits occupancy;Z_{DIGI} [cm]; #phi_{DIGI} [rad]",
                                 65,
                                 -260.,
                                 260.,
                                 126,
                                 -3.15,
                                 3.15);
  if (optionalPlots_) {
    meLocalOccupancy_ = ibook.book2D("EtlLocalOccupancy",
                                        "ETL DIGI hits local occupancy;X_{DIGI} [cm]; Y_{DIGI} [cm]",
                                        100,
                                        -10.,
                                        10,
                                        60,
                                        -3.,
                                        3.);
    meHitXlocal_ = ibook.book1D("EtlHitXlocal", "ETL DIGI local X ;X_{DIGI}^{LOC} [cm]", 100, -10., 10.);
    meHitYlocal_ = ibook.book1D("EtlHitYlocal", "ETL DIGI local Y ;Y_{DIGI}^{LOC} [cm]", 60, -3., 3.);
    meHitZlocal_ = ibook.book1D("EtlHitZlocal", "ETL DIGI local z ;z_{DIGI}^{LOC} [cm]", 10, -1, 1);
  }

  meHitX_ = ibook.book1D("EtlHitX", "ETL DIGI hits X;X_{DIGI} [cm]", 60, -120., 120.);
  meHitY_ = ibook.book1D("EtlHitY", "ETL DIGI hits Y;Y_{DIGI} [cm]", 60, -120., 120.);
  meHitZ_ = ibook.book1D("EtlHitZ", "ETL DIGI hits Z;Z_{DIGI} [cm]", 100, -260., 260.);
  meHitPhi_ = ibook.book1D("EtlHitPhi", "ETL DIGI hits #phi;#phi_{DIGI} [rad]", 126, -3.15, 3.15);
  meHitEta_ = ibook.book1D("EtlHitEta", "ETL DIGI hits #eta;#eta_{DIGI}", 100, -1.55, 1.55);

  meHitToAVsQ_ = ibook.bookProfile(
      "EtlHitToAVsQ", "ETL DIGI Time of Arrival vs charge;Q_{DIGI} [ADC counts];ToA_{DIGI} [# clk cycles]", 50, 0., 1024., 0., 1024.);
  meHitToTVsQ_ = ibook.bookProfile(
      "EtlHitToTVsQ", "ETL DIGI Time over Threshold vs charge;Q_{DIGI} [ADC counts];ToT_{DIGI} [# clk cycles]", 50, 0., 1024., 0., 1024.);
  meHitCALVsQ_ = ibook.bookProfile(
      "EtlHitToAVsQ", "ETL DIGI Calibration code vs charge;Q_{DIGI} [ADC counts];CAL_{DIGI}", 50, 0., 1024., 0., 1024.);

  meHitQvsPhi_ = ibook.bookProfile(
      "EtlHitQvsPhi", "ETL DIGI charge vs #phi;#phi_{DIGI} [rad];Q_{DIGI} [ADC counts]", 50, -3.15, 3.15, 0., 1024.);
  meHitQvsEta_ = ibook.bookProfile(
      "EtlHitQvsEta", "ETL DIGI charge vs #eta;#eta_{DIGI};Q_{DIGI} [ADC counts]", 50, -1.55, 1.55, 0., 1024.);
  meHitQvsZ_ = ibook.bookProfile(
      "EtlHitQvsZ", "ETL DIGI charge vs Z;Z_{DIGI} [cm];Q_{DIGI} [ADC counts]", 50, -260., 260., 0., 1024.);

  meHitToAVsPhi_ = ibook.bookProfile(
      "EtlHitToAVsPhi", "ETL DIGI Time of Arrival vs #phi;#phi_{DIGI} [rad];ToA_{DIGI} [# clk cycles]", 50, -3.15, 3.15, 0., 1024.);
  meHitToTVsPhi_ = ibook.bookProfile(
      "EtlHitToTVsPhi", "ETL DIGI Time over Threshold vs #phi;#phi_{DIGI} [rad];ToT_{DIGI} [# clk cycles]", 50, -3.15, 3.15, 0., 1024.);
  meHitCALVsPhi_ = ibook.bookProfile(
      "EtlHitCALVsPhi", "ETL DIGI Calibration code vs #phi;#phi_{DIGI} [rad];CAL_{DIGI}", 50, -3.15, 3.15, 0., 1024.);

  meHitToAVsEta_ = ibook.bookProfile(
      "EtlHitToAVsEta", "ETL DIGI Time of Arrival vs #eta;#eta_{DIGI};ToA_{DIGI} [# clk cycles]", 50, -1.55, 1.55, 0., 1024.);
  meHitToTVsEta_ = ibook.bookProfile(
      "EtlHitToTVsEta", "ETL DIGI Time over Threshold vs #eta;#eta_{DIGI};ToT_{DIGI} [# clk cycles]", 50, -1.55, 1.55, 0., 1024.);
  meHitCALVsEta_ = ibook.bookProfile(
      "EtlHitCALVsEta", "ETL DIGI Calibration code vs #eta;#eta_{DIGI};CAL_{DIGI}", 50, -1.55, 1.55, 0., 1024.);

  meHitToAVsZ_ = ibook.bookProfile(
      "EtlHitToAVsZ", "ETL DIGI Time of Arrival vs Z;Z_{DIGI} [cm];ToA_{DIGI} [# clk cycles]", 50, -260., 260., 0., 1024.);
  meHitToTVsZ_ = ibook.bookProfile(
      "EtlHitToTVsZ", "ETL DIGI Time over Threshold vs Z;Z_{DIGI} [cm];ToT_{DIGI} [# clk cycles]", 50, -260., 260., 0., 1024.);
  meHitCALVsZ_ = ibook.bookProfile(
      "EtlHitCALVsZ", "ETL DIGI Calibration code vs Z;Z_{DIGI} [cm];CAL_{DIGI}", 50, -260., 260., 0., 1024.);
}

// ------------ method fills 'descriptions' with the allowed parameters for the module  ------------
void EtlDigiSoAHitsValidation::fillDescriptions(edm::ConfigurationDescriptions& descriptions) {
  edm::ParameterSetDescription desc;

  desc.add<std::string>("folder", "MTD/ETL/DigiHitsSoA");
  desc.add<edm::InputTag>("inputTag", edm::InputTag("mix", "FTLEndcapSoA"));
  desc.add<bool>("optionalPlots", false);

  descriptions.add("etlDigiSoAHitsDefaultValid", desc);
}

DEFINE_FWK_MODULE(EtlDigiSoAHitsValidation);
