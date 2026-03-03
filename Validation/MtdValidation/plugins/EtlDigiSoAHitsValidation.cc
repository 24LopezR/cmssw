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

  MonitorElement* meNhits_[4];
  MonitorElement* meNhitsPerLGAD_[4];
  MonitorElement* meNLgadWithHits_[4];
  MonitorElement* meNhitsPerLGADoverQ_[4];
  MonitorElement* meNLgadWithHitsoverQ_[4];
  MonitorElement* meNhitsPerLGADoverEta_[4];
  MonitorElement* meNLgadWithHitsoverEta_[4];

  MonitorElement* meHitCharge_[4];
  MonitorElement* meHitTime_[4];
  MonitorElement* meHitToT_[4];
  MonitorElement* meHitCAL_[4];

  MonitorElement* meOccupancy_[4];

  MonitorElement* meLocalOccupancy_[2];  //folding the two ETL discs
  MonitorElement* meHitXlocal_[2];
  MonitorElement* meHitYlocal_[2];

  MonitorElement* meHitX_[4];
  MonitorElement* meHitY_[4];
  MonitorElement* meHitZ_[4];
  MonitorElement* meHitPhi_[4];
  MonitorElement* meHitEta_[4];

  MonitorElement* meHitTvsQ_[4];
  MonitorElement* meHitToTvsQ_[4];
  MonitorElement* meHitCALvsQ_[4];
  MonitorElement* meHitQvsPhi_[4];
  MonitorElement* meHitQvsEta_[4];
  MonitorElement* meHitTvsPhi_[4];
  MonitorElement* meHitTvsEta_[4];
  MonitorElement* meHitToTvsPhi_[4];
  MonitorElement* meHitToTvsEta_[4];
  MonitorElement* meHitCALvsPhi_[4];
  MonitorElement* meHitCALvsEta_[4];

  std::array<std::unordered_map<uint32_t, uint32_t>, 4> ndigiPerLGAD_;

  // Constants to define the bins for Q and Eta in occupancy studies
  static constexpr int n_bin_Q = 32;
  static constexpr double Q_Min = 0.;
  static constexpr double Q_Max = 256.;

  static constexpr int n_bin_Eta = 3;
  static constexpr double eta_bins_edges_neg[n_bin_Eta + 1] = {-3.0, -2.5, -2.1, -1.5};
  static constexpr double eta_bins_edges_pos[n_bin_Eta + 1] = {1.5, 2.1, 2.5, 3.0};

  std::array<std::unordered_map<uint32_t, std::array<uint32_t, n_bin_Q>>, 4> ndigiPerLGADoverQ_;
  std::array<std::unordered_map<uint32_t, std::array<uint32_t, n_bin_Eta>>, 4> ndigiPerLGADoverEta_;

  /** // --- histograms declaration

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
  MonitorElement* meHitCALVsZ_;**/
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

  // --- Loop over the ETL DIGI hits

  unsigned int n_digi_etl[4] = {0, 0, 0, 0}; 
  for (size_t i = 0; i < 4; i++) {
    ndigiPerLGAD_[i].clear();
    ndigiPerLGADoverQ_[i].clear();
    ndigiPerLGADoverEta_[i].clear();
  }

  size_t index(0);
  const auto etlDigiView = etlDigiHitsHandle->view();
  for (int i = 0; i < etlDigiView.metadata().size(); i++) {
    double weight = 1.0;
    ETLDetId detId = etldigi::rawId(etlDigiView, i);
    DetId geoId = detId.geographicalId();
    const MTDGeomDet* thedet = geom->idToDet(geoId);
    if (thedet == nullptr)
      throw cms::Exception("EtlDigiSoAHitsValidation") << "GeographicalID: " << std::hex << geoId.rawId() << " ("
                                                       << detId.rawId() << ") is invalid!" << std::dec << std::endl;

    const PixelTopology& topo = static_cast<const PixelTopology&>(thedet->topology());

    Local3DPoint local_point(topo.localX(etldigi::rowID(etlDigiView, i)), topo.localY(etldigi::colID(etlDigiView, i)), 0.);
    const auto& global_point = thedet->toGlobal(local_point);

    // --- Fill the histograms

    int idet = 999;
    if (detId.discSide() == 1) {
      weight = -weight;
    }
    if ((detId.zside() == -1) && (detId.nDisc() == 1)) {
      idet = 0;
    } else if ((detId.zside() == -1) && (detId.nDisc() == 2)) {
      idet = 1;
    } else if ((detId.zside() == 1) && (detId.nDisc() == 1)) {
      idet = 2;
    } else if ((detId.zside() == 1) && (detId.nDisc() == 2)) {
      idet = 3;
    } else {
      edm::LogWarning("EtlDigiHitsValidation") << "Unknown ETL DetId configuration: " << detId;
      continue;
    }

    index++;
    LogDebug("EtlDigiHitsValidation") << "Digi # " << index << " DetId " << detId.rawId() << " idet " << idet;

    uint16_t adc = etldigi::charge(etlDigiView, i);
    uint16_t toa = etldigi::ToAdata(etlDigiView, i);
    uint16_t tot = etldigi::ToTdata(etlDigiView, i);
    uint16_t cal = etldigi::CALdata(etlDigiView, i);

    meHitCharge_[idet]->Fill(adc);
    meHitTime_[idet]->Fill(toa);
    meHitToT_[idet]->Fill(tot);
    meHitCAL_[idet]->Fill(cal);
    meOccupancy_[idet]->Fill(global_point.x(), global_point.y(), weight);

    if (optionalPlots_) {
      if ((idet == 0) || (idet == 1)) {
        meLocalOccupancy_[0]->Fill(local_point.x(), local_point.y());
        meHitXlocal_[0]->Fill(local_point.x());
        meHitYlocal_[0]->Fill(local_point.y());

      } else if ((idet == 2) || (idet == 3)) {
        meLocalOccupancy_[1]->Fill(local_point.x(), local_point.y());
        meHitXlocal_[1]->Fill(local_point.x());
        meHitYlocal_[1]->Fill(local_point.y());
      }
    }

    meHitX_[idet]->Fill(global_point.x());
    meHitY_[idet]->Fill(global_point.y());
    meHitZ_[idet]->Fill(global_point.z());
    meHitPhi_[idet]->Fill(global_point.phi());
    meHitEta_[idet]->Fill(global_point.eta());

    meHitTvsQ_[idet]->Fill(adc, toa);
    meHitToTvsQ_[idet]->Fill(adc, tot);
    meHitCALvsQ_[idet]->Fill(adc, cal);
    meHitQvsPhi_[idet]->Fill(global_point.phi(), adc);
    meHitQvsEta_[idet]->Fill(global_point.eta(), adc);
    meHitTvsPhi_[idet]->Fill(global_point.phi(), toa);
    meHitTvsEta_[idet]->Fill(global_point.eta(), toa);
    meHitToTvsPhi_[idet]->Fill(global_point.phi(), tot);
    meHitToTvsEta_[idet]->Fill(global_point.eta(), tot);
    meHitCALvsPhi_[idet]->Fill(global_point.phi(), cal);
    meHitCALvsEta_[idet]->Fill(global_point.eta(), cal);

        n_digi_etl[idet]++;
    size_t ncount(0);
    ndigiPerLGAD_[idet].emplace(detId.rawId(), ncount);
    ndigiPerLGAD_[idet].at(detId.rawId())++;

    // --- Occupancy study for different thresholds on Q
    double bin_w_Q = (Q_Max - Q_Min) / n_bin_Q;
    // Initialize the Map Entry (if first hit for this LGAD)
    // The array must be initialized to all zeros.
    std::array<uint32_t, n_bin_Q> zero_counts_Q{};  // Array of N_bin_q zeros
    ndigiPerLGADoverQ_[idet].emplace(detId.rawId(), zero_counts_Q);
    // Increment the Appropriate Counters
    auto& threshold_counters = ndigiPerLGADoverQ_[idet].at(detId.rawId());
    for (int i = 0; i < n_bin_Q; i++) {
      // Calculate the lower bound of the charge bin, which acts as the threshold
      double th_Q = Q_Min + i * bin_w_Q;
      // If the hit charge is greater than this threshold, increment the counter for this bin
      if (adc > th_Q) {
        threshold_counters[i]++;
      }
    }
    // --- Occupancy study for different Eta bins
    std::array<uint32_t, n_bin_Eta> zero_counts_Eta{};
    ndigiPerLGADoverEta_[idet].emplace(detId.rawId(), zero_counts_Eta);
    auto& Eta_counters = ndigiPerLGADoverEta_[idet].at(detId.rawId());
    for (int i = 0; i < n_bin_Eta; i++) {
      double lower_edge = ((idet == 0) || (idet == 1)) ? eta_bins_edges_neg[i] : eta_bins_edges_pos[i];
      double upper_edge = ((idet == 0) || (idet == 1)) ? eta_bins_edges_neg[i + 1] : eta_bins_edges_pos[i + 1];
      if (global_point.eta() >= lower_edge && global_point.eta() < upper_edge) {
        Eta_counters[i]++;
      }
    }

  }  // dataFrame loop

  for (int i = 0; i < 4; i++) {
    meNhits_[i]->Fill(log10(n_digi_etl[i]));
    for (const auto& thisNdigi : ndigiPerLGAD_[i]) {
      meNhitsPerLGAD_[i]->Fill(thisNdigi.second);
    }
    // Number of LGADs with at least one hit.
    meNLgadWithHits_[i]->Fill(ndigiPerLGAD_[i].size());
  }

  // --- Occupancy study for different thresholds on Q
  double bin_w_Q = (Q_Max - Q_Min) / n_bin_Q;
  for (int i = 0; i < 4; i++) {  // Loop over the 4 ETL regions
    // For each threshold bin (x-axis of the profile)
    for (int j = 0; j < n_bin_Q; j++) {
      double Q_value = Q_Min + j * bin_w_Q + (bin_w_Q / 2.);  // Center of the threshold bin
      double total_n_hits_for_this_threshold = 0.;
      // Variable to count LGADs with at least one hit above threshold j
      size_t n_lgads_with_hits_for_this_threshold = 0;
      // Sum the counts from all LGADs for the current threshold 'j'
      for (const auto& entry : ndigiPerLGADoverQ_[i]) {
        total_n_hits_for_this_threshold += entry.second[j];
        // Check if the total hits for this LGAD at this specific threshold is > 0
        if (entry.second[j] > 0) {
          n_lgads_with_hits_for_this_threshold++;
        }
      }
      // Calculate the average number of hits per LGAD
      double average_n_hits = 0.;
      if (n_lgads_with_hits_for_this_threshold > 0) {
        average_n_hits = total_n_hits_for_this_threshold / static_cast<double>(n_lgads_with_hits_for_this_threshold);
      }
      // Fill the profile with the AVERAGE N_hits found at this threshold
      meNhitsPerLGADoverQ_[i]->Fill(Q_value, average_n_hits);
      // Fill the profile with average Number of LGADs with Hits per Event vs Q Threshold
      meNLgadWithHitsoverQ_[i]->Fill(Q_value, n_lgads_with_hits_for_this_threshold);
    }
  }
  // --- Occupancy study for different bins on Eta
  for (int i = 0; i < 4; i++) {  // Loop over the 4 ETL regions
    for (int j = 0; j < n_bin_Eta; j++) {
      double eta_low = ((i == 0) || (i == 1)) ? eta_bins_edges_neg[j] : eta_bins_edges_pos[j];
      double eta_high = ((i == 0) || (i == 1)) ? eta_bins_edges_neg[j + 1] : eta_bins_edges_pos[j + 1];
      double eta_value = (eta_low + eta_high) / 2.;  // Center of the Eta bin
      double total_n_hits_for_this_eta_bin = 0.;
      size_t n_lgads_with_hits_for_this_eta_bin = 0;
      for (const auto& entry : ndigiPerLGADoverEta_[i]) {
        total_n_hits_for_this_eta_bin += entry.second[j];
        if (entry.second[j] > 0) {
          n_lgads_with_hits_for_this_eta_bin++;
        }
      }
      double average_n_hits = 0.;
      if (n_lgads_with_hits_for_this_eta_bin > 0) {
        average_n_hits = total_n_hits_for_this_eta_bin / static_cast<double>(n_lgads_with_hits_for_this_eta_bin);
      }
      meNhitsPerLGADoverEta_[i]->Fill(eta_value, average_n_hits);
      meNLgadWithHitsoverEta_[i]->Fill(eta_value, n_lgads_with_hits_for_this_eta_bin);
    }
  }
}

// ------------ method for histogram booking ------------
void EtlDigiSoAHitsValidation::bookHistograms(DQMStore::IBooker& ibook,
                                              edm::Run const& run,
                                              edm::EventSetup const& iSetup) {
  ibook.setCurrentFolder(folder_);

  // --- histograms booking

  meNhits_[0] = ibook.book1D("EtlNhitsZnegD1",
                             "Number of ETL DIGI hits (-Z, Single(topo1D)/First(topo2D) disk);log_{10}(N_{DIGI})",
                             100,
                             0.,
                             5.25);
  meNhits_[1] =
      ibook.book1D("EtlNhitsZnegD2", "Number of ETL DIGI hits (-Z, Second disk);log_{10}(N_{DIGI})", 100, 0., 5.25);
  meNhits_[2] = ibook.book1D("EtlNhitsZposD1",
                             "Number of ETL DIGI hits (+Z, Single(topo1D)/First(topo2D) disk);log_{10}(N_{DIGI})",
                             100,
                             0.,
                             5.25);
  meNhits_[3] =
      ibook.book1D("EtlNhitsZposD2", "Number of ETL DIGI hits (+Z, Second disk);log_{10}(N_{DIGI})", 100, 0., 5.25);

  meNhitsPerLGAD_[0] = ibook.book1D("EtlNhitsPerLGADZnegD1",
                                    "Number of ETL DIGI hits (-Z, Single(topo1D)/First(topo2D) disk) per LGAD;N_{DIGI}",
                                    20,
                                    0.,
                                    20.);
  meNhitsPerLGAD_[1] =
      ibook.book1D("EtlNhitsPerLGADZnegD2", "Number of ETL DIGI hits (-Z, Second disk) per LGAD;N_{DIGI}", 20, 0., 20.);
  meNhitsPerLGAD_[2] = ibook.book1D("EtlNhitsPerLGADZposD1",
                                    "Number of ETL DIGI hits (+Z, Single(topo1D)/First(topo2D) disk) per LGAD;N_{DIGI}",
                                    20,
                                    0.,
                                    20.);
  meNhitsPerLGAD_[3] =
      ibook.book1D("EtlNhitsPerLGADZposD2", "Number of ETL DIGI hits (+Z, Second disk) per LGAD;N_{DIGI}", 20, 0., 20.);

  meNLgadWithHits_[0] = ibook.book1D("EtlNLgadWithHitsZnegD1",
                                     "Number of ETL LGADs with at least 1 DIGI hit (-Z, D1);N_{LGAD with hit}",
                                     50,
                                     0.,
                                     4000.);
  meNLgadWithHits_[1] = ibook.book1D("EtlNLgadWithHitsZnegD2",
                                     "Number of ETL LGADs with at least 1 DIGI hit (-Z, D2);N_{LGAD with hit}",
                                     50,
                                     0.,
                                     4000.);
  meNLgadWithHits_[2] = ibook.book1D("EtlNLgadWithHitsZposD1",
                                     "Number of ETL LGADs with at least 1 DIGI hit (+Z, D1);N_{LGAD with hit}",
                                     50,
                                     0.,
                                     4000.);
  meNLgadWithHits_[3] = ibook.book1D("EtlNLgadWithHitsZposD2",
                                     "Number of ETL LGADs with at least 1 DIGI hit (+Z, D2);N_{LGAD with hit}",
                                     50,
                                     0.,
                                     4000.);

  meNhitsPerLGADoverQ_[0] =
      ibook.bookProfile("EtlNhitsPerLGADvsQThZnegD1",
                        "ETL DIGI Hits per LGAD vs Q Threshold (-Z, D1);Q Threshold [ADC counts];<N_{DIGI} per LGAD>",
                        n_bin_Q,
                        Q_Min,
                        Q_Max,
                        0.,
                        20.);
  meNhitsPerLGADoverQ_[1] =
      ibook.bookProfile("EtlNhitsPerLGADvsQThZnegD2",
                        "ETL DIGI Hits per LGAD vs Q Threshold (-Z, D2);Q Threshold [ADC counts];<N_{DIGI} per LGAD>",
                        n_bin_Q,
                        Q_Min,
                        Q_Max,
                        0.,
                        20.);
  meNhitsPerLGADoverQ_[2] =
      ibook.bookProfile("EtlNhitsPerLGADvsQThZposD1",
                        "ETL DIGI Hits per LGAD vs Q Threshold (+Z, D1);Q Threshold [ADC counts];<N_{DIGI} per LGAD>",
                        n_bin_Q,
                        Q_Min,
                        Q_Max,
                        0.,
                        20.);
  meNhitsPerLGADoverQ_[3] =
      ibook.bookProfile("EtlNhitsPerLGADvsQThZposD2",
                        "ETL DIGI Hits per LGAD vs Q Threshold (+Z, D2);Q Threshold [ADC counts];<N_{DIGI} per LGAD>",
                        n_bin_Q,
                        Q_Min,
                        Q_Max,
                        0.,
                        20.);

  meNLgadWithHitsoverQ_[0] = ibook.bookProfile(
      "EtlNLgadWithHitsvsQThZnegD1",
      "Number of ETL LGADs with at least 1 DIGI hit vs Q Threshold (-Z, D1);Q Threshold [ADC counts];N_{LGAD with hit}",
      n_bin_Q,
      Q_Min,
      Q_Max,
      0.,
      4000.);
  meNLgadWithHitsoverQ_[1] = ibook.bookProfile(
      "EtlNLgadWithHitsvsQThZnegD2",
      "Number of ETL LGADs with at least 1 DIGI hit vs Q Threshold (-Z, D2);Q Threshold [ADC counts];N_{LGAD with hit}",
      n_bin_Q,
      Q_Min,
      Q_Max,
      0.,
      4000.);
  meNLgadWithHitsoverQ_[2] = ibook.bookProfile(
      "EtlNLgadWithHitsvsQThZposD1",
      "Number of ETL LGADs with at least 1 DIGI hit vs Q Threshold (+Z, D1);Q Threshold [ADC counts];N_{LGAD with hit}",
      n_bin_Q,
      Q_Min,
      Q_Max,
      0.,
      4000.);
  meNLgadWithHitsoverQ_[3] = ibook.bookProfile(
      "EtlNLgadWithHitsvsQThZposD2",
      "Number of ETL LGADs with at least 1 DIGI hit vs Q Threshold (+Z, D2);Q Threshold [ADC counts];N_{LGAD with hit}",
      n_bin_Q,
      Q_Min,
      Q_Max,
      0.,
      4000.);

  meNhitsPerLGADoverEta_[0] =
      ibook.bookProfile("EtlNhitsPerLGADvsEtaZnegD1",
                        "ETL DIGI Hits per LGAD vs Eta Bin (-Z, D1);#eta_{DIGI};<N_{DIGI} per LGAD>",
                        n_bin_Eta,
                        eta_bins_edges_neg,
                        0.,
                        20.);
  meNhitsPerLGADoverEta_[1] =
      ibook.bookProfile("EtlNhitsPerLGADvsEtaZnegD2",
                        "ETL DIGI Hits per LGAD vs Eta Bin (-Z, D2);#eta_{DIGI};<N_{DIGI} per LGAD>",
                        n_bin_Eta,
                        eta_bins_edges_neg,
                        0.,
                        20.);
  meNhitsPerLGADoverEta_[2] =
      ibook.bookProfile("EtlNhitsPerLGADvsEtaZposD1",
                        "ETL DIGI Hits per LGAD vs Eta Bin (+Z, D1);#eta_{DIGI};<N_{DIGI} per LGAD>",
                        n_bin_Eta,
                        eta_bins_edges_pos,
                        0.,
                        20.);
  meNhitsPerLGADoverEta_[3] =
      ibook.bookProfile("EtlNhitsPerLGADvsEtaZposD2",
                        "ETL DIGI Hits per LGAD vs Eta Bin (+Z, D2);#eta_{DIGI};<N_{DIGI} per LGAD>",
                        n_bin_Eta,
                        eta_bins_edges_pos,
                        0.,
                        20.);

  meNLgadWithHitsoverEta_[0] = ibook.bookProfile(
      "EtlNLgadWithHitsvsEtaZnegD1",
      "Number of ETL LGADs with at least 1 DIGI hit vs Eta Bin (-Z, D1);#eta_{DIGI};N_{LGAD with hit}",
      n_bin_Eta,
      eta_bins_edges_neg,
      0.,
      4000.);
  meNLgadWithHitsoverEta_[1] = ibook.bookProfile(
      "EtlNLgadWithHitsvsEtaZnegD2",
      "Number of ETL LGADs with at least 1 DIGI hit vs Eta Bin (-Z, D2);#eta_{DIGI};N_{LGAD with hit}",
      n_bin_Eta,
      eta_bins_edges_neg,
      0.,
      4000.);
  meNLgadWithHitsoverEta_[2] = ibook.bookProfile(
      "EtlNLgadWithHitsvsEtaZposD1",
      "Number of ETL LGADs with at least 1 DIGI hit vs Eta Bin (+Z, D1);#eta_{DIGI};N_{LGAD with hit}",
      n_bin_Eta,
      eta_bins_edges_pos,
      0.,
      4000.);
  meNLgadWithHitsoverEta_[3] = ibook.bookProfile(
      "EtlNLgadWithHitsvsEtaZposD2",
      "Number of ETL LGADs with at least 1 DIGI hit vs Eta Bin (+Z, D2);#eta_{DIGI};N_{LGAD with hit}",
      n_bin_Eta,
      eta_bins_edges_pos,
      0.,
      4000.);

  meHitCharge_[0] = ibook.book1D("EtlHitChargeZnegD1",
                                 "ETL DIGI hits charge (-Z, Single(topo1D)/First(topo2D) disk);Q_{DIGI} [ADC counts]",
                                 100,
                                 0.,
                                 256.);
  meHitCharge_[1] =
      ibook.book1D("EtlHitChargeZnegD2", "ETL DIGI hits charge (-Z, Second disk);Q_{DIGI} [ADC counts]", 100, 0., 256.);
  meHitCharge_[2] = ibook.book1D("EtlHitChargeZposD1",
                                 "ETL DIGI hits charge (+Z, Single(topo1D)/First(topo2D) disk);Q_{DIGI} [ADC counts]",
                                 100,
                                 0.,
                                 256.);
  meHitCharge_[3] =
      ibook.book1D("EtlHitChargeZposD2", "ETL DIGI hits charge (+Z, Second disk);Q_{DIGI} [ADC counts]", 100, 0., 256.);

  meHitTime_[0] = ibook.book1D("EtlHitTimeZnegD1",
                               "ETL DIGI hits ToA (-Z, Single(topo1D)/First(topo2D) disk);ToA_{DIGI} [TDC counts]",
                               100,
                               0.,
                               2000.);
  meHitTime_[1] =
      ibook.book1D("EtlHitTimeZnegD2", "ETL DIGI hits ToA (-Z, Second disk);ToA_{DIGI} [TDC counts]", 100, 0., 2000.);
  meHitTime_[2] = ibook.book1D("EtlHitTimeZposD1",
                               "ETL DIGI hits ToA (+Z, Single(topo1D)/First(topo2D) disk);ToA_{DIGI} [TDC counts]",
                               100,
                               0.,
                               2000.);
  meHitTime_[3] =
      ibook.book1D("EtlHitTimeZposD2", "ETL DIGI hits ToA (+Z, Second disk);ToA_{DIGI} [TDC counts]", 100, 0., 2000.);

  meHitToT_[0] = ibook.book1D("EtlHitToTZnegD1",
                              "ETL DIGI hits ToT (-Z, Single(topo1D)/First(topo2D) disk);ToT_{DIGI} [TDC counts]",
                              100,
                              0.,
                              500.);
  meHitToT_[1] =
      ibook.book1D("EtlHitToTZnegD2", "ETL DIGI hits ToT (-Z, Second disk);ToT_{DIGI} [TDC counts]", 100, 0., 500.);
  meHitToT_[2] = ibook.book1D("EtlHitToTZposD1",
                              "ETL DIGI hits ToT (+Z, Single(topo1D)/First(topo2D) disk);ToT_{DIGI} [TDC counts]",
                              100,
                              0.,
                              500.);
  meHitToT_[3] =
      ibook.book1D("EtlHitToTZposD2", "ETL DIGI hits ToT (+Z, Second disk);ToT_{DIGI} [TDC counts]", 100, 0., 500.);

  meHitCAL_[0] = ibook.book1D("EtlHitCALZnegD1",
                              "ETL DIGI hits CAL (-Z, Single(topo1D)/First(topo2D) disk);CAL_{DIGI}",
                              100,
                              0.,
                              500.);
  meHitCAL_[1] =
      ibook.book1D("EtlHitCALZnegD2", "ETL DIGI hits CAL (-Z, Second disk);CAL_{DIGI}", 100, 0., 500.);
  meHitCAL_[2] = ibook.book1D("EtlHitCALZposD1",
                              "ETL DIGI hits CAL (+Z, Single(topo1D)/First(topo2D) disk);CAL_{DIGI}",
                              100,
                              0.,
                              500.);
  meHitCAL_[3] =
      ibook.book1D("EtlHitCALZposD2", "ETL DIGI hits CAL (+Z, Second disk);CAL_{DIGI}", 100, 0., 500.);

  meOccupancy_[0] =
      ibook.book2D("EtlOccupancyZnegD1",
                   "ETL DIGI hits occupancy (-Z, Single(topo1D)/First(topo2D) disk);X_{DIGI} [cm];Y_{DIGI} [cm]",
                   135,
                   -135.,
                   135.,
                   135,
                   -135.,
                   135.);
  meOccupancy_[1] = ibook.book2D("EtlOccupancyZnegD2",
                                 "ETL DIGI hits occupancy (-Z, Second disk);X_{DIGI} [cm];Y_{DIGI} [cm]",
                                 135,
                                 -135.,
                                 135.,
                                 135,
                                 -135.,
                                 135.);
  meOccupancy_[2] =
      ibook.book2D("EtlOccupancyZposD1",
                   "ETL DIGI hits occupancy (+Z, Single(topo1D)/First(topo2D) disk);X_{DIGI} [cm];Y_{DIGI} [cm]",
                   135,
                   -135.,
                   135.,
                   135,
                   -135.,
                   135.);
  meOccupancy_[3] = ibook.book2D("EtlOccupancyZposD2",
                                 "ETL DIGI hits occupancy (+Z, Second disk);X_{DIGI} [cm];Y_{DIGI} [cm]",
                                 135,
                                 -135.,
                                 135.,
                                 135,
                                 -135.,
                                 135.);
  if (optionalPlots_) {
    meLocalOccupancy_[0] = ibook.book2D("EtlLocalOccupancyZneg",
                                        "ETL DIGI hits local occupancy (-Z);X_{DIGI} [cm];Y_{DIGI} [cm]",
                                        100,
                                        -2.2,
                                        2.2,
                                        50,
                                        -1.1,
                                        1.1);
    meLocalOccupancy_[1] = ibook.book2D("EtlLocalOccupancyZpos",
                                        "ETL DIGI hits local occupancy (+Z);X_{DIGI} [cm];Y_{DIGI} [cm]",
                                        100,
                                        -2.2,
                                        2.2,
                                        50,
                                        -1.1,
                                        1.1);
    meHitXlocal_[0] = ibook.book1D("EtlHitXlocalZneg", "ETL DIGI local X (-Z);X_{DIGI}^{LOC} [cm]", 100, -2.2, 2.2);
    meHitXlocal_[1] = ibook.book1D("EtlHitXlocalZpos", "ETL DIGI local X (+Z);X_{DIGI}^{LOC} [cm]", 100, -2.2, 2.2);
    meHitYlocal_[0] = ibook.book1D("EtlHitYlocalZneg", "ETL DIGI local Y (-Z);Y_{DIGI}^{LOC} [cm]", 50, -1.1, 1.1);
    meHitYlocal_[1] = ibook.book1D("EtlHitYlocalZpos", "ETL DIGI local Y (-Z);Y_{DIGI}^{LOC} [cm]", 50, -1.1, 1.1);
  }
  meHitX_[0] = ibook.book1D(
      "EtlHitXZnegD1", "ETL DIGI hits X (-Z, Single(topo1D)/First(topo2D) disk);X_{DIGI} [cm]", 100, -130., 130.);
  meHitX_[1] = ibook.book1D("EtlHitXZnegD2", "ETL DIGI hits X (-Z, Second disk);X_{DIGI} [cm]", 100, -130., 130.);
  meHitX_[2] = ibook.book1D(
      "EtlHitXZposD1", "ETL DIGI hits X (+Z, Single(topo1D)/First(topo2D) disk);X_{DIGI} [cm]", 100, -130., 130.);
  meHitX_[3] = ibook.book1D("EtlHitXZposD2", "ETL DIGI hits X (+Z, Second disk);X_{DIGI} [cm]", 100, -130., 130.);
  meHitY_[0] = ibook.book1D(
      "EtlHitYZnegD1", "ETL DIGI hits Y (-Z, Single(topo1D)/First(topo2D) disk);Y_{DIGI} [cm]", 100, -130., 130.);
  meHitY_[1] = ibook.book1D("EtlHitYZnegD2", "ETL DIGI hits Y (-Z, Second disk);Y_{DIGI} [cm]", 100, -130., 130.);
  meHitY_[2] = ibook.book1D(
      "EtlHitYZposD1", "ETL DIGI hits Y (+Z, Single(topo1D)/First(topo2D) disk);Y_{DIGI} [cm]", 100, -130., 130.);
  meHitY_[3] = ibook.book1D("EtlHitYZposD2", "ETL DIGI hits Y (+Z, Second disk);Y_{DIGI} [cm]", 100, -130., 130.);
  meHitZ_[0] = ibook.book1D(
      "EtlHitZZnegD1", "ETL DIGI hits Z (-Z, Single(topo1D)/First(topo2D) disk);Z_{DIGI} [cm]", 100, -302., -298.);
  meHitZ_[1] = ibook.book1D("EtlHitZZnegD2", "ETL DIGI hits Z (-Z, Second disk);Z_{DIGI} [cm]", 100, -304., -300.);
  meHitZ_[2] = ibook.book1D(
      "EtlHitZZposD1", "ETL DIGI hits Z (+Z, Single(topo1D)/First(topo2D) disk);Z_{DIGI} [cm]", 100, 298., 302.);
  meHitZ_[3] = ibook.book1D("EtlHitZZposD2", "ETL DIGI hits Z (+Z, Second disk);Z_{DIGI} [cm]", 100, 300., 304.);

  meHitPhi_[0] = ibook.book1D("EtlHitPhiZnegD1",
                              "ETL DIGI hits #phi (-Z, Single(topo1D)/First(topo2D) disk);#phi_{DIGI} [rad]",
                              100,
                              -3.15,
                              3.15);
  meHitPhi_[1] =
      ibook.book1D("EtlHitPhiZnegD2", "ETL DIGI hits #phi (-Z, Second disk);#phi_{DIGI} [rad]", 100, -3.15, 3.15);
  meHitPhi_[2] = ibook.book1D("EtlHitPhiZposD1",
                              "ETL DIGI hits #phi (+Z, Single(topo1D)/First(topo2D) disk);#phi_{DIGI} [rad]",
                              100,
                              -3.15,
                              3.15);
  meHitPhi_[3] =
      ibook.book1D("EtlHitPhiZposD2", "ETL DIGI hits #phi (+Z, Second disk);#phi_{DIGI} [rad]", 100, -3.15, 3.15);
  meHitEta_[0] = ibook.book1D(
      "EtlHitEtaZnegD1", "ETL DIGI hits #eta (-Z, Single(topo1D)/First(topo2D) disk);#eta_{DIGI}", 100, -3.2, -1.56);
  meHitEta_[1] = ibook.book1D("EtlHitEtaZnegD2", "ETL DIGI hits #eta (-Z, Second disk);#eta_{DIGI}", 100, -3.2, -1.56);
  meHitEta_[2] = ibook.book1D(
      "EtlHitEtaZposD1", "ETL DIGI hits #eta (+Z, Single(topo1D)/First(topo2D) disk);#eta_{DIGI}", 100, 1.56, 3.2);
  meHitEta_[3] = ibook.book1D("EtlHitEtaZposD2", "ETL DIGI hits #eta (+Z, Second disk);#eta_{DIGI}", 100, 1.56, 3.2);
  meHitTvsQ_[0] = ibook.bookProfile(
      "EtlHitTvsQZnegD1",
      "ETL DIGI ToA vs charge (-Z, Single(topo1D)/First(topo2D) disk);Q_{DIGI} [ADC counts];ToA_{DIGI} [TDC counts]",
      50,
      0.,
      256.,
      0.,
      2048.);
  meHitTvsQ_[1] =
      ibook.bookProfile("EtlHitTvsQZnegD2",
                        "ETL DIGI ToA vs charge (-Z, Second Disk);Q_{DIGI} [ADC counts];ToA_{DIGI} [TDC counts]",
                        50,
                        0.,
                        256.,
                        0.,
                        2048.);
  meHitTvsQ_[2] = ibook.bookProfile(
      "EtlHitTvsQZposD1",
      "ETL DIGI ToA vs charge (+Z, Single(topo1D)/First(topo2D) disk);Q_{DIGI} [ADC counts];ToA_{DIGI} [TDC counts]",
      50,
      0.,
      256.,
      0.,
      2048.);
  meHitTvsQ_[3] =
      ibook.bookProfile("EtlHitTvsQZposD2",
                        "ETL DIGI ToA vs charge (+Z, Second disk);Q_{DIGI} [ADC counts];ToA_{DIGI} [TDC counts]",
                        50,
                        0.,
                        256.,
                        0.,
                        2048.);
  meHitToTvsQ_[0] = ibook.bookProfile(
      "EtlHitToTvsQZnegD1",
      "ETL DIGI ToT vs charge (-Z, Single(topo1D)/First(topo2D) disk);Q_{DIGI} [ADC counts];ToT_{DIGI} [TDC counts]",
      50,
      0.,
      256.,
      0.,
      2048.);
  meHitToTvsQ_[1] =
      ibook.bookProfile("EtlHitToTvsQZnegD2",
                        "ETL DIGI ToT vs charge (-Z, Second Disk);Q_{DIGI} [ADC counts];ToT_{DIGI} [TDC counts]",
                        50,
                        0.,
                        256.,
                        0.,
                        2048.);
  meHitToTvsQ_[2] = ibook.bookProfile(
      "EtlHitToTvsQZposD1",
      "ETL DIGI ToT vs charge (+Z, Single(topo1D)/First(topo2D) disk);Q_{DIGI} [ADC counts];ToT_{DIGI} [TDC counts]",
      50,
      0.,
      256.,
      0.,
      2048.);
  meHitToTvsQ_[3] =
      ibook.bookProfile("EtlHitToTvsQZposD2",
                        "ETL DIGI ToT vs charge (+Z, Second disk);Q_{DIGI} [ADC counts];ToT_{DIGI} [TDC counts]",
                        50,
                        0.,
                        256.,
                        0.,
                        2048.);
  meHitCALvsQ_[0] = ibook.bookProfile(
      "EtlHitCALvsQZnegD1",
      "ETL DIGI CAL vs charge (-Z, Single(topo1D)/First(topo2D) disk);Q_{DIGI} [ADC counts];CAL_{DIGI}",
      50,
      0.,
      256.,
      0.,
      2048.);
  meHitCALvsQ_[1] =
      ibook.bookProfile("EtlHitCALvsQZnegD2",
                        "ETL DIGI CAL vs charge (-Z, Second Disk);Q_{DIGI} [ADC counts];CAL_{DIGI}",
                        50,
                        0.,
                        256.,
                        0.,
                        2048.);
  meHitCALvsQ_[2] = ibook.bookProfile(
      "EtlHitCALvsQZposD1",
      "ETL DIGI CAL vs charge (+Z, Single(topo1D)/First(topo2D) disk);Q_{DIGI} [ADC counts];CAL_{DIGI}",
      50,
      0.,
      256.,
      0.,
      2048.);
  meHitCALvsQ_[3] =
      ibook.bookProfile("EtlHitCALvsQZposD2",
                        "ETL DIGI CAL vs charge (+Z, Second disk);Q_{DIGI} [ADC counts];CAL_{DIGI}",
                        50,
                        0.,
                        256.,
                        0.,
                        2048.);
  meHitQvsPhi_[0] = ibook.bookProfile(
      "EtlHitQvsPhiZnegD1",
      "ETL DIGI charge vs #phi (-Z, Single(topo1D)/First(topo2D) disk);#phi_{DIGI} [rad];Q_{DIGI} [ADC counts]",
      50,
      -3.15,
      3.15,
      0.,
      1024.);
  meHitQvsPhi_[1] =
      ibook.bookProfile("EtlHitQvsPhiZnegD2",
                        "ETL DIGI charge vs #phi (-Z, Second disk);#phi_{DIGI} [rad];Q_{DIGI} [ADC counts]",
                        50,
                        -3.15,
                        3.15,
                        0.,
                        1024.);
  meHitQvsPhi_[2] = ibook.bookProfile(
      "EtlHitQvsPhiZposD1",
      "ETL DIGI charge vs #phi (+Z, Single(topo1D)/First(topo2D) disk);#phi_{DIGI} [rad];Q_{DIGI} [ADC counts]",
      50,
      -3.15,
      3.15,
      0.,
      1024.);
  meHitQvsPhi_[3] =
      ibook.bookProfile("EtlHitQvsPhiZposD2",
                        "ETL DIGI charge vs #phi (+Z, Second disk);#phi_{DIGI} [rad];Q_{DIGI} [ADC counts]",
                        50,
                        -3.15,
                        3.15,
                        0.,
                        1024.);
  meHitQvsEta_[0] = ibook.bookProfile(
      "EtlHitQvsEtaZnegD1",
      "ETL DIGI charge vs #eta (-Z, Single(topo1D)/First(topo2D) disk);#eta_{DIGI};Q_{DIGI} [ADC counts]",
      50,
      -3.2,
      -1.56,
      0.,
      1024.);
  meHitQvsEta_[1] = ibook.bookProfile("EtlHitQvsEtaZnegD2",
                                      "ETL DIGI charge vs #eta (-Z, Second disk);#eta_{DIGI};Q_{DIGI} [ADC counts]",
                                      50,
                                      -3.2,
                                      -1.56,
                                      0.,
                                      1024.);
  meHitQvsEta_[2] = ibook.bookProfile(
      "EtlHitQvsEtaZposD1",
      "ETL DIGI charge vs #eta (+Z, Single(topo1D)/First(topo2D) disk);#eta_{DIGI};Q_{DIGI} [ADC counts]",
      50,
      1.56,
      3.2,
      0.,
      1024.);
  meHitQvsEta_[3] = ibook.bookProfile("EtlHitQvsEtaZposD2",
                                      "ETL DIGI charge vs #eta (+Z, Second disk);#eta_{DIGI};Q_{DIGI} [ADC counts]",
                                      50,
                                      1.56,
                                      3.2,
                                      0.,
                                      1024.);
  meHitTvsPhi_[0] = ibook.bookProfile(
      "EtlHitTvsPhiZnegD1",
      "ETL DIGI ToA vs #phi (-Z, Single(topo1D)/First(topo2D) disk);#phi_{DIGI} [rad];ToA_{DIGI} [TDC counts]",
      50,
      -3.15,
      3.15,
      0.,
      2048.);
  meHitTvsPhi_[1] =
      ibook.bookProfile("EtlHitTvsPhiZnegD2",
                        "ETL DIGI ToA vs #phi (-Z, Second disk);#phi_{DIGI} [rad];ToA_{DIGI} [TDC counts]",
                        50,
                        -3.15,
                        3.15,
                        0.,
                        2048.);
  meHitTvsPhi_[2] = ibook.bookProfile(
      "EtlHitTvsPhiZposD1",
      "ETL DIGI ToA vs #phi (+Z, Single(topo1D)/First(topo2D) disk);#phi_{DIGI} [rad];ToA_{DIGI} [TDC counts]",
      50,
      -3.15,
      3.15,
      0.,
      2048.);
  meHitTvsPhi_[3] =
      ibook.bookProfile("EtlHitTvsPhiZposD2",
                        "ETL DIGI ToA vs #phi (+Z, Second disk);#phi_{DIGI} [rad];ToA_{DIGI} [TDC counts]",
                        50,
                        -3.15,
                        3.15,
                        0.,
                        2048.);
  meHitTvsEta_[0] = ibook.bookProfile(
      "EtlHitTvsEtaZnegD1",
      "ETL DIGI ToA vs #eta (-Z, Single(topo1D)/First(topo2D) disk);#eta_{DIGI};ToA_{DIGI} [TDC counts]",
      50,
      -3.2,
      -1.56,
      0.,
      2048.);
  meHitTvsEta_[1] = ibook.bookProfile("EtlHitTvsEtaZnegD2",
                                      "ETL DIGI ToA vs #eta (-Z, Second disk);#eta_{DIGI};ToA_{DIGI} [TDC counts]",
                                      50,
                                      -3.2,
                                      -1.56,
                                      0.,
                                      2048.);
  meHitTvsEta_[2] = ibook.bookProfile(
      "EtlHitTvsEtaZposD1",
      "ETL DIGI ToA vs #eta (+Z, Single(topo1D)/First(topo2D) disk);#eta_{DIGI};ToA_{DIGI} [TDC counts]",
      50,
      1.56,
      3.2,
      0.,
      2048.);
  meHitTvsEta_[3] = ibook.bookProfile("EtlHitTvsEtaZposD2",
                                      "ETL DIGI ToA vs #eta (+Z, Second disk);#eta_{DIGI};ToA_{DIGI} [TDC counts]",
                                      50,
                                      1.56,
                                      3.2,
                                      0.,
                                      2048.);
  meHitToTvsPhi_[0] = ibook.bookProfile(
      "EtlHitToTvsPhiZnegD1",
      "ETL DIGI ToT vs #phi (-Z, Single(topo1D)/First(topo2D) disk);#phi_{DIGI} [rad];ToT_{DIGI} [TDC counts]",
      50,
      -3.15,
      3.15,
      0.,
      2048.);
  meHitToTvsPhi_[1] =
      ibook.bookProfile("EtlHitToTvsPhiZnegD2",
                        "ETL DIGI ToT vs #phi (-Z, Second disk);#phi_{DIGI} [rad];ToT_{DIGI} [TDC counts]",
                        50,
                        -3.15,
                        3.15,
                        0.,
                        2048.);
  meHitToTvsPhi_[2] = ibook.bookProfile(
      "EtlHitToTvsPhiZposD1",
      "ETL DIGI ToT vs #phi (+Z, Single(topo1D)/First(topo2D) disk);#phi_{DIGI} [rad];ToT_{DIGI} [TDC counts]",
      50,
      -3.15,
      3.15,
      0.,
      2048.);
  meHitToTvsPhi_[3] =
      ibook.bookProfile("EtlHitToTvsPhiZposD2",
                        "ETL DIGI ToT vs #phi (+Z, Second disk);#phi_{DIGI} [rad];ToT_{DIGI} [TDC counts]",
                        50,
                        -3.15,
                        3.15,
                        0.,
                        2048.);
  meHitToTvsEta_[0] = ibook.bookProfile(
      "EtlHitToTvsEtaZnegD1",
      "ETL DIGI ToT vs #eta (-Z, Single(topo1D)/First(topo2D) disk);#eta_{DIGI};ToT_{DIGI} [TDC counts]",
      50,
      -3.2,
      -1.56,
      0.,
      2048.);
  meHitToTvsEta_[1] = ibook.bookProfile("EtlHitToTvsEtaZnegD2",
                                      "ETL DIGI ToT vs #eta (-Z, Second disk);#eta_{DIGI};ToT_{DIGI} [TDC counts]",
                                      50,
                                      -3.2,
                                      -1.56,
                                      0.,
                                      2048.);
  meHitToTvsEta_[2] = ibook.bookProfile(
      "EtlHitToTvsEtaZposD1",
      "ETL DIGI ToT vs #eta (+Z, Single(topo1D)/First(topo2D) disk);#eta_{DIGI};ToT_{DIGI} [TDC counts]",
      50,
      1.56,
      3.2,
      0.,
      2048.);
  meHitToTvsEta_[3] = ibook.bookProfile("EtlHitToTvsEtaZposD2",
                                      "ETL DIGI ToT vs #eta (+Z, Second disk);#eta_{DIGI};ToT_{DIGI} [TDC counts]",
                                      50,
                                      1.56,
                                      3.2,
                                      0.,
                                      2048.);
  meHitCALvsPhi_[0] = ibook.bookProfile(
      "EtlHitCALvsPhiZnegD1",
      "ETL DIGI CAL vs #phi (-Z, Single(topo1D)/First(topo2D) disk);#phi_{DIGI} [rad];CAL_{DIGI}",
      50,
      -3.15,
      3.15,
      0.,
      2048.);
  meHitCALvsPhi_[1] =
      ibook.bookProfile("EtlHitCALvsPhiZnegD2",
                        "ETL DIGI CAL vs #phi (-Z, Second disk);#phi_{DIGI} [rad];CAL_{DIGI}",
                        50,
                        -3.15,
                        3.15,
                        0.,
                        2048.);
  meHitCALvsPhi_[2] = ibook.bookProfile(
      "EtlHitCALvsPhiZposD1",
      "ETL DIGI CAL vs #phi (+Z, Single(topo1D)/First(topo2D) disk);#phi_{DIGI} [rad];CAL_{DIGI}",
      50,
      -3.15,
      3.15,
      0.,
      2048.);
  meHitCALvsPhi_[3] =
      ibook.bookProfile("EtlHitCALvsPhiZposD2",
                        "ETL DIGI CAL vs #phi (+Z, Second disk);#phi_{DIGI} [rad];CAL_{DIGI}",
                        50,
                        -3.15,
                        3.15,
                        0.,
                        2048.);
  meHitCALvsEta_[0] = ibook.bookProfile(
      "EtlHitCALvsEtaZnegD1",
      "ETL DIGI CAL vs #eta (-Z, Single(topo1D)/First(topo2D) disk);#eta_{DIGI};CAL_{DIGI}",
      50,
      -3.2,
      -1.56,
      0.,
      2048.);
  meHitCALvsEta_[1] = ibook.bookProfile("EtlHitCALvsEtaZnegD2",
                                      "ETL DIGI CAL vs #eta (-Z, Second disk);#eta_{DIGI};CAL_{DIGI}",
                                      50,
                                      -3.2,
                                      -1.56,
                                      0.,
                                      2048.);
  meHitCALvsEta_[2] = ibook.bookProfile(
      "EtlHitCALvsEtaZposD1",
      "ETL DIGI CAL vs #eta (+Z, Single(topo1D)/First(topo2D) disk);#eta_{DIGI};CAL_{DIGI}",
      50,
      1.56,
      3.2,
      0.,
      2048.);
  meHitCALvsEta_[3] = ibook.bookProfile("EtlHitCALvsEtaZposD2",
                                      "ETL DIGI CAL vs #eta (+Z, Second disk);#eta_{DIGI};CAL_{DIGI}",
                                      50,
                                      1.56,
                                      3.2,
                                      0.,
                                      2048.);

  /** // --- histograms booking

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
      "EtlHitCALVsZ", "ETL DIGI Calibration code vs Z;Z_{DIGI} [cm];CAL_{DIGI}", 50, -260., 260., 0., 1024.);**/
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
