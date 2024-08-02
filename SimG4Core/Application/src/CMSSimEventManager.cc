#include "SimG4Core/Application/interface/CMSSimEventManager.h"
#include "SimG4Core/Application/interface/RunAction.h"
#include "SimG4Core/Application/interface/EventAction.h"
#include "SimG4Core/Application/interface/StackingAction.h"
#include "SimG4Core/Application/interface/TrackingAction.h"

#include "FWCore/ParameterSet/interface/ParameterSet.h"
#include "FWCore/MessageLogger/interface/MessageLogger.h"

#include "G4Track.hh"
#include "G4Event.hh"
#include "G4TrajectoryContainer.hh"
#include "G4PrimaryTransformer.hh"
#include "G4TrackingManager.hh"
#include "G4TrackStatus.hh"
#include "G4UserSteppingAction.hh"

#include "G4SDManager.hh"
#include "G4StateManager.hh"
#include "G4ApplicationState.hh"
#include "G4TransportationManager.hh"
#include "G4Navigator.hh"

CMSSimEventManager::CMSSimEventManager(const edm::ParameterSet& iConfig)
    : verbose_(iConfig.getParameter<int>("EventVerbose")) {
  m_stateManager = G4StateManager::GetStateManager();
  m_defTrackManager = new G4TrackingManager();
  m_defTrackManager->SetVerboseLevel(3);
  m_primaryTransformer = new G4PrimaryTransformer();
  m_sdManager = G4SDManager::GetSDMpointerIfExist();
  m_navigator = G4TransportationManager::GetTransportationManager()->GetNavigatorForTracking();
  m_tracks.reserve(1000);
}

CMSSimEventManager::~CMSSimEventManager() {
  delete m_primaryTransformer;
  delete m_defTrackManager;
  delete m_eventAction;
  delete m_stackingAction;
}

void CMSSimEventManager::ProcessOneEvent(G4Event* anEvent) {
  trackID_ = 0;
  m_stateManager->SetNewState(G4State_EventProc);

  // Resetting Navigator has been moved to CMSSimEventManager,
  // so that resetting is now done for every event.
  G4ThreeVector center(0, 0, 0);
  m_navigator->LocateGlobalPointAndSetup(center, nullptr, false);

  G4Track* track = nullptr;

  anEvent->SetHCofThisEvent(m_sdManager->PrepareNewEvent());
  m_sdManager->PrepareNewEvent();
  m_eventAction->BeginOfEventAction(anEvent);

  // Fill primary tracks
  std::cout << "[CMSSimEventManager::ProcessOneEvent] START StackTracks(m_primaryTransformer->GimmePrimaries(anEvent), true)" << std::endl;
  StackTracks(m_primaryTransformer->GimmePrimaries(anEvent), true);
  std::cout << "[CMSSimEventManager::ProcessOneEvent] END   StackTracks(m_primaryTransformer->GimmePrimaries(anEvent), true)" << std::endl;

  if (0 < verbose_) {
    edm::LogVerbatim("CMSSimEventManager::ProcessOneEvent")
        << "### Event #" << anEvent->GetEventID() << "  " << trackID_ << " primary tracks";
  }

  // Loop over main stack of tracks
  bool v = false;
  do {
    track = m_tracks.back();
    G4PrimaryParticle* pp = track->GetDynamicParticle()->GetPrimaryParticle();
    if (pp != nullptr and (abs(pp->GetPDGcode()) == 13 or (abs(pp->GetPDGcode()) > 1000000 and abs(pp->GetPDGcode()) < 2000000)) and pp->GetTotalMomentum()/CLHEP::GeV > 50) { v = true; }
    m_tracks.pop_back();
    if (v) {std::cout << "[CMSSimEventManager::ProcessOneEvent] track_vtx=" << track-> GetVertexPosition() << std::endl;}
    if (v) {std::cout << "[CMSSimEventManager::ProcessOneEvent] START m_defTrackManager->ProcessOneTrack(track) [ID:" << pp->GetPDGcode() << ", p=" << pp->GetTotalMomentum()/CLHEP::GeV << "]" << std::endl;}
    m_defTrackManager->ProcessOneTrack(track);
    if (v) {std::cout << "[CMSSimEventManager::ProcessOneEvent] END   m_defTrackManager->ProcessOneTrack(track) [ID:" << pp->GetPDGcode() << ", p=" << pp->GetTotalMomentum()/CLHEP::GeV << "]" << std::endl;}
    G4TrackVector* secondaries = m_defTrackManager->GimmeSecondaries();
    if (v) {std::cout << "[CMSSimEventManager::ProcessOneEvent] START StackTracks(secondaries, false)" << std::endl;}
    StackTracks(secondaries, false);
    if (v) {std::cout << "[CMSSimEventManager::ProcessOneEvent] END StackTracks(secondaries, false)" << std::endl;}
    delete track;
    v = false;
  } while (!m_tracks.empty());

  m_sdManager->TerminateCurrentEvent(anEvent->GetHCofThisEvent());
  m_eventAction->EndOfEventAction(anEvent);
  m_stateManager->SetNewState(G4State_GeomClosed);
}

void CMSSimEventManager::StackTracks(G4TrackVector* trackVector, bool IDisSet) {
  bool v = false;
  if (trackVector == nullptr || trackVector->empty())
    return;
  for (auto& newTrack : *trackVector) {
    ++trackID_;
    G4PrimaryParticle* pp = newTrack->GetDynamicParticle()->GetPrimaryParticle();
    if (pp != nullptr) {
      if ((abs(pp->GetPDGcode()) == 13 or (abs(pp->GetPDGcode()) > 1000000 and abs(pp->GetPDGcode()) < 2000000)) and pp->GetTotalMomentum()/CLHEP::GeV > 50) { v = true; }
      if (abs(pp->GetPDGcode()) == 1000013) { std::cout << "SMuon is here!" << std::endl; }
    }
    if (!IDisSet) {
      newTrack->SetTrackID(trackID_);
      if (pp != nullptr) {
        if (v) {std::cout << "[CMSSimEventManager::StackTracks] Track: ID= " << trackID_ << ", pdg=" << pp->GetPDGcode() << ", p=" << pp->GetTotalMomentum()/CLHEP::GeV << std::endl;}
        pp->SetTrackID(trackID_);
      }
    }
    if (m_stackingAction->ClassifyNewTrack(newTrack) == fKill) {
      if (v) { std::cout << "[CMSSimEventManager::StackTracks] Killing track with ID " << trackID_ << std::endl; }
      delete newTrack;
    } else {
      newTrack->SetOriginTouchableHandle(newTrack->GetTouchableHandle());
      if (v) {std::cout << "[CMSSimEventManager::StackTracks] Track: ID=" << trackID_ << ", pdg=" << pp->GetPDGcode() << ", p=" << pp->GetTotalMomentum()/CLHEP::GeV
                                                           << ", Track_vertex=" << newTrack->GetVertexPosition() << std::endl;}
      m_tracks.push_back(newTrack);
    }
    v = false;
  }
  trackVector->clear();
}

void CMSSimEventManager::SetUserAction(EventAction* ptr) { m_eventAction = ptr; }

void CMSSimEventManager::SetUserAction(StackingAction* ptr) { m_stackingAction = ptr; }

void CMSSimEventManager::SetUserAction(TrackingAction* ptr) {
  m_trackingAction = ptr;
  m_defTrackManager->SetUserAction((G4UserTrackingAction*)ptr);
}

void CMSSimEventManager::SetUserAction(G4UserSteppingAction* ptr) { m_defTrackManager->SetUserAction(ptr); }
