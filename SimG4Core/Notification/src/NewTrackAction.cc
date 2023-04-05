
#include "SimG4Core/Notification/interface/NewTrackAction.h"
#include "SimG4Core/Notification/interface/TrackInformation.h"

#include "FWCore/MessageLogger/interface/MessageLogger.h"

#include "G4Track.hh"

NewTrackAction::NewTrackAction() {}

void NewTrackAction::primary(G4Track *aTrack) const { addUserInfoToPrimary(aTrack); }

void NewTrackAction::secondary(G4Track *aSecondary, const G4Track &mother, int flag) const {
  const TrackInformation *motherInfo = static_cast<const TrackInformation *>(mother.GetUserInformation());
  addUserInfoToSecondary(aSecondary, *motherInfo, flag);
  LogDebug("SimTrackManager") << "NewTrackAction: Add track " << aSecondary->GetTrackID() << " from mother "
                              << mother.GetTrackID();
}

void NewTrackAction::addUserInfoToPrimary(G4Track *aTrack) const {
  TrackInformation *trkInfo = new TrackInformation();
  trkInfo->setPrimary(true);
  trkInfo->setStoreTrack();
  trkInfo->putInHistory();
  trkInfo->setGenParticlePID(aTrack->GetDefinition()->GetPDGEncoding());
  trkInfo->setGenParticleP(aTrack->GetMomentum().mag());
  aTrack->SetUserInformation(trkInfo);
}

void NewTrackAction::addUserInfoToSecondary(G4Track *aTrack, const TrackInformation &motherInfo, int flag) const {
  TrackInformation *trkInfo = new TrackInformation();
  LogDebug("SimG4CoreApplication") << "NewTrackAction called for " << aTrack->GetTrackID() << " mother "
                                   << motherInfo.isPrimary() << " flag " << flag;

  // Take care of cascade decays
  if (flag == 1) {
    trkInfo->setPrimary(true);
    trkInfo->setGenParticlePID(aTrack->GetDefinition()->GetPDGEncoding());
    trkInfo->setGenParticleP(aTrack->GetMomentum().mag());
  } else {
    trkInfo->setGenParticlePID(motherInfo.genParticlePID());
    trkInfo->setGenParticleP(motherInfo.genParticleP());
  }

  // Store if decay or conversion
  if (flag > 0) {
    trkInfo->setStoreTrack();
    trkInfo->putInHistory();
    trkInfo->setIDonCaloSurface(aTrack->GetTrackID(),
                                motherInfo.getIDCaloVolume(),
                                motherInfo.getIDLastVolume(),
                                aTrack->GetDefinition()->GetPDGEncoding(),
                                aTrack->GetMomentum().mag());
  } else {
    // transfer calo ID from mother (to be checked in TrackingAction)
    trkInfo->setIDonCaloSurface(motherInfo.getIDonCaloSurface(),
                                motherInfo.getIDCaloVolume(),
                                motherInfo.getIDLastVolume(),
                                motherInfo.caloSurfaceParticlePID(),
                                motherInfo.caloSurfaceParticleP());
  }

  if (motherInfo.hasCastorHit()) {
    trkInfo->setCastorHitPID(motherInfo.getCastorHitPID());
  }

  // manage ID of tracks in BTL to map them to SimTracks to be stored
  if (flag > 0) {
    if (isInBTL(aTrack)) {
      if ((motherInfo.storeTrack() && motherInfo.isFromTtoBTL()) || motherInfo.isBTLdaughter()) {
        trkInfo->setBTLdaughter();
        trkInfo->setIdAtBTLentrance(motherInfo.idAtBTLentrance());
      }
    }
  }

  aTrack->SetUserInformation(trkInfo);
}

bool NewTrackAction::isInBTL(const G4Track *aTrack) const {
  bool out = false;
  const G4VTouchable *touch = aTrack->GetTouchable();
  if (touch->GetVolume()->GetLogicalVolume()->GetRegion()->GetName() != "FastTimerRegion") {
    return out;
  }
  int theSize = touch->GetHistoryDepth() + 1;
  if (theSize > 1) {
    for (int ii = 0; ii < theSize; ii++) {
      const G4String &vName = touch->GetVolume(ii)->GetName();
      if (vName == "BarrelTimingLayer" || vName == "btl:BarrelTimingLayer_1") {
        out = true;
        break;
      }
    }
  }

  return out;
}
