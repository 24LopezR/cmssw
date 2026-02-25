#ifndef DataFormats_FTLDigiSoA_interface_ETLDigiSoA_h
#define DataFormats_FTLDigiSoA_interface_ETLDigiSoA_h

#include <alpaka/alpaka.hpp>

#include "DataFormats/SoATemplate/interface/SoALayout.h"

namespace etldigi {

  GENERATE_SOA_LAYOUT(ETLDigiSoALayout,
                      SOA_COLUMN(uint32_t, rawId),     // Raw ID of the module/ETROC
                      SOA_COLUMN(uint8_t,  header),    // Header
                      SOA_COLUMN(uint8_t,  status),    // status of the ETROC
                      SOA_COLUMN(uint8_t,  nhits),     // N hits
                      SOA_COLUMN(uint8_t,  colID),     // ETROC column ID
                      SOA_COLUMN(uint8_t,  rowID),     // ETROC row ID
                      SOA_COLUMN(uint16_t, ToTdata)    // ToA                      
                      SOA_COLUMN(uint16_t, ToTdata)    // ToT                      
                      SOA_COLUMN(uint16_t, CALdata)    // CAL                      

  using ETLDigiSoA = ETLDigiSoALayout<>;
  using ETLDigiSoAView = ETLDigiSoA::View;
  using ETLDigiSoAConstView = ETLDigiSoA::ConstView;

  std::ostream &operator<<(std::ostream &out, ETLDigiSoA::View::const_element const &digi);

  // Getters
  ALPAKA_FN_HOST_ACC inline uint32_t rawId(const ETLDigiSoAConstView &etlDigi, int32_t i) {
    return (etlDigi[i].rawId());
  }
  ALPAKA_FN_HOST_ACC inline uint8_t header(const ETLDigiSoAConstView &etlDigi, int8_t i) {
    return (etlDigi[i].header());
  }
  ALPAKA_FN_HOST_ACC inline uint8_t status(const ETLDigiSoAConstView &etlDigi, int8_t i) {
    return (etlDigi[i].status());
  }
  ALPAKA_FN_HOST_ACC inline uint8_t nHits(const ETLDigiSoAConstView &etlDigi, int8_t i) {
    return (etlDigi[i].nHits());
  }
  ALPAKA_FN_HOST_ACC inline uint8_t colID(const ETLDigiSoAConstView &etlDigi, int8_t i) {
    return (etlDigi[i].colID());
  }
  ALPAKA_FN_HOST_ACC inline uint8_t rowID(const ETLDigiSoAConstView &etlDigi, int8_t i) {
    return (etlDigi[i].rowID());
  }
  ALPAKA_FN_HOST_ACC inline uint16_t ToAdata(const ETLDigiSoAConstView &etlDigi, int8_t i) {
    return (etlDigi[i].TDCword());
  }
  ALPAKA_FN_HOST_ACC inline uint16_t ToTdata(const ETLDigiSoAConstView &etlDigi, int8_t i) {
    return (etlDigi[i].TDCword());
  }
  ALPAKA_FN_HOST_ACC inline uint16_t CALdata(const ETLDigiSoAConstView &etlDigi, int8_t i) {
    return (etlDigi[i].TDCword());
  }

  // Setters
  ALPAKA_FN_HOST_ACC inline void rawId(ETLDigiSoA::View &etlDigi, int32_t i, uint32_t value) {
    etlDigi[i].rawId() = value;
  }
  ALPAKA_FN_HOST_ACC inline void header(ETLDigiSoA::View &etlDigi, int32_t i, uint8_t value) {
    etlDigi[i].header() = value;
  }
  ALPAKA_FN_HOST_ACC inline void status(ETLDigiSoA::View &etlDigi, int32_t i, uint8_t value) {
    etlDigi[i].status() = value;
  }
  ALPAKA_FN_HOST_ACC inline void nHits(ETLDigiSoA::View &etlDigi, int32_t i, uint8_t value) {
    etlDigi[i].nHits() = value;
  }
  ALPAKA_FN_HOST_ACC inline void colID(ETLDigiSoA::View &etlDigi, int32_t i, uint8_t value) {
    etlDigi[i].colID() = value;
  }
  ALPAKA_FN_HOST_ACC inline void rowID(ETLDigiSoA::View &etlDigi, int32_t i, uint8_t value) {
    etlDigi[i].rowID() = value;
  }
  ALPAKA_FN_HOST_ACC inline void ToAdata(ETLDigiSoA::View &etlDigi, int32_t i, uint16_t value) {
    etlDigi[i].ToAdata() = value;
  }
  ALPAKA_FN_HOST_ACC inline void ToTdata(ETLDigiSoA::View &etlDigi, int32_t i, uint16_t value) {
    etlDigi[i].ToTdata() = value;
  }
  ALPAKA_FN_HOST_ACC inline void CALdata(ETLDigiSoA::View &etlDigi, int32_t i, uint16_t value) {
    etlDigi[i].CALdata() = value;
  }
  ALPAKA_FN_HOST_ACC inline void setDigi(ETLDigiSoA::View &etlDigi,
                                         int32_t i,
                                         uint32_t rawId_val,
                                         uint8_t header_val,
                                         uint8_t status_val,
                                         uint8_t nHits_val,
                                         uint8_t colID_val,
                                         uint8_t rowID_val,
                                         uint16_t ToAdata_val,
                                         uint16_t ToTdata_val,
                                         uint16_t CALdata_val) {
    etlDigi[i].rawId()   = rawId_val;
    etlDigi[i].header()  = header_val;
    etlDigi[i].status()  = status_val;
    etlDigi[i].nHits()   = nHits_val;
    etlDigi[i].colID()   = colID_val;
    etlDigi[i].rowID()   = rowID_val;
    etlDigi[i].ToAdata() = ToAdata_val;
    etlDigi[i].ToTdata() = ToTdata_val;
    etlDigi[i].CALdata() = CALdata_val;
  }

}  // namespace etldigi
#endif  // DataFormats_FTLDigi_interface_ETLDigiSoA_h
