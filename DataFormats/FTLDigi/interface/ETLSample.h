#ifndef DIGIFTL_ETLSAMPLE_H
#define DIGIFTL_ETLSAMPLE_H

#include <iostream>
#include <ostream>
#include <cstdint>

/**
   @class ETLSample
   @short wrapper for a data word
 */

class ETLSample {
public:
  enum ETLSampleDataMasks {
    kThreshMask = 0x1,
    kEAMask = 0x3,
    kToTMask = 0xff,
    kToAMask = 0x1ff,
    kCalMask = 0x1ff,
    kHitFlagMask = 0x7
  };
  enum ETLSamplePositionMasks {
    kRowMask = 0xf,
    kColMask = 0xf
  };
  enum ETLSampleDataShifts {
    kThreshShift = 31,
    kEAShift = 29,
    kToTShift = 21,
    kToAShift = 12,
    kCalShift = 3,
    kHitFlagShift = 0
  };
  enum ETLSamplePositionShifts {
    kRowShift = 0,
    kColShift = 4
  };

  /**
     @short CTOR
   */
  ETLSample() : value_(0), position_(0) {}
  ETLSample(uint32_t value, uint8_t position) : value_(value), position_(position) {}
  ETLSample(const ETLSample& o) : value_(o.value_), position_(o.position_) {}

  /**
     @short setters
   */
  void setThreshold(bool thr) { setDataWord(thr, kThreshMask, kThreshShift); }
  void setEA(uint16_t ea) { setDataWord(ea, kEAMask, kEAShift); }
  void setToT(uint16_t tot) { setDataWord(tot, kToTMask, kToTShift); }
  void setToA(uint16_t toa) { setDataWord(toa, kToAMask, kToAShift); }
  void setCal(uint16_t cal) { setDataWord(data, kCalMask, kCalShift); }
  void setHitFlag(uint16_t hitflag) { setDataWord(hitflag, kHitFlagMask, kHitFlagShift); }
  void setRow(uint8_t row) { setPositionWord(row, kRowMask, kRowShift); }
  void setCol(uint8_t col) { setPositionWord(col, kColMask, kColShift); }
  void set(bool thr, bool ea, uint16_t tot, uint16_t toa, uint16_t cal, uint16_t hitflag, uint8_t row, uint8_t col) {
    value_ = (((uint32_t)thr & kThreshMask) << kThreshShift | 
              ((uint32_t)ea & kEAMask) << kEAShift |
              ((uint32_t)tot & kToTMask) << kToTShift | 
              ((uint32_t)toa & kToAMask) << kToAShift | 
              ((uint32_t)cal & kCalMask) << kCalShift |
              ((uint32_t)hitflag & kHitFlagMask) << kHitFlagShift);
    position_ = (((uint8_t)row & kRowMask) << kRowShift |
                 ((uint8_t)col & kColMask) << kColShift);
  }
  void print(std::ostream& out = std::cout) {
    out << "( row col ) : ( " << row() << ' ' << column() << " ) "
        << "THR: " << threshold() << " EA: " << ea() << " ToT: " << tot() << " ToA: " << toa() << " Cal: " << cal() << " HitFlag: " << hitflag() 
        << " Raw Data=0x" << std::hex << raw_data() << std::dec << std::endl;
  }

  /**
     @short getters
  */
  uint32_t raw_data() const { return value_; }
  uint32_t raw_position() const { return position_; }
  bool threshold() const { return ((value_ >> kThreshShift) & kThreshMask); }
  uint32_t ea() const { return ((value_ >> kEAShift) & kEAMask); }
  uint32_t tot() const { return ((value_ >> kToTShift) & kToTMask); }
  uint32_t toa() const { return ((value_ >> kToAShift) & kToAMask); }
  uint32_t cal() const { return ((value_ >> kCalShift) & kCalMask); }
  uint32_t hitflag() const { return ((value_ >> kHitFlagShift) & kHitFlagMask); }
  uint8_t row() const { return ((position_ >> kRowShift) & kRowMask); }
  uint8_t column() const { return ((position_ >> kColShift) & kColMask); }
  //uint32_t operator()() { return value_; }

private:
  /**
     @short wrapper to reset words at a given position
   */
  void setDataWord(uint32_t word, uint32_t mask, uint32_t pos) {
    //clear required bits
    value_ &= ~(mask << pos);
    //now set the new value
    value_ |= ((word & mask) << pos);
  }
  void setPositionWord(uint8_t word, uint8_t mask, uint8_t pos) {
    //clear required bits
    position_ &= ~(mask << pos);
    //now set the new value
    position_ |= ((word & mask) << pos);
  }

  // a 32-bit word
  uint32_t value_;
  uint8_t position_;
};

#endif
