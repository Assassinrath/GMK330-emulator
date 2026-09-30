#include "register_map.h"

static int32_t phaseApparent(uint16_t voltTimes10, uint16_t currTimes100) {
  return (int32_t)(((uint32_t)voltTimes10 * (uint32_t)currTimes100) / 1000);
}

static int16_t phasePF(int32_t powerW, int32_t apparentPowerVA) {
  if (apparentPowerVA <= 0) return 0;
  float powerFactor = (float)powerW / (float)apparentPowerVA;
  if (powerFactor > 1.0f) powerFactor = 1.0f;
  if (powerFactor < -1.0f) powerFactor = -1.0f;
  return (int16_t)lroundf(powerFactor * 1000.0f);
}

static uint16_t hiWord(int32_t value) {
  return (uint16_t)((uint32_t)value >> 16);
}

static uint16_t loWord(int32_t value) {
  return (uint16_t)value;
}

uint16_t gmk330TranslateRegister(const MeterData &meter, uint16_t address) {
  const int32_t apparentA = phaseApparent(meter.Va, meter.Ia);
  const int32_t apparentB = phaseApparent(meter.Vb, meter.Ib);
  const int32_t apparentC = phaseApparent(meter.Vc, meter.Ic);
  switch (address) {
    case 0x0132: return meter.Va;
    case 0x0133: return meter.Vb;
    case 0x0134: return meter.Vc;
    case 0x0135: return 0;                  case 0x0136: return meter.Ia;
    case 0x0137: return 0;                  case 0x0138: return meter.Ib;
    case 0x0139: return 0;                  case 0x013A: return meter.Ic;
    case 0x013B: return hiWord(meter.Pa);   case 0x013C: return loWord(meter.Pa);
    case 0x013D: return hiWord(meter.Pb);   case 0x013E: return loWord(meter.Pb);
    case 0x013F: return hiWord(meter.Pc);   case 0x0140: return loWord(meter.Pc);
    case 0x0141: return hiWord(meter.Pa + meter.Pb + meter.Pc);
    case 0x0142: return loWord(meter.Pa + meter.Pb + meter.Pc);
    case 0x0143: return 0;                  case 0x0144: return 0;
    case 0x0145: return 0;                  case 0x0146: return 0;
    case 0x0147: return 0;                  case 0x0148: return 0;
    case 0x0149: return 0;                  case 0x014A: return 0;
    case 0x014B: return hiWord(apparentA);  case 0x014C: return loWord(apparentA);
    case 0x014D: return hiWord(apparentB);  case 0x014E: return loWord(apparentB);
    case 0x014F: return hiWord(apparentC);  case 0x0150: return loWord(apparentC);
    case 0x0151: return hiWord(apparentA + apparentB + apparentC);
    case 0x0152: return loWord(apparentA + apparentB + apparentC);
    case 0x0153: return (uint16_t)phasePF(meter.Pa, apparentA);
    case 0x0154: return (uint16_t)phasePF(meter.Pb, apparentB);
    case 0x0155: return (uint16_t)phasePF(meter.Pc, apparentC);
    case 0x0156: return 2000;
    case 0x0157: return 5000;
    default: return 0;
  }
}

uint16_t gmk330DiscoveryRegister(uint16_t address) {
  switch (address) {
    case 0xF000: return 0x4D65;
    case 0xF001: return 0x7400;
    case 0x0203: return 0x0002;
    case 0x0204: return 0x0000;
    case 0x0205: return 0x0BB8;
    case 0x0206: return 0x0001;
    case 0x0207: return 0x0008;
    case 0x007B: return 0x0007;
    case 0x02EE: return 0xAA55;
    default: return 0;
  }
}
