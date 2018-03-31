/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 \file     SEIread.h
 \brief    reading funtionality for SEI messages
 */

#ifndef __SEIREAD__
#define __SEIREAD__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//! \ingroup TLibDecoder
//! \{

#include "TLibCommon/SEI.h"
class TComInputBitstream;


class SEIReader: public SyntaxElementParser
{
public:
  SEIReader() {};
  virtual ~SEIReader() {};
  Void parseSEImessage(TComInputBitstream* bs, SEIMessages& seis, const NalUnitType nalUnitType, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
protected:
  Void xReadSEImessage                        (SEIMessages& seis, const NalUnitType nalUnitType, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIuserDataUnregistered          (SEIuserDataUnregistered &sei,          UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIActiveParameterSets           (SEIActiveParameterSets  &sei,          UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIDecodingUnitInfo              (SEIDecodingUnitInfo& sei,              UInt payloadSize, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIDecodedPictureHash            (SEIDecodedPictureHash& sei,            UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIBufferingPeriod               (SEIBufferingPeriod& sei,               UInt payloadSize, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIPictureTiming                 (SEIPictureTiming& sei,                 UInt payloadSize, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIRecoveryPoint                 (SEIRecoveryPoint& sei,                 UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIFramePacking                  (SEIFramePacking& sei,                  UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEISegmentedRectFramePacking     (SEISegmentedRectFramePacking& sei,     UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIDisplayOrientation            (SEIDisplayOrientation &sei,            UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEITemporalLevel0Index           (SEITemporalLevel0Index &sei,           UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIRegionRefreshInfo             (SEIGradualDecodingRefreshInfo &sei,    UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEINoDisplay                     (SEINoDisplay &sei,                     UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIToneMappingInfo               (SEIToneMappingInfo& sei,               UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEISOPDescription                (SEISOPDescription &sei,                UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIScalableNesting               (SEIScalableNesting& sei, const NalUnitType nalUnitType, UInt payloadSize, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
  Void xParseSEITempMotionConstraintsTileSets (SEITempMotionConstrainedTileSets& sei, UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEITimeCode                      (SEITimeCode& sei,                      UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIChromaSamplingFilterHint      (SEIChromaSamplingFilterHint& sei,      UInt payloadSize/*,TComSPS* */,       std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIKneeFunctionInfo              (SEIKneeFunctionInfo& sei,              UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIMasteringDisplayColourVolume  (SEIMasteringDisplayColourVolume& sei,  UInt payloadSize,                     std::ostream *pDecodedMessageOutputStream);

  Void sei_read_code(std::ostream *pOS, UInt uiLength, UInt& ruiCode, const Char *pSymbolName);
  Void sei_read_uvlc(std::ostream *pOS,                UInt& ruiCode, const Char *pSymbolName);
  Void sei_read_svlc(std::ostream *pOS,                Int&  ruiCode, const Char *pSymbolName);
  Void sei_read_flag(std::ostream *pOS,                UInt& ruiCode, const Char *pSymbolName);
};


//! \}

#endif
