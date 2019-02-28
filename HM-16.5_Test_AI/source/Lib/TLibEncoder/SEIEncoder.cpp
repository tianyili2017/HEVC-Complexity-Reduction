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

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/SEI.h"
#include "TEncGOP.h"
#include "TEncTop.h"

//! \ingroup TLibEncoder
//! \{

Void SEIEncoder::initSEIActiveParameterSets (SEIActiveParameterSets *seiActiveParameterSets, const TComVPS *vps, const TComSPS *sps)
{
  assert (m_isInitialized);
  assert (seiActiveParameterSets!=NULL);
  assert (vps!=NULL);
  assert (sps!=NULL);

  seiActiveParameterSets->activeVPSId = vps->getVPSId(); 
  seiActiveParameterSets->m_selfContainedCvsFlag = false;
  seiActiveParameterSets->m_noParameterSetUpdateFlag = false;
  seiActiveParameterSets->numSpsIdsMinus1 = 0;
  seiActiveParameterSets->activeSeqParameterSetId.resize(seiActiveParameterSets->numSpsIdsMinus1 + 1);
  seiActiveParameterSets->activeSeqParameterSetId[0] = sps->getSPSId();
}

Void SEIEncoder::initSEIFramePacking(SEIFramePacking *seiFramePacking, Int currPicNum)
{
  assert (m_isInitialized);
  assert (seiFramePacking!=NULL);

  seiFramePacking->m_arrangementId = m_pcCfg->getFramePackingArrangementSEIId();
  seiFramePacking->m_arrangementCancelFlag = 0;
  seiFramePacking->m_arrangementType = m_pcCfg->getFramePackingArrangementSEIType();
  assert((seiFramePacking->m_arrangementType > 2) && (seiFramePacking->m_arrangementType < 6) );
  seiFramePacking->m_quincunxSamplingFlag = m_pcCfg->getFramePackingArrangementSEIQuincunx();
  seiFramePacking->m_contentInterpretationType = m_pcCfg->getFramePackingArrangementSEIInterpretation();
  seiFramePacking->m_spatialFlippingFlag = 0;
  seiFramePacking->m_frame0FlippedFlag = 0;
  seiFramePacking->m_fieldViewsFlag = (seiFramePacking->m_arrangementType == 2);
  seiFramePacking->m_currentFrameIsFrame0Flag = ((seiFramePacking->m_arrangementType == 5) && (currPicNum&1) );
  seiFramePacking->m_frame0SelfContainedFlag = 0;
  seiFramePacking->m_frame1SelfContainedFlag = 0;
  seiFramePacking->m_frame0GridPositionX = 0;
  seiFramePacking->m_frame0GridPositionY = 0;
  seiFramePacking->m_frame1GridPositionX = 0;
  seiFramePacking->m_frame1GridPositionY = 0;
  seiFramePacking->m_arrangementReservedByte = 0;
  seiFramePacking->m_arrangementPersistenceFlag = true;
  seiFramePacking->m_upsampledAspectRatio = 0;
}

Void SEIEncoder::initSEISegmentedRectFramePacking(SEISegmentedRectFramePacking *seiSegmentedRectFramePacking)
{
  assert (m_isInitialized);
  assert (seiSegmentedRectFramePacking!=NULL);

  seiSegmentedRectFramePacking->m_arrangementCancelFlag = m_pcCfg->getSegmentedRectFramePackingArrangementSEICancel();
  seiSegmentedRectFramePacking->m_contentInterpretationType = m_pcCfg->getSegmentedRectFramePackingArrangementSEIType();
  seiSegmentedRectFramePacking->m_arrangementPersistenceFlag = m_pcCfg->getSegmentedRectFramePackingArrangementSEIPersistence();
}

Void SEIEncoder::initSEIDisplayOrientation(SEIDisplayOrientation* seiDisplayOrientation)
{
  assert (m_isInitialized);
  assert (seiDisplayOrientation!=NULL);

  seiDisplayOrientation->cancelFlag = false;
  seiDisplayOrientation->horFlip = false;
  seiDisplayOrientation->verFlip = false;
  seiDisplayOrientation->anticlockwiseRotation = m_pcCfg->getDisplayOrientationSEIAngle();
}

Void SEIEncoder::initSEIToneMappingInfo(SEIToneMappingInfo *seiToneMappingInfo)
{
  assert (m_isInitialized);
  assert (seiToneMappingInfo!=NULL);

  seiToneMappingInfo->m_toneMapId = m_pcCfg->getTMISEIToneMapId();
  seiToneMappingInfo->m_toneMapCancelFlag = m_pcCfg->getTMISEIToneMapCancelFlag();
  seiToneMappingInfo->m_toneMapPersistenceFlag = m_pcCfg->getTMISEIToneMapPersistenceFlag();

  seiToneMappingInfo->m_codedDataBitDepth = m_pcCfg->getTMISEICodedDataBitDepth();
  assert(seiToneMappingInfo->m_codedDataBitDepth >= 8 && seiToneMappingInfo->m_codedDataBitDepth <= 14);
  seiToneMappingInfo->m_targetBitDepth = m_pcCfg->getTMISEITargetBitDepth();
  assert(seiToneMappingInfo->m_targetBitDepth >= 1 && seiToneMappingInfo->m_targetBitDepth <= 17);
  seiToneMappingInfo->m_modelId = m_pcCfg->getTMISEIModelID();
  assert(seiToneMappingInfo->m_modelId >=0 &&seiToneMappingInfo->m_modelId<=4);

  switch( seiToneMappingInfo->m_modelId)
  {
  case 0:
    {
      seiToneMappingInfo->m_minValue = m_pcCfg->getTMISEIMinValue();
      seiToneMappingInfo->m_maxValue = m_pcCfg->getTMISEIMaxValue();
      break;
    }
  case 1:
    {
      seiToneMappingInfo->m_sigmoidMidpoint = m_pcCfg->getTMISEISigmoidMidpoint();
      seiToneMappingInfo->m_sigmoidWidth = m_pcCfg->getTMISEISigmoidWidth();
      break;
    }
  case 2:
    {
      UInt num = 1u<<(seiToneMappingInfo->m_targetBitDepth);
      seiToneMappingInfo->m_startOfCodedInterval.resize(num);
      Int* ptmp = m_pcCfg->getTMISEIStartOfCodedInterva();
      if(ptmp)
      {
        for(Int i=0; i<num;i++)
        {
          seiToneMappingInfo->m_startOfCodedInterval[i] = ptmp[i];
        }
      }
      break;
    }
  case 3:
    {
      seiToneMappingInfo->m_numPivots = m_pcCfg->getTMISEINumPivots();
      seiToneMappingInfo->m_codedPivotValue.resize(seiToneMappingInfo->m_numPivots);
      seiToneMappingInfo->m_targetPivotValue.resize(seiToneMappingInfo->m_numPivots);
      Int* ptmpcoded = m_pcCfg->getTMISEICodedPivotValue();
      Int* ptmptarget = m_pcCfg->getTMISEITargetPivotValue();
      if(ptmpcoded&&ptmptarget)
      {
        for(Int i=0; i<(seiToneMappingInfo->m_numPivots);i++)
        {
          seiToneMappingInfo->m_codedPivotValue[i]=ptmpcoded[i];
          seiToneMappingInfo->m_targetPivotValue[i]=ptmptarget[i];
        }
      }
      break;
    }
  case 4:
    {
      seiToneMappingInfo->m_cameraIsoSpeedIdc = m_pcCfg->getTMISEICameraIsoSpeedIdc();
      seiToneMappingInfo->m_cameraIsoSpeedValue = m_pcCfg->getTMISEICameraIsoSpeedValue();
      assert( seiToneMappingInfo->m_cameraIsoSpeedValue !=0 );
      seiToneMappingInfo->m_exposureIndexIdc = m_pcCfg->getTMISEIExposurIndexIdc();
      seiToneMappingInfo->m_exposureIndexValue = m_pcCfg->getTMISEIExposurIndexValue();
      assert( seiToneMappingInfo->m_exposureIndexValue !=0 );
      seiToneMappingInfo->m_exposureCompensationValueSignFlag = m_pcCfg->getTMISEIExposureCompensationValueSignFlag();
      seiToneMappingInfo->m_exposureCompensationValueNumerator = m_pcCfg->getTMISEIExposureCompensationValueNumerator();
      seiToneMappingInfo->m_exposureCompensationValueDenomIdc = m_pcCfg->getTMISEIExposureCompensationValueDenomIdc();
      seiToneMappingInfo->m_refScreenLuminanceWhite = m_pcCfg->getTMISEIRefScreenLuminanceWhite();
      seiToneMappingInfo->m_extendedRangeWhiteLevel = m_pcCfg->getTMISEIExtendedRangeWhiteLevel();
      assert( seiToneMappingInfo->m_extendedRangeWhiteLevel >= 100 );
      seiToneMappingInfo->m_nominalBlackLevelLumaCodeValue = m_pcCfg->getTMISEINominalBlackLevelLumaCodeValue();
      seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue = m_pcCfg->getTMISEINominalWhiteLevelLumaCodeValue();
      assert( seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue > seiToneMappingInfo->m_nominalBlackLevelLumaCodeValue );
      seiToneMappingInfo->m_extendedWhiteLevelLumaCodeValue = m_pcCfg->getTMISEIExtendedWhiteLevelLumaCodeValue();
      assert( seiToneMappingInfo->m_extendedWhiteLevelLumaCodeValue >= seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue );
      break;
    }
  default:
    {
      assert(!"Undefined SEIToneMapModelId");
      break;
    }
  }
}

Void SEIEncoder::initSEISOPDescription(SEISOPDescription *sopDescriptionSEI, TComSlice *slice, Int picInGOP, Int lastIdr, Int currGOPSize)
{
  assert (m_isInitialized);
  assert (sopDescriptionSEI != NULL);
  assert (slice != NULL);

  Int sopCurrPOC = slice->getPOC();
  sopDescriptionSEI->m_sopSeqParameterSetId = slice->getSPS()->getSPSId();

  Int i = 0;
  Int prevEntryId = picInGOP;
  for (Int j = picInGOP; j < currGOPSize; j++)
  {
    Int deltaPOC = m_pcCfg->getGOPEntry(j).m_POC - m_pcCfg->getGOPEntry(prevEntryId).m_POC;
    if ((sopCurrPOC + deltaPOC) < m_pcCfg->getFramesToBeEncoded())
    {
      sopCurrPOC += deltaPOC;
      sopDescriptionSEI->m_sopDescVclNaluType[i] = m_pcEncGOP->getNalUnitType(sopCurrPOC, lastIdr, slice->getPic()->isField());
      sopDescriptionSEI->m_sopDescTemporalId[i] = m_pcCfg->getGOPEntry(j).m_temporalId;
      sopDescriptionSEI->m_sopDescStRpsIdx[i] = m_pcEncTop->getReferencePictureSetIdxForSOP(sopCurrPOC, j);
      sopDescriptionSEI->m_sopDescPocDelta[i] = deltaPOC;

      prevEntryId = j;
      i++;
    }
  }

  sopDescriptionSEI->m_numPicsInSopMinus1 = i - 1;
}

Void SEIEncoder::initSEIBufferingPeriod(SEIBufferingPeriod *bufferingPeriodSEI, TComSlice *slice)
{
  assert (m_isInitialized);
  assert (bufferingPeriodSEI != NULL);
  assert (slice != NULL);

  UInt uiInitialCpbRemovalDelay = (90000/2);                      // 0.5 sec
  bufferingPeriodSEI->m_initialCpbRemovalDelay      [0][0]     = uiInitialCpbRemovalDelay;
  bufferingPeriodSEI->m_initialCpbRemovalDelayOffset[0][0]     = uiInitialCpbRemovalDelay;
  bufferingPeriodSEI->m_initialCpbRemovalDelay      [0][1]     = uiInitialCpbRemovalDelay;
  bufferingPeriodSEI->m_initialCpbRemovalDelayOffset[0][1]     = uiInitialCpbRemovalDelay;

  Double dTmp = (Double)slice->getSPS()->getVuiParameters()->getTimingInfo()->getNumUnitsInTick() / (Double)slice->getSPS()->getVuiParameters()->getTimingInfo()->getTimeScale();

  UInt uiTmp = (UInt)( dTmp * 90000.0 );
  uiInitialCpbRemovalDelay -= uiTmp;
  uiInitialCpbRemovalDelay -= uiTmp / ( slice->getSPS()->getVuiParameters()->getHrdParameters()->getTickDivisorMinus2() + 2 );
  bufferingPeriodSEI->m_initialAltCpbRemovalDelay      [0][0]  = uiInitialCpbRemovalDelay;
  bufferingPeriodSEI->m_initialAltCpbRemovalDelayOffset[0][0]  = uiInitialCpbRemovalDelay;
  bufferingPeriodSEI->m_initialAltCpbRemovalDelay      [0][1]  = uiInitialCpbRemovalDelay;
  bufferingPeriodSEI->m_initialAltCpbRemovalDelayOffset[0][1]  = uiInitialCpbRemovalDelay;

  bufferingPeriodSEI->m_rapCpbParamsPresentFlag = 0;
  //for the concatenation, it can be set to one during splicing.
  bufferingPeriodSEI->m_concatenationFlag = 0;
  //since the temporal layer HRD is not ready, we assumed it is fixed
  bufferingPeriodSEI->m_auCpbRemovalDelayDelta = 1;
  bufferingPeriodSEI->m_cpbDelayOffset = 0;
  bufferingPeriodSEI->m_dpbDelayOffset = 0;
}

//! initialize scalable nesting SEI message.
//! Note: The SEI message structures input into this function will become part of the scalable nesting SEI and will be 
//!       automatically freed, when the nesting SEI is disposed.
Void SEIEncoder::initSEIScalableNesting(SEIScalableNesting *scalableNestingSEI, SEIMessages &nestedSEIs)
{
  assert (m_isInitialized);
  assert (scalableNestingSEI != NULL);

  scalableNestingSEI->m_bitStreamSubsetFlag           = 1;      // If the nested SEI messages are picture buffering SEI messages, picture timing SEI messages or sub-picture timing SEI messages, bitstream_subset_flag shall be equal to 1
  scalableNestingSEI->m_nestingOpFlag                 = 0;
  scalableNestingSEI->m_nestingNumOpsMinus1           = 0;      //nesting_num_ops_minus1
  scalableNestingSEI->m_allLayersFlag                 = 0;
  scalableNestingSEI->m_nestingNoOpMaxTemporalIdPlus1 = 6 + 1;  //nesting_no_op_max_temporal_id_plus1
  scalableNestingSEI->m_nestingNumLayersMinus1        = 1 - 1;  //nesting_num_layers_minus1
  scalableNestingSEI->m_nestingLayerId[0]             = 0;

  scalableNestingSEI->m_nestedSEIs.clear();
  for (SEIMessages::iterator it=nestedSEIs.begin(); it!=nestedSEIs.end(); it++)
  {
    scalableNestingSEI->m_nestedSEIs.push_back((*it));
  }
}

Void SEIEncoder::initSEIRecoveryPoint(SEIRecoveryPoint *recoveryPointSEI, TComSlice *slice)
{
  assert (m_isInitialized);
  assert (recoveryPointSEI != NULL);
  assert (slice != NULL);

  recoveryPointSEI->m_recoveryPocCnt    = 0;
  recoveryPointSEI->m_exactMatchingFlag = ( slice->getPOC() == 0 ) ? (true) : (false);
  recoveryPointSEI->m_brokenLinkFlag    = false;
}

//! calculate hashes for entire reconstructed picture
Void SEIEncoder::initDecodedPictureHashSEI(SEIDecodedPictureHash *decodedPictureHashSEI, TComPic *pcPic, std::string &rHashString, const BitDepths &bitDepths)
{
  assert (m_isInitialized);
  assert (decodedPictureHashSEI!=NULL);
  assert (pcPic!=NULL);

  if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 1)
  {
    decodedPictureHashSEI->method = SEIDecodedPictureHash::MD5;
    UInt numChar=calcMD5(*pcPic->getPicYuvRec(), decodedPictureHashSEI->m_pictureHash, bitDepths);
    rHashString = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
  }
  else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 2)
  {
    decodedPictureHashSEI->method = SEIDecodedPictureHash::CRC;
    UInt numChar=calcCRC(*pcPic->getPicYuvRec(), decodedPictureHashSEI->m_pictureHash, bitDepths);
    rHashString = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
  }
  else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 3)
  {
    decodedPictureHashSEI->method = SEIDecodedPictureHash::CHECKSUM;
    UInt numChar=calcChecksum(*pcPic->getPicYuvRec(), decodedPictureHashSEI->m_pictureHash, bitDepths);
    rHashString = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
  }
}

Void SEIEncoder::initTemporalLevel0IndexSEI(SEITemporalLevel0Index *temporalLevel0IndexSEI, TComSlice *slice)
{
  assert (m_isInitialized);
  assert (temporalLevel0IndexSEI!=NULL);
  assert (slice!=NULL);

  if (slice->getRapPicFlag())
  {
    m_tl0Idx = 0;
    m_rapIdx = (m_rapIdx + 1) & 0xFF;
  }
  else
  {
    m_tl0Idx = (m_tl0Idx + (slice->getTLayer() ? 0 : 1)) & 0xFF;
  }
  temporalLevel0IndexSEI->tl0Idx = m_tl0Idx;
  temporalLevel0IndexSEI->rapIdx = m_rapIdx;
}

Void SEIEncoder::initSEITempMotionConstrainedTileSets (SEITempMotionConstrainedTileSets *sei, const TComPPS *pps)
{
  assert (m_isInitialized);
  assert (sei!=NULL);
  assert (pps!=NULL);

  if(pps->getTilesEnabledFlag())
  {
    sei->m_mc_all_tiles_exact_sample_value_match_flag = false;
    sei->m_each_tile_one_tile_set_flag                = false;
    sei->m_limited_tile_set_display_flag              = false;
    sei->setNumberOfTileSets((pps->getNumTileColumnsMinus1() + 1) * (pps->getNumTileRowsMinus1() + 1));

    for(Int i=0; i < sei->getNumberOfTileSets(); i++)
    {
      sei->tileSetData(i).m_mcts_id = i;  //depends the application;
      sei->tileSetData(i).setNumberOfTileRects(1);

      for(Int j=0; j<sei->tileSetData(i).getNumberOfTileRects(); j++)
      {
        sei->tileSetData(i).topLeftTileIndex(j)     = i+j;
        sei->tileSetData(i).bottomRightTileIndex(j) = i+j;
      }

      sei->tileSetData(i).m_exact_sample_value_match_flag    = false;
      sei->tileSetData(i).m_mcts_tier_level_idc_present_flag = false;
    }
  }
  else
  {
    assert(!"Tile is not enabled");
  }
}

Void SEIEncoder::initSEIKneeFunctionInfo(SEIKneeFunctionInfo *seiKneeFunctionInfo)
{
  assert (m_isInitialized);
  assert (seiKneeFunctionInfo!=NULL);

  seiKneeFunctionInfo->m_kneeId = m_pcCfg->getKneeSEIId();
  seiKneeFunctionInfo->m_kneeCancelFlag = m_pcCfg->getKneeSEICancelFlag();
  if ( !seiKneeFunctionInfo->m_kneeCancelFlag )
  {
    seiKneeFunctionInfo->m_kneePersistenceFlag = m_pcCfg->getKneeSEIPersistenceFlag();
    seiKneeFunctionInfo->m_kneeInputDrange = m_pcCfg->getKneeSEIInputDrange();
    seiKneeFunctionInfo->m_kneeInputDispLuminance = m_pcCfg->getKneeSEIInputDispLuminance();
    seiKneeFunctionInfo->m_kneeOutputDrange = m_pcCfg->getKneeSEIOutputDrange();
    seiKneeFunctionInfo->m_kneeOutputDispLuminance = m_pcCfg->getKneeSEIOutputDispLuminance();

    seiKneeFunctionInfo->m_kneeNumKneePointsMinus1 = m_pcCfg->getKneeSEINumKneePointsMinus1();
    Int* piInputKneePoint  = m_pcCfg->getKneeSEIInputKneePoint();
    Int* piOutputKneePoint = m_pcCfg->getKneeSEIOutputKneePoint();
    if(piInputKneePoint&&piOutputKneePoint)
    {
      seiKneeFunctionInfo->m_kneeInputKneePoint.resize(seiKneeFunctionInfo->m_kneeNumKneePointsMinus1+1);
      seiKneeFunctionInfo->m_kneeOutputKneePoint.resize(seiKneeFunctionInfo->m_kneeNumKneePointsMinus1+1);
      for(Int i=0; i<=seiKneeFunctionInfo->m_kneeNumKneePointsMinus1; i++)
      {
        seiKneeFunctionInfo->m_kneeInputKneePoint[i] = piInputKneePoint[i];
        seiKneeFunctionInfo->m_kneeOutputKneePoint[i] = piOutputKneePoint[i];
      }
    }
  }
}

Void SEIEncoder::initSEIChromaSamplingFilterHint(SEIChromaSamplingFilterHint *seiChromaSamplingFilterHint, Int iHorFilterIndex, Int iVerFilterIndex)
{
  assert (m_isInitialized);
  assert (seiChromaSamplingFilterHint!=NULL);

  seiChromaSamplingFilterHint->m_verChromaFilterIdc = iVerFilterIndex;
  seiChromaSamplingFilterHint->m_horChromaFilterIdc = iHorFilterIndex;
  seiChromaSamplingFilterHint->m_verFilteringProcessFlag = 1;
  seiChromaSamplingFilterHint->m_targetFormatIdc = 3;
  seiChromaSamplingFilterHint->m_perfectReconstructionFlag = false;
  if(seiChromaSamplingFilterHint->m_verChromaFilterIdc == 1)
  {
    seiChromaSamplingFilterHint->m_numVerticalFilters = 1;
    seiChromaSamplingFilterHint->m_verTapLengthMinus1 = (Int*)malloc(seiChromaSamplingFilterHint->m_numVerticalFilters * sizeof(Int));
    seiChromaSamplingFilterHint->m_verFilterCoeff =    (Int**)malloc(seiChromaSamplingFilterHint->m_numVerticalFilters * sizeof(Int*));
    for(Int i = 0; i < seiChromaSamplingFilterHint->m_numVerticalFilters; i ++)
    {
      seiChromaSamplingFilterHint->m_verTapLengthMinus1[i] = 0;
      seiChromaSamplingFilterHint->m_verFilterCoeff[i] = (Int*)malloc(seiChromaSamplingFilterHint->m_verTapLengthMinus1[i] * sizeof(Int));
      for(Int j = 0; j < seiChromaSamplingFilterHint->m_verTapLengthMinus1[i]; j ++)
      {
        seiChromaSamplingFilterHint->m_verFilterCoeff[i][j] = 0;
      }
    }
  }
  else
  {
    seiChromaSamplingFilterHint->m_numVerticalFilters = 0;
    seiChromaSamplingFilterHint->m_verTapLengthMinus1 = NULL;
    seiChromaSamplingFilterHint->m_verFilterCoeff = NULL;
  }
  if(seiChromaSamplingFilterHint->m_horChromaFilterIdc == 1)
  {
    seiChromaSamplingFilterHint->m_numHorizontalFilters = 1;
    seiChromaSamplingFilterHint->m_horTapLengthMinus1 = (Int*)malloc(seiChromaSamplingFilterHint->m_numHorizontalFilters * sizeof(Int));
    seiChromaSamplingFilterHint->m_horFilterCoeff = (Int**)malloc(seiChromaSamplingFilterHint->m_numHorizontalFilters * sizeof(Int*));
    for(Int i = 0; i < seiChromaSamplingFilterHint->m_numHorizontalFilters; i ++)
    {
      seiChromaSamplingFilterHint->m_horTapLengthMinus1[i] = 0;
      seiChromaSamplingFilterHint->m_horFilterCoeff[i] = (Int*)malloc(seiChromaSamplingFilterHint->m_horTapLengthMinus1[i] * sizeof(Int));
      for(Int j = 0; j < seiChromaSamplingFilterHint->m_horTapLengthMinus1[i]; j ++)
      {
        seiChromaSamplingFilterHint->m_horFilterCoeff[i][j] = 0;
      }
    }
  }
  else
  {
    seiChromaSamplingFilterHint->m_numHorizontalFilters = 0;
    seiChromaSamplingFilterHint->m_horTapLengthMinus1 = NULL;
    seiChromaSamplingFilterHint->m_horFilterCoeff = NULL;
  }
}

Void SEIEncoder::initSEITimeCode(SEITimeCode *seiTimeCode)
{
  assert (m_isInitialized);
  assert (seiTimeCode!=NULL);
  //  Set data as per command line options
  seiTimeCode->numClockTs = m_pcCfg->getNumberOfTimesets();
  for(Int i = 0; i < seiTimeCode->numClockTs; i++)
  {
    seiTimeCode->timeSetArray[i] = m_pcCfg->getTimeSet(i);
  }
}

//! \}
