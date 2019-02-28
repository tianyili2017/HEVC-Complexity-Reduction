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

/** \file     encmain.cpp
    \brief    Encoder application main
*/

#include <time.h>
#include <iostream>
#include "TAppEncTop.h"
#include "TAppCommon/program_options_lite.h"

//! \ingroup TAppEncoder
//! \{

#include "../Lib/TLibCommon/Debug.h"
/*
int copyFile(char * fileSrc, char * fileDst)  //argc表示命令参数个数， argv[]表示参数名称
{
	int c;
	FILE *fpSrc, *fpDest;  //定义两个指向文件的指针

	fpSrc = fopen(fileSrc, "rb");    //以读取二进制的方式打开源文件
	if (fpSrc == NULL) {
		printf("Source file open failure: %s.\n", fileSrc);  //源文件不存在的时候提示错误
		return -1;
	}
	fpDest = fopen(fileDst, "wb");  // //以写入二进制的方式打开目标文件
	if (fpDest == NULL) {
		printf("Destination file open failure: %s.\n", fileDst);
		return -2;
	}
	while ((c = fgetc(fpSrc)) != EOF) {   //从源文件中读取数据知道结尾
		fputc(c, fpDest);
	}
	fclose(fpSrc);  //关闭文件指针，释放内存
	fclose(fpDest);
	return 0;
}
*/
// ====================================================================================================================
// Main function
// ====================================================================================================================

int main(int argc, char* argv[])
{
  TAppEncTop  cTAppEncTop;

  // print information
  fprintf( stdout, "\n" );
  fprintf( stdout, "HM software: Encoder Version [%s] (including RExt)", NV_VERSION );
  fprintf( stdout, NVM_ONOS );
  fprintf( stdout, NVM_COMPILEDBY );
  fprintf( stdout, NVM_BITS );
  fprintf( stdout, "\n\n" );

  // create application encoder class
  cTAppEncTop.create();

  // parse configuration
  try
  {
    if(!cTAppEncTop.parseCfg( argc, argv ))
    {
      cTAppEncTop.destroy();
#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
      EnvVar::printEnvVar();
#endif
      return 1;
    }
  }
  catch (df::program_options_lite::ParseFailure &e)
  {
    std::cerr << "Error parsing option \""<< e.arg <<"\" with argument \""<< e.val <<"\"." << std::endl;
    return 1;
  }

#if PRINT_MACRO_VALUES
  printMacroSettings();
#endif

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  EnvVar::printEnvVarInUse();
#endif

  // starting time
  Double dResult;
  clock_t lBefore = clock();

  // call encoding function
  cTAppEncTop.encode();

  // ending time
  dResult = (Double)(clock()-lBefore) / CLOCKS_PER_SEC;
  printf("\n Total Time: %12.3f sec.\n", dResult);

  // destroy application encoder class
  cTAppEncTop.destroy();
  
  /*
  // 20170319 added
  //static char cmd[500];
  static char phDateTime[100];
  static char phInputFile[200];
  static char phFileNameCUDepth[100], phFileNamePUPartSize[100], phFileNameTUDepth[100], phFileNameIndex[100];
  static char phFileNameCUDepthText[100], phFileNamePUPartSizeText[100], phFileNameTUDepthText[100];

  FILE * fpYuvNameTemp = fopen("YuvNameTemp.dat", "r+");
  assert(fscanf(fpYuvNameTemp, "%s\n%s", phInputFile, phDateTime)>=0);
  fclose(fpYuvNameTemp);

  sprintf(phFileNameCUDepth, "Info_%s_CUDepth.dat", phDateTime);
  sprintf(phFileNamePUPartSize, "Info_%s_PUPartSize.dat", phDateTime);
  sprintf(phFileNameTUDepth, "Info_%s_TUDepth.dat", phDateTime);
  sprintf(phFileNameIndex, "Info_%s_Index.dat", phDateTime);

  sprintf(phFileNameCUDepthText, "Info_%s_CUDepthText.dat", phDateTime);
  sprintf(phFileNamePUPartSizeText, "Info_%s_PUPartSizeText.dat", phDateTime);
  sprintf(phFileNameTUDepthText, "Info_%s_TUDepthText.dat", phDateTime);

  copyFile((char*)"Info_CUDepth.dat", phFileNameCUDepth);
  copyFile((char*)"Info_PUPartSize.dat", phFileNamePUPartSize);
  copyFile((char*)"Info_TUDepth.dat", phFileNameTUDepth);
  copyFile((char*)"Info_Index.dat", phFileNameIndex);
  copyFile((char*)"Info_CUDepthText.dat", phFileNameCUDepthText);
  copyFile((char*)"Info_PUPartSizeText.dat", phFileNamePUPartSizeText);
  copyFile((char*)"Info_TUDepthText.dat", phFileNameTUDepthText);
  */
  /*
  sprintf(cmd, "CopyFile.exe %s Info_CUDepth.dat", phFileNameCUDepth); system(cmd);
  sprintf(cmd, "CopyFile.exe %s Info_PUPartSize.dat", phFileNamePUPartSize); system(cmd);
  sprintf(cmd, "CopyFile.exe %s Info_TUDepth.dat", phFileNameTUDepth); system(cmd);
  sprintf(cmd, "CopyFile.exe %s Info_Index.dat", phFileNameIndex); system(cmd);
  sprintf(cmd, "CopyFile.exe %s Info_CUDepthText.dat", phFileNameCUDepthText); system(cmd);
  sprintf(cmd, "CopyFile.exe %s Info_PUPartSizeText.dat", phFileNamePUPartSizeText); system(cmd);
  sprintf(cmd, "CopyFile.exe %s Info_TUDepthText.dat", phFileNameTUDepthText); system(cmd);
  */
  // 20170319 end

  return 0;
}

//! \}
