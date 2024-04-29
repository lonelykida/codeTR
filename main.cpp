/* Authors: Lutong Wang and Bangqi Xu */
/*
 * Copyright (c) 2019, The Regents of the University of California
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <string>
#include <chrono>
#include "FlexRoute.h"

using namespace std;
using namespace fr;

/*
  1.打开参数文件并检查是否成功打开。
  2.逐行读取参数文件内容。
  3.对于每一行，如果不是以#开头（表示注释），则解析出字段和值。
  4.根据字段的不同，设置相应的变量值，其中涉及到的变量包括LEF_FILE、DEF_FILE、GUIDE_FILE、OUT_FILE等。
  5.如果成功读取了参数数量超过5个，则返回0；否则返回2，表示读取参数文件失败。
  这段代码的核心功能是根据参数文件的内容来设置程序的参数，以便后续的处理能够按照指定的参数进行。
*/

int readParams(const string &fileName) {
  int readParamCnt = 0; //读取的参数数量计数
  fstream fin(fileName.c_str());  //file input stream，试以读取模式打开名为fileName的文件
  string line;          //读取每一行
  if (fin.is_open()){   //判断文件是否打开成功
    while (fin.good()){ //判断文件是否读取完毕，如果文件流处于可读取状态，则继续读取下一行
      getline(fin, line); //读取下一行
      if (line[0] != '#'){  //如果不是以#开头（表示注释），则解析出字段和值
        char delimiter=':'; //定义分隔符为':'
        int pos = line.find(delimiter); //查找分隔符的位置，返回的是分隔符第一次出现的位置
        string field = line.substr(0, pos); //截取字段,即第0个位置到第1个分隔符之间的内容
        string value = line.substr(pos + 1);//截取值，即第1个分隔符后面的内容
        stringstream ss(value);         //将value转换为字符串流，以方便地转换为其它类型的值，如ss = "32"，则将其装入int val后(ss>>val)，val=32(int类型)
        if (field == "lef")           { LEF_FILE = value; ++readParamCnt;}  //如果字段为lef，则将其值赋给LEF_FILE,并将readParamCnt加1
        else if (field == "def")      { DEF_FILE = value; REF_OUT_FILE = DEF_FILE; ++readParamCnt;} //如果字段为def，则将其值赋给DEF_FILE，并将REF_OUT_FILE赋值为DEF_FILE
        else if (field == "guide")    { GUIDE_FILE = value; ++readParamCnt;}  //如果字段为guide，则将其值赋给GUIDE_FILE,并将readParamCnt加1
        else if (field == "outputTA") { OUTTA_FILE = value; ++readParamCnt;}  //如果字段为outputTA，则将其值赋给OUTTA_FILE,并将readParamCnt加1
        else if (field == "output")   { OUT_FILE = value; ++readParamCnt;}  //如果字段为output，则将其值赋给OUT_FILE,并将readParamCnt加1
        else if (field == "outputguide") { OUTGUIDE_FILE = value; ++readParamCnt;}  //如果字段为outputguide，则将其值赋给OUTGUIDE_FILE,并将readParamCnt加1
        else if (field == "outputMaze") { OUT_MAZE_FILE = value; ++readParamCnt;} //如果字段为outputMaze，则将其值赋给OUT_MAZE_FILE,并将readParamCnt加1
        else if (field == "outputDRC") { DRC_RPT_FILE = value; ++readParamCnt;} //如果字段为outputDRC，则将其值赋给DRC_RPT_FILE,并将readParamCnt加1
        else if (field == "outputCMap") { CMAP_FILE = value; ++readParamCnt;} //如果字段为outputCMap，则将其值赋给CMAP_FILE,并将readParamCnt加1
        else if (field == "threads")  { MAX_THREADS = atoi(value.c_str()); ++readParamCnt;} //如果字段为threads，则将其值赋给MAX_THREADS,并将readParamCnt加1
        else if (field == "verbose")    VERBOSE = atoi(value.c_str());  //如果字段为verbose，则将其值赋给VERBOSE,并将readParamCnt加1
        else if (field == "dbProcessNode") { DBPROCESSNODE = value; ++readParamCnt;}  //如果字段为dbProcessNode，则将其值赋给DBPROCESSNODE,并将readParamCnt加1
        else if (field == "drouteOnGridOnlyPrefWireBottomLayerNum") { ONGRIDONLY_WIRE_PREF_BOTTOMLAYERNUM = atoi(value.c_str()); ++readParamCnt;} //如果字段为drouteOnGridOnlyPrefWireBottomLayerNum，则将其值赋给ONGRIDONLY_WIRE_PREF_BOTTOMLAYERNUM,并将readParamCnt加1
        else if (field == "drouteOnGridOnlyPrefWireTopLayerNum") { ONGRIDONLY_WIRE_PREF_TOPLAYERNUM = atoi(value.c_str()); ++readParamCnt;}
        else if (field == "drouteOnGridOnlyNonPrefWireBottomLayerNum") { ONGRIDONLY_WIRE_NONPREF_BOTTOMLAYERNUM = atoi(value.c_str()); ++readParamCnt;}
        else if (field == "drouteOnGridOnlyNonPrefWireTopLayerNum") { ONGRIDONLY_WIRE_NONPREF_TOPLAYERNUM = atoi(value.c_str()); ++readParamCnt;}
        else if (field == "drouteOnGridOnlyViaBottomLayerNum") { ONGRIDONLY_VIA_BOTTOMLAYERNUM = atoi(value.c_str()); ++readParamCnt;}
        else if (field == "drouteOnGridOnlyViaTopLayerNum") { ONGRIDONLY_VIA_TOPLAYERNUM = atoi(value.c_str()); ++readParamCnt;}
        else if (field == "drouteViaInPinBottomLayerNum") { VIAINPIN_BOTTOMLAYERNUM = atoi(value.c_str()); ++readParamCnt;}
        else if (field == "drouteViaInPinTopLayerNum") { VIAINPIN_TOPLAYERNUM = atoi(value.c_str()); ++readParamCnt;}
        else if (field == "drouteEndIterNum") { END_ITERATION = atoi(value.c_str()); ++readParamCnt;}
        else if (field == "OR_SEED") {OR_SEED = atoi(value.c_str()); ++readParamCnt;}
        else if (field == "OR_K") {OR_K = atof(value.c_str()); ++readParamCnt;}
      }
    }
    fin.close();        //关闭文件
  }
  
  if (readParamCnt < 5) { //读取参数文件不全（<5），返回2
    return 2;
  } else {
    return 0;
  }
}

int main(int argc, char** argv) {

  //cout <<CPX_MIN <<endl;
  //return 0;
  using namespace std::chrono;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();  //记录当前时间now到t1中
  
  //double startTime = omp_get_wtime();
  //std::ios::sync_with_stdio(false);
  //cout <<"TritonRoute Version 0.0.6.0" <<endl;
  //cout <<"Developed by Lutong Wang and Bangqi Xu\n"
  //     <<"\n"
  //     <<"Copyright (c) 2019, The Regents of the University of California\n"
  //     <<"All rights reserved.\n"
  //     <<"\n"
  //     <<"Redistribution and use in source and binary forms, with or without\n"
  //     <<"modification, are permitted provided that the following conditions are met:\n"
  //     <<"    * Redistributions of source code must retain the above copyright\n"
  //     <<"      notice, this list of conditions and the following disclaimer.\n"
  //     <<"    * Redistributions in binary form must reproduce the above copyright\n"
  //     <<"      notice, this list of conditions and the following disclaimer in the\n"
  //     <<"      documentation and/or other materials provided with the distribution.\n"
  //     <<"    * Neither the name of the University nor the\n"
  //     <<"      names of its contributors may be used to endorse or promote products\n"
  //     <<"      derived from this software without specific prior written permission.\n"
  //     <<"\n"
  //     <<"THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS \"AS IS\" AND\n"
  //     <<"ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED\n"
  //     <<"WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE\n"
  //     <<"DISCLAIMED. IN NO EVENT SHALL THE REGENTS BE LIABLE FOR ANY\n"
  //     <<"DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES\n"
  //     <<"(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;\n"
  //     <<"LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND\n"
  //     <<"ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT\n"
  //     <<"(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS\n"
  //     <<"SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n"
  //     <<"\n";
  
  //若传入的参数只有1个，则说明用户只输入了函数名，则报错2
  if (argc == 1) {  
    cout <<"Error: usage ./TritonRoute -lef <LEF_FILE> -def <DEF_FILE> -guide <GUIDE_FILE> -output <OUTPUT_DEF>" <<endl;
    return 2;
  }

  if (argc == 2) {  
    int readSuccess = readParams(string(argv[1]));  //用户输入了两个参数，并将第二个参数传入readParams中
    if (readSuccess) {  //若readParams返回0，则说明读取成功，否则读取参数文件出错
      cout <<"Error reading param file!!!" <<endl;
      return 2;
    }
  } else {  //读取成功
  /*
    这段代码是在命令行参数数量大于2时的处理逻辑，用于解析并读取命令行参数。
    它通过循环遍历命令行参数，逐个解析参数的值，并将其赋给对应的变量。具体解析的参数包括：

    -lef <LEF_FILE>：设置LEF文件路径。
    -def <DEF_FILE>：设置DEF文件路径，并将其作为REF_OUT_FILE的值。
    -guide <GUIDE_FILE>：设置GUIDE文件路径。
    -threads <MAX_THREADS>：设置最大线程数。
    -output <OUT_FILE>：设置输出文件路径。
    -verbose <VERBOSE>：设置详细输出等级。
  */
    argv++;
    argc--;
    while (argc--) {
      if (strcmp(*argv, "-lef") == 0) { //比较*argv所指向的字符串（即命令行参数）和"-lef"是否相等。
                                        //相等则表示当前参数是-lef，需要读取下一个参数作为LEF_FILE的值
        argv++;
        argc--;
        LEF_FILE = *argv;
        //cout <<"lef: " <<LEF_FILE <<endl;
      } else if (strcmp(*argv, "-def") == 0) {  //同上，看DEF路径
        argv++;
        argc--;
        DEF_FILE = *argv;
        REF_OUT_FILE = DEF_FILE;        //REF_OUT_FILE等于DEF_FILE
        //cout <<"def: " <<DEF_FILE <<endl;
      } else if (strcmp(*argv, "-guide") == 0) {  //同上，将guide路径记录到GUIDE_FILE
        argv++;
        argc--;
        GUIDE_FILE = *argv;
        //cout <<"guide: " <<GUIDE_FILE <<endl;
      } else if (strcmp(*argv, "-threads") == 0) {  //从命令行参数*argv（一个字符串）中读取一个整数，并将其存储到MAX_THREADS变量中
        argv++;
        argc--;
        sscanf(*argv, "%d", &MAX_THREADS);
        //cout <<"thread: " <<MAX_THREADS <<endl;
      } else if (strcmp(*argv, "-output") == 0) { //同上，输出文件路径为OUT_FILE
        argv++;
        argc--;
        OUT_FILE = *argv;
        //cout <<"output: " <<OUT_FILE <<endl;
      } else if (strcmp(*argv, "-verbose") == 0) {  //读取-verbose选项的参数值，并将其转换为整数存储在VERBOSE变量中
        argv++;
        argc--;
        VERBOSE = atoi(*argv);
        //cout <<"output: " <<OUT_FILE <<endl;
      } else {    //否则表示有不合法的命令行参数，报错
        cout <<"ERROR: Illegal command line option: " <<*argv <<endl;
        return 2;
      }
      argv++;
    }
  }
  
  FlexRoute router; //创建FlexRoute对象
  router.main();    //执行main命令
  
  high_resolution_clock::time_point t2 = high_resolution_clock::now();      //输出总执行时间
  duration<double> time_span = duration_cast<duration<double>>(t2 - t101)0;
  if (VERBOSE > 0) {
    cout <<endl <<"Runtime taken (hrt): " << time_span.count()    <<endl;
  }
  return 0;
}
