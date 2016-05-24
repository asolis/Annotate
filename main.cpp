/**************************************************************************************************
 **************************************************************************************************
 
     BSD 3-Clause License (https://www.tldrlegal.com/l/bsd3)
     
     Copyright (c) 2015 Andrés Solís Montero <http://www.solism.ca>, All rights reserved.
     
     
     Redistribution and use in source and binary forms, with or without modification,
     are permitted provided that the following conditions are met:
     
     1. Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright notice,
        this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.
     3. Neither the name of the copyright holder nor the names of its contributors
        may be used to endorse or promote products derived from this software
        without specific prior written permission.
     
     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
     AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
     IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
     LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
     THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
     OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
     OF THE POSSIBILITY OF SUCH DAMAGE.
 
 **************************************************************************************************
 **************************************************************************************************/

#include "aprocess.h"

using namespace viva;

int main(int argc, const char * argv[])
{

    const String keys =
        "{help h            |           | print this message}"
        "{@sequence         |           | url, file, folder, sequence}"
        "{m method          |p          | choices =  p | r | a  (i.e., poly, rotated rect, axis aligned rect}"
        "{W width           |-1         | scale input using this width, keeps aspect ratio annotations will be transformed to the initial image size}"
        "{H height          |-1         | scale input using this height, keeps aspect ratio annotations will be transformed to the initial image size}"
        "{r ratio           |-1         | ratio = height/width, -1 no constraints}"
        "{t track           |e          | choices =  e | d | n  (i.e., enable, disable, and new). The 'e' enable option will generate a proposal position for next frame. Disable 'd' option will keep the same position from previous frame. New 'n' will clear the annotations from previous frame}"
        "{o output          |           | filename for annnotation results}"
		"{ox output XML		|			| filename for xml file annotation results}"

    ;

    CommandLineParser parser(argc, argv, keys);

    if (parser.has("h"))
        parser.printMessage();

    string sequence = parser.get<string>(0);
    string mname    = parser.get<string>("m");

    Ptr<Input> input = InputFactory::create(sequence,
                                    Size(parser.get<int>("W"),
                                         parser.get<int>("H")));

    int method = AnnotateProcess::POLY;
    if (mname == "a")
        method = AnnotateProcess::AXIS_RECT;
    else if (mname == "r")
        method = AnnotateProcess::ROTA_RECT;

    string track = parser.get<string>("t");
    bool continuity = (track == "d" || track == "e");
    bool tracking   = (track == "e");
    Ptr<AnnotateProcess> process =
                new AnnotateProcess(parser.get<float>("r"),
                                    method, continuity, tracking);

    Processor processor;
    processor.setInput(input);
    Ptr<ProcessFrame> proc = process;
    processor.setProcess(proc);
    processor.startPaused();
    processor.listenToMouseEvents();
    processor.listenToKeyboardEvents();
    processor.run();


	Size org = input->getOrgSize();
	Size cur = Size(input->getWidth(), input->getHeight());
    if (parser.has("o"))
    {        
        string annotations = parser.get<string>("o");
        process->writeAnnotations(annotations,
                                  (float)org.width/(float)cur.width,
                                  (float)org.height/(float)cur.height);
    }

	if (parser.has("ox"))
	{
		string annotations = parser.get<string>("ox");
		process->writeXMLAnnotations(annotations,
			(float)org.width / (float)cur.width,
			(float)org.height / (float)cur.height);
	}

    return 0;
}
