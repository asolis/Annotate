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
#include "clp.hpp"
#include "mprocessor.hpp"

using namespace viva;

int main(int argc, const char * argv[])
{

    const String keys =
        "{help h            |           | print this message}"
        "{@sequence         |           | list of url, file, folder, sequence}"
        "{m method          |p          | choices =  p | r | a  (i.e., poly, rotated rect, axis aligned rect}"
        "{W width           |-1         | scale input using this width, keeps aspect ratio annotations will be transformed to the initial image size}"
        "{H height          |-1         | scale input using this height, keeps aspect ratio annotations will be transformed to the initial image size}"
        "{r ratio           |-1         | ratio = height/width, -1 no constraints}"
        "{t track           |e          | choices =  e | d | n  (i.e., enable, disable, and new). The 'e' enable option will generate a proposal position for next frame. Disable 'd' option will keep the same position from previous frame. New 'n' will clear the annotations from previous frame}"
		"{i import          |           | import xml file (only for image list input)}"
		"{a actionType      |           | import actionType file}"
		"{o output          |           | filename for annnotation results}"
		"{f format          |xml        | choices = xml | csv }"

    ;

    CommandLineParserExt parser(argc, argv, keys);

    /** Print Help **/
    if (parser.has("h"))
        parser.printMessage();

    /** specify the polygonal annotation to start with **/
    string mname    = parser.get<string>("m");
    int method = AnnotateProcess::POLY;
    if (mname == "a")
        method = AnnotateProcess::AXIS_RECT;
    else if (mname == "r")
        method = AnnotateProcess::ROTA_RECT;

    /** enable continuity or tracking **/
    string track = parser.get<string>("t");
    bool continuity = (track == "d" || track == "e");
    bool tracking   = (track == "e");

    vector<Ptr<Input>> _inputs;
	vector<int> _startFrame;
    vector<Ptr<AnnotateProcess>> _process;
    vector<Ptr<ProcessFrame>> proc;

    /** create the list of inputs and annotation process **/
    for (size_t i = 1; i <= parser.n_positional_args(); i++)
    {
        Ptr<Input> input = InputFactory::create(parser.get<string>(i),
                                Size(parser.get<int>("W"), parser.get<int>("H")));

        _inputs.push_back(input);

        Ptr<AnnotateProcess> process;
        if (parser.get<string>("f") == "xml")
        {
            process = new XMLAnnotateProcess(parser.get<float>("r"),
                                             method, continuity,
                                             tracking, parser.has("a"),
                                             input->totalFrames());
        }
        else
        {
            process = new CSVAnnotateProcess(parser.get<float>("r"),
                                             method, continuity,
                                             tracking, parser.has("a"),
                                             input->totalFrames());
        }

        int latestFrame = 0;
        if (parser.has("i"))
            latestFrame = process->read(parser.get<string>("i"));
		_startFrame.push_back(latestFrame);

        if (parser.has("a"))
            process->readActionTypeFile(parser.get<string>("a"));

        _process.push_back(process);
        proc.push_back(process);
    }


	MultipleProcess processor;
    processor.setInput(_inputs);
	processor.setStartFrame(_startFrame);
	processor.setProcess(proc);
    processor.listenToMouseEvents();
    processor.listenToKeyboardEvents();
    processor.run();

    if (parser.has("o"))
    {
        for (size_t i = 0; i < parser.n_positional_args(); i++)
        {
            Ptr<Input> input = _inputs[i];
            Size org = input->getOrgSize();
            Size cur = Size(input->getWidth(), input->getHeight());

            Ptr<AnnotateProcess> process = _process[i];
            string annotations = parser.get<string>("o");
            process->write(annotations,
                                  (float)org.width/(float)cur.width,
                                  (float)org.height/(float)cur.height);
        }
    }


    return 0;
}
