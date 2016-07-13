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
        "{@sequence         |           | folder containing a sequence of images}"
        "{m method          |a          | choices =  p | r | a  (i.e., poligon, rotated rect, axis aligned rect}"
        "{W width           |-1         | scale input to the desired width value. Image aspect ratio is kept}"
        "{H height          |-1         | scale input to the desired height value. Image aspect ratio is kept}"
        "{r ratio           |-1         | annotation constraint ratio = height/width. Value of -1  means no constraints}"
        "{t track           |on         | will generate a prediction position for the next frame}"
		"{i import          |           | import xml file}"
		"{a actionType      |           | import action file with an action per line}"
        "{s skip            |0          | skip number of frames}"
		"{o output          |           | output xml file}"
    ;

    CommandLineParserExt parser(argc, argv, keys);

    /** Print Help **/
    if (parser.has("h"))
        parser.printMessage();

    vector<Ptr<Input>> _inputs;
    vector<Ptr<XMLAnnotateProcess>> _process;
    
    size_t _startFrame  = parser.get<int>("s");

    vector<string> actions;
    vector<pair<Point,Point>> matches;

    if (parser.has("i"))
        InputFactory::load(parser, actions, matches, _inputs, _process);
    else
        InputFactory::initialize(parser, actions, _inputs,  _process);




    vector<Ptr<ProcessFrame>> proc (_process.begin(), _process.end());
    
    Ptr<MatchingProcess> _mp   = new MatchingProcess(_process, matches);
    Ptr<ProcessFrame> matching = _mp;
    
	MultipleProcess processor;
    if (parser.has("s"))
        processor.setStartFrame(parser.get<int>("s"));

    processor.setInput(_inputs);
	processor.setStartFrame(_startFrame);
	processor.setProcess(proc);
    processor.setMachingProcess(matching);
    processor.listenToMouseEvents();
    processor.listenToKeyboardEvents();
    processor.run();

    if (parser.has("o"))
        InputFactory::write(parser.get<string>("o"), actions, _mp->matches, _process);


    return 0;
}




