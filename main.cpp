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


size_t read(CommandLineParserExt &parser,
            vector<Ptr<Input>> &inputs,
            vector<Ptr<XMLAnnotateProcess>> &processes)
{
    ifstream file(parser.get<string>("i"));
    vector<char> buffer((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
    buffer.push_back('\0');
    
    xml_document<> doc;
    xml_node<>* sequence;
    doc.parse<0>(&buffer[0]);
    
    size_t maxID = 0;
    sequence = doc.first_node(NODE::SEQ.c_str());
    for (xml_node<> * sequence = doc.first_node();
                      sequence;
                      sequence = sequence->next_sibling())
    {
        
//        int w, h;
//        size_t id;
//        vector<vector<Annotation>> ann;
//        vector<string> filenames;
        size_t sMaxID = 9;//XMLAnnotateProcess::readSequence(*sequence, ann, filenames, id, w, h);
        maxID = (sMaxID > maxID)? sMaxID : maxID;
        
    }
    file.close();
    return maxID;
}


void write(const string &filename,
           vector<string> &actions,
           vector<Ptr<XMLAnnotateProcess>> &processes)
{
    xml_document<> doc;
    
    XMLAnnotateProcess::writeHeader(doc);
    XMLAnnotateProcess::writeActions(doc, actions);
    
    for (size_t i = 0; i < processes.size(); i++)
    {
        processes[i]->writeSequence(doc);
    }
    ofstream file;
    file.open(filename);
    file << doc;
    file.close();
}



int main(int argc, const char * argv[])
{

    const String keys =
        "{help h            |           | print this message}"
        "{@sequence         |           | list of url, file, folder, sequence}"
        "{m method          |a          | choices =  p | r | a  (i.e., poligon, rotated rect, axis aligned rect}"
        "{W width           |-1         | scale input using this width, keeps aspect ratio annotations will be transformed to the initial image size}"
        "{H height          |-1         | scale input using this height, keeps aspect ratio annotations will be transformed to the initial image size}"
        "{r ratio           |-1         | ratio = height/width, -1 no constraints}"
        "{t track           |on         | will generate a prediction position for the next frame}"
		"{i import          |           | import xml file (only for image list input)}"
		"{a actionType      |           | import actionType file}"
		"{o output          |           | filename for annnotation results}"
    ;

    CommandLineParserExt parser(argc, argv, keys);

    /** Print Help **/
    if (parser.has("h"))
        parser.printMessage();

    /** specify the polygonal annotation to start with **/
    string mname    = parser.get<string>("m");
    int method = AnnotateProcess::AXIS_RECT;
    if (mname == "r")
        method = AnnotateProcess::ROTA_RECT;
    else if (mname == "p")
        method = AnnotateProcess::POLY;
    


    vector<Ptr<Input>> _inputs;
    vector<Ptr<XMLAnnotateProcess>> _process;
    vector<Ptr<ProcessFrame>> proc;

    
    size_t _startFrame  = 0;
    size_t maxID = 0;


    vector<string> actions;
    if (parser.has("a"))
        AnnotateProcess::readActionTypeFile(parser.get<string>("a"),
                                            actions);

    if (parser.has("i"))
        maxID = read(parser, _inputs, _process);



    
    /** create the list of inputs and annotation process **/
    for (size_t i = 0; i < parser.n_positional_args(); i++)
    {
        Ptr<Input> input = InputFactory::create(parser.get<string>(i),
                                Size(parser.get<int>("W"),
                                     parser.get<int>("H")));

        _inputs.push_back(input);

        Ptr<XMLAnnotateProcess> process;

        process = new XMLAnnotateProcess(parser.get<string>(i),
                                         input->getWidth(),
                                         input->getHeight(),
                                         i,
                                         parser.get<float>("r"),
                                         method,
                                         parser.has("t"),
                                         parser.has("a"),
                                         input->totalFrames());

        process->setActions(actions);
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
        write(parser.get<string>("o"), actions, _process);


    return 0;
}




