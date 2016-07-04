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
#include "clp.hpp"
#include "aprocess.h"

using namespace viva;


int main(int argc, const char * argv[])
{

	const String keys =
		"{help h            |                | print this message}"
		"{@sequences        |                | folder containing the sequence}"
		"{i input           |                | input annotated file}"
        "{s                 |0               | sequence id (default first sequence in xml)}"
		"{o output          |output          | folder name for output}"
		"{p pattern         |%06d%03d.png    | output filename pattern}"
		"{m masked          |                | masking output images if not axis align}"
    ;

    CommandLineParserExt parser(argc, argv, keys);
    cout << parser.has("i") << endl;
    if (!parser.has("i") || parser.has("h"))
    {
        parser.printMessage();
        return 0;
    }
    
    xml_document<> doc;
    vector<char> buffer;
    XMLAnnotateProcess::readXML(parser.get<string>("i"), doc, buffer);
    vector<string> actns;
    xml_node<>* root   = doc.first_node(NODE::ANNOTATE.c_str());
    xml_node<>* aNode  = root->first_node(NODE::ACTIONS.c_str());
    XMLAnnotateProcess::readActions(*aNode, actns);
    
    xml_node<> *sequence = root->first_node(NODE::SEQ.c_str());
    for (size_t skeep = 0; skeep < parser.get<int>("id"); skeep++)
        sequence = sequence->next_sibling();
    
    vector<string> filenames;
    int width, height;
    string type;
    size_t sID, cFC;
    
    XMLAnnotateProcess::readSequenceMetadata(*sequence, sID, cFC, type, width, height);
    XMLAnnotateProcess::readSequenceFilenames(*sequence, filenames);
    Ptr<Input> input = InputFactory::create(parser.get<string>(0), filenames, Size(width, height));
    vector<vector<Annotation>> _gtruth;
    XMLAnnotateProcess::readSequenceAnnotations(*sequence, _gtruth);
    
    
    string gfile = parser.get<string>("g");
	string xmlgfile = parser.get<string>("x");

   
    string outputFolder = parser.get<string>("o");
    if (!viva::Files::isDir(outputFolder))
        viva::Files::makeDir(outputFolder);

    string pattern = parser.get<string>("p");
    bool isMasked  = parser.has("m");


    auto processFrame = [&_gtruth, &outputFolder, &pattern, &isMasked]
        (const size_t frameN, const Mat &input, Mat &output){

        input.copyTo(output);
        if (frameN < _gtruth.size())
        {
            char buffer[100];
            Mat mask;
            if (isMasked)
                mask = Mat::zeros(input.rows, input.cols, input.type());

            for (size_t i = 0; i < _gtruth[frameN].size(); i++)
            {
                Annotation &annotation = _gtruth[frameN][i];
                vector<Point2f> &pts =  annotation.area;
                Draw::displayPolygon(output, pts, Color::yellow, 2, true);
                Draw::displayPolygonInfo(output, to_string(i+1), pts);

                Rect tmp  = boundingRect(pts);
                tmp.width -= 1;
                tmp.height-= 1;
                tmp = tmp & Rect(0,0, input.cols, input.rows);


                if (isMasked)
                {
                    vector<Point> pts2;
                    for (size_t k = 0; k < pts.size(); k++)
                        pts2.push_back(Point(pts[k].x, pts[k].y));

                    fillConvexPoly(mask, &pts2[0], pts.size(), Color::white);
                }

                sprintf(buffer, pattern.c_str(), (int)frameN, (int)i);
                string fullname = outputFolder +
                        viva::Files::PATH_SEPARATOR + string(buffer);

                if (isMasked)
                {
                    Mat masked;
                    input.copyTo(masked, mask);

                    imwrite(fullname, masked(tmp));
                }
                else
                    imwrite(fullname, input(tmp));
            }
        }
    };

    Processor processor;
    processor.setInput(input);
    processor.setProcess(processFrame);
    processor.listenToMouseEvents();
    processor.listenToKeyboardEvents();
    processor.run();



    return 0;
}