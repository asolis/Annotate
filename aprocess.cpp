
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


const Scalar Color::red  = Scalar(0,0,255);
const Scalar Color::blue = Scalar(255,0,0);
const Scalar Color::green= Scalar(0,255,0);
const Scalar Color::white= Scalar(255,255,255);
const Scalar Color::purple= Scalar(255,0,255);
const Scalar Color::yellow =  Scalar(0, 255, 255);
const Scalar Color::teal = Scalar(255, 255, 0);
const Scalar Color::orange = Scalar(22,134,241);



bool InputFactory::isVideoFile(const string &sequence)
{
    return (viva::Files::isFile(sequence));
}
bool InputFactory::isWebFile(const string &sequence)
{
    return sequence.substr(0,4) == "http";
}
bool InputFactory::isStringSequence(const string &sequence)
{
//    char res0[sequence.length() * 2];
//    char res1[sequence.length() * 2];
    char *res0 = new char[sequence.length() * 2];
    char *res1 = new char[sequence.length() * 2];
    sprintf(res0, sequence.c_str(), 0);
    sprintf(res1, sequence.c_str(), 1);
    string path0(res0);
    string path1(res1);
    return (viva::Files::isFile(path0) || viva::Files::isFile(path1));
}
bool InputFactory::isFolderSequence(const string &sequence)
{
    return (viva::Files::isDir(sequence));
}
bool InputFactory::isCameraID(const string &s)
{
    return !s.empty() && std::find_if(s.begin(),
                                      s.end(), [](char c) { return !std::isdigit(c); }) == s.end();
}


Ptr<Input> InputFactory::create(const string &sequence, const Size sz)
{
    if (isVideoFile(sequence))
    {
        return new VideoInput(sequence, sz);
    }
    if (isWebFile(sequence))
    {
        return new VideoInput(sequence, sz);
    }
    if (isStringSequence(sequence))
    {
        return new VideoInput(sequence, sz);
    }
    if (isFolderSequence(sequence))
    {
        return new ImageListInput(sequence, sz, -1, 0);
    }
    if (isCameraID(sequence))
    {
        return new VideoInput(stoi(sequence), sz);
    }
    return Ptr<Input>();
}

string  InputFactory::findInputGroundTruth(const string &sequence, const string &defaultname)
{
    string basename;
    if (isVideoFile(sequence))
    {
        viva::Files::getBasename(sequence, basename);
    }
    if (isWebFile(sequence))
    {
        //TODO
    }
    if (isStringSequence(sequence))
    {
        viva::Files::getBasename(sequence, basename);
    }
    if (isCameraID(sequence))
    {
        //TODO
    }

    if (isFolderSequence(sequence))
    {
        if (sequence.back() != viva::Files::PATH_SEPARATOR.front())
            basename = sequence + viva::Files::PATH_SEPARATOR;
        else
            basename = sequence;
    }

    return basename + defaultname;
}


float AnnotateProcess::closestPointToRay(const Point2f &pt, const Point2f &s, const Point2f &e)
{
    float A = pt.x - s.x;
    float B = pt.y - s.y;
    float C = e.x - s.x;
    float D = e.y - s.y;

    float dot = A*C + B*D;
    float len_sq = C*C + D*D;
    return dot/len_sq;
}
bool AnnotateProcess::acceptPolygon(const vector<Point2f> &polyg, int m)
{
    return  (m == AXIS_RECT && polyg.size() >= 2) ||
    (m == ROTA_RECT && polyg.size() >= 4) ||
    (m == POLY && polyg.size() >= 3);
}
void AnnotateProcess::swapPolygon(int i)
{
	swap(drawing, annotations[currentFrameN][i].annotateFrame);
	swap(mode, annotations[currentFrameN][i].mode);
	swap(currentActionType, annotations[currentFrameN][i].actionType);
}
bool AnnotateProcess::ptInsidePolygon(const Point2f &pt, const vector<Point2f> &polygon)
{
    return polygon.size() > 0 && pointPolygonTest(polygon, pt, false) > 0;
}
int AnnotateProcess::findIndexOfPolygonContainingPt(const Point2f &pt)
{
    int found = -1;
    for ( size_t i = 0; i < annotations[currentFrameN].size(); i++)
    {
        if (ptInsidePolygon(pt, annotations[currentFrameN][i].annotateFrame))
            return i;
    }
    return found;
}
Point2f AnnotateProcess::vectorPerpendicularToSegment(const Point2f &s, const Point2f &e)
{
    Point2f tmp = e - s;
    return Point2f( -tmp.y, tmp.x);
}

void Draw::displayPolygonNumber(Mat &image, const vector<Point2f> &pts, int number)
{
	if (pts.size() > 0)
	{
		auto ref = std::min_element(pts.begin(), pts.end(),
			[](const Point2f &p1, const Point2f &p2)
		{
			return p1.y < p2.y;
		});

		rectangle(image, *ref - Point2f(0, 10),
			*ref + Point2f(70, 0),
			Color::yellow, -1);

		string display = "";
		display += to_string(number);
		putText(image, display, *ref - Point2f(0, 2),
			FONT_HERSHEY_SIMPLEX, .3, Color::red, 1, CV_AA);
	}
}

void Draw::displayPolygonNumberNAction(Mat &image, const vector<Point2f> &pts, int number, string actionType)
{
    if (pts.size() > 0)
    {
        auto ref = std::min_element(pts.begin(), pts.end(),
                                  [](const Point2f &p1, const Point2f &p2)
                                  {
                                      return p1.y < p2.y;
                                  });

        rectangle(image, *ref - Point2f(0,10),
                         *ref + Point2f(70,0),
                        Color::yellow, -1);

		string display = "";
		display += to_string(number);
		display += " - ";
		display += actionType;
        putText(image, display, *ref - Point2f(0,2),
                FONT_HERSHEY_SIMPLEX, .3, Color::red, 1, CV_AA);
    }
}
void Draw::displayPolygon(Mat &image, const vector<Point2f> &pts, const Scalar &color, int thickness, bool close)
{
    int i = 0;
    for ( i = 0; i < ((int)pts.size() - 1); i++)
    {
        line(image, pts[i], pts[i + 1],
             color, thickness, CV_AA );
    }
    if (close && pts.size() > 0 )
        line(image, pts[i], pts[0],
             color, thickness, CV_AA );
}

void AnnotateProcess::getRRect(const Point2f &mPos, float ratio,
              Point2f &p1,
              Point2f &p2,
              Point2f &p3,
              Point2f &p4)
{
    Point2f vec = vectorPerpendicularToSegment( p1,
                                               p2);

    float t = closestPointToRay(mPos,
                                p2,
                                p2 + vec);
    p3 = p2 + t * vec;

    if (ratio > 0)
    {
        Point2f vec_ = p1 - p2;
        float mag      = cv::norm(vec_);
        double magP2P3 = cv::norm(p3 - p2);
        vec_ = (vec_/mag) * (1/ratio) * magP2P3;
        p1 = p2 + vec_;
    }

    p4 = p1 + t * vec;
}
void AnnotateProcess::getARect(const Point2f &mPos, float ratio,
              Point2f &p1,
              Point2f &p2,
              Point2f &p3,
              Point2f &p4)
{
    Point2f p = mPos;

    if (ratio > 0)
    {
        Point2f vec(1.0f, ratio);
        float t = closestPointToRay(mPos,
                                    p1,
                                    p1 + vec);
        p =  p1 + t * vec;
    }

    vector<Point2f> tmp = {p1, p};
    Rect area = boundingRect(tmp);
    area.width -= 1;
    area.height -= 1;

    p1 = area.tl();
    p2 = area.tl() + Point(area.width, 0);
    p3 = area.br();
    p4 = area.tl() + Point(0, area.height);
}
void AnnotateProcess::helpHUD(Mat &image)
{
    std::stringstream ss;
    ss <<  " (m) : ";

    int margin = 10;
    int fsize  = 20;
    int characterWidth  = 15;

    if (mode == AXIS_RECT)
        ss << "Axis Align Rectangle ";
    else if (mode == ROTA_RECT)
        ss << "Rotated Rectangle ";
    else
        ss << "Polygon ";

    if (ratio > 0 && mode != POLY)
        ss << "(ratio=" << ratio << ")";

    vector<string> help;
	help.push_back(" Frame Number : " + std::to_string(currentFrameN) + "/" + std::to_string(totalFrame-1));
	help.push_back(" Annotation Number : " + std::to_string(annotations.at(currentFrameN).size()));
	help.push_back(" Options:");
    help.push_back(ss.str());
    help.push_back(" (-) : Reduce ratio 0.1");
    help.push_back(" (+) : Increase ratio 0.1");
    help.push_back(" (h) : Toggle this help ");
    help.push_back(" (n) : Next frame");
	if(actionAnnotating)
		help.push_back(" (l) : Get the action class list");
    help.push_back(" (a) : Accept annotation");
    help.push_back(" (d) : Delete annotation");
    help.push_back(" (b) : Remove last point/annotation");
    help.push_back(" (c) : Clear all annotations");
    help.push_back(" (SPACE) : Pause/play video");
    help.push_back(" (SHIFT + Click) : Select Polygon");
    help.push_back(" (ESC) : Exit Annotate");


    Rect region(0, 0, characterWidth * fsize + margin,
                help.size() * fsize + margin);
    region &= Rect(0,0, image.cols, image.rows);

    GaussianBlur(image(region),
                 image(region),
                 Size(0,0), 5);

    for (size_t i = 0, h = fsize; i < help.size(); i++, h+= fsize)
    {
        putText(image, help[i], Point(fsize,h),
                FONT_HERSHEY_SIMPLEX, .5, Color::yellow, 1, CV_AA);
    }

}

void AnnotateProcess::helpActionHUB(Mat &image) 
{
	int margin = 10;
	int fsize = 15;
	int characterWidth = 13;

	vector<string> help;
	help.push_back(" Frame Number : " + std::to_string(currentFrameN) + "/" + std::to_string(totalFrame-1));
	help.push_back(" Annotation Number : " + std::to_string(annotations.at(currentFrameN).size()));
	help.push_back(" Options:");
	for (int i = 0; i < actionType.size(); i++)
	{
		help.push_back(" (" + std::to_string(i) + ") : " + actionType.at(i));
	}

	Rect region(0, 0, characterWidth * fsize + margin,
		help.size() * fsize + margin);
	region &= Rect(0, 0, image.cols, image.rows);

	GaussianBlur(image(region),
		image(region),
		Size(0, 0), 5);

	for (int i = 0, h = fsize; i < help.size(); i++, h += fsize)
	{
		if ((i-3) < 0)
		{
			putText(image, help[i], Point(fsize, h),
				FONT_HERSHEY_SIMPLEX, .5, Color::yellow, 1, CV_AA);
		} 
		else if (currentActionType == actionType.at(i-3))
		{
			putText(image, help[i], Point(fsize, h),
				FONT_HERSHEY_SIMPLEX, .5, Color::red, 1, CV_AA);
		}
		else
		{
			putText(image, help[i], Point(fsize, h),
				FONT_HERSHEY_SIMPLEX, .5, Color::yellow, 1, CV_AA);
		}
	}
}

Point2f AnnotateProcess::centroid(const vector<Point2f> &corners)
{
    Moments mu = moments(corners);
    return Point2f(mu.m10/mu.m00, mu.m01/mu.m00);
}

void AnnotateProcess::leftButtonDown(int x, int y, int flags)
{
    if  (flags &  EVENT_FLAG_CTRLKEY  || flags & EVENT_FLAG_SHIFTKEY)
    {
        mouseShift = Point2f(x,y);
        selection = findIndexOfPolygonContainingPt(mouseShift);
        if (selection >= 0)
        {
            drawing.clear();
            swapPolygon(selection);
        }

    }
    else
    {
        if (mode == AXIS_RECT && drawing.size() == 1)
        {
            vector<Point2f> _aRect(4);
            _aRect[0] = drawing.front();
            getARect(mousePos, ratio, _aRect[0], _aRect[1], _aRect[2], _aRect[3]);
            drawing.pop_back();
            drawing.push_back(_aRect[0]);
            drawing.push_back(_aRect[1]);
            drawing.push_back(_aRect[2]);
            drawing.push_back(_aRect[3]);

        }
        else if (mode == AXIS_RECT && drawing.size() >= 2)
        {
            newAnnotation();
            drawing.push_back(Point2f(x,y));
        }
        else if (mode == ROTA_RECT && drawing.size() >= 4)
        {
            newAnnotation();
            drawing.push_back(Point2f(x,y));
        }
        else if (mode == ROTA_RECT && drawing.size() == 2)
        {
            Point2f p3, p4;
            getRRect(mousePos, ratio, drawing.back(), drawing.front(), p3, p4);
            drawing.push_back(p4);
            drawing.push_back(p3);
        }
        else if (mode == ROTA_RECT && drawing.size() == 1)
        {
            drawing.push_back(Point2f(x,y));
        }
        else
        {
            drawing.push_back(Point2f(x,y));
        }

        mouseShift = centroid(drawing);
    }
};

void AnnotateProcess::mouseMove(int x, int y, int flags)
{
    if  ((flags &  EVENT_FLAG_CTRLKEY  || flags & EVENT_FLAG_SHIFTKEY) &&
         ptInsidePolygon(Point2f(x,y), drawing))
    {
        Point2f vector =  Point2f(x,y) - mouseShift;
        for(size_t i = 0; i < drawing.size(); i++ )
            drawing[i]+= vector;

        mouseShift = mouseShift + vector;
    }
    mousePos = Point2f(x,y);
};

void AnnotateProcess::keyboardInput(int key)
{
    if (key == 'b' || key == 'B')
    {
        if (drawing.empty())
        {
            if (!annotations[currentFrameN].empty())
            {
                annotations[currentFrameN].pop_back();
                if (tracking && !trackers.empty())
                    trackers.pop_back();
            }
        }
        if (mode == AXIS_RECT && drawing.size() == 4)
        {
            drawing.pop_back();
            drawing.pop_back();
        }
        if (mode == ROTA_RECT && drawing.size() == 4)
        {
            drawing.pop_back();
        }
        if (!drawing.empty())
        {
            drawing.pop_back();
        }

    }
    if (key == 'c' || key == 'C')
    {
        drawing.clear();

        while (!annotations[currentFrameN].empty())
            annotations[currentFrameN].pop_back();

        if (tracking)
            trackers.clear();
    }
    if (key == 'h' || key == 'H')
    {
        showHelp = !showHelp;
        showActionHelp = false;
    }

    if (key == 'd' || key == 'D')
    {
        remAnnotation();
        //drawing.clear();
    }

    if (key == 'a' || key == 'A')
    {
        newAnnotation();
    }

    if (key == 'm' || key == 'M')
    {
        newAnnotation();
        mode = (++mode)%3;
    }
    if (key == '-')
    {
        ratio = max(ratio - .1f , 0.f);
    }
    if (key == '+')
    {
        ratio += .1;
    }

    if (actionAnnotating && (key == 'l' || key == 'L'))
    {
        // add the action annotation code
        showActionHelp = !showActionHelp;
        showHelp = false;
    }

    for (int i = 0; i < actionType.size(); i++)
    {
        if (key == ('0' + i))
            currentActionType = actionType.at(i);
    }
};

void AnnotateProcess::newTracker()
{
    if (tracking)
    {
        Ptr<SKCFDCF> tmp = new SKCFDCF();
        Rect area = boundingRect(drawing);
        area.width -= 1;
        area.height -= 1;
        tmp->initialize(currentFrame,area);

        if (selection < 0)
        {
            trackers.push_back(tmp);
        }
        else if (selection >= 0 && selection < trackers.size())
        {
            trackers[selection] = tmp;
        }
    }
}

void AnnotateProcess::remTracker(int i)
{
    if (tracking && i >= 0 && i < trackers.size())
    {
        trackers.erase(trackers.begin() + i);
    }
}

void AnnotateProcess::remAnnotation()
{
    if (selection >= 0 && selection < annotations[currentFrameN].size())
    {
        annotations[currentFrameN].erase(annotations[currentFrameN].begin() + selection);
        trackers.erase(trackers.begin() + selection);
    }
    drawing.clear();
    selection = -1;
}

void AnnotateProcess::newAnnotation()
{
    if (acceptPolygon(drawing, mode))
    {
        Annotation tmp;
        if (selection < 0)
        {
            tmp.annotateFrame = drawing;
            tmp.mode = mode;
            tmp.actionType = currentActionType;
            tmp.ID = peopleAmount++;

            annotations[currentFrameN].push_back(tmp);
        }
        else if (selection >= 0 && selection < annotations[currentFrameN].size() )
        {
            annotations[currentFrameN][selection].annotateFrame = drawing;
            annotations[currentFrameN][selection].mode = mode;
            annotations[currentFrameN][selection].actionType = currentActionType;
        }
        newTracker();
        drawing.clear();
        selection = -1;

    }
}

bool AnnotateProcess::readActionTypeFile(const string &filename)
{
    string line;
    ifstream myfile(filename);
    if (myfile.is_open())
    {
        while (getline(myfile, line))
        {
            actionType.push_back(line);
        }
        if (actionType.size() != 0)
        {
            currentActionType = actionType.at(0);
        }
        myfile.close();
        return true;
    }
    else
    {
        cout << "Unable to open file";
        return false;
    }
}

int CSVAnnotateProcess::parse(const string &filename, vector<vector<Annotation>> &ann)
{
    ifstream file;
    file.open(filename);
    string line;

    //    vector<vector<vector<Point2f>>> _data;
    //
    //    _data.clear();

    ann.clear();

    int currentFrameN = 0;

    while (file)
    {
        if (!getline(file, line)) break;

        currentFrameN++;

        istringstream streamline(line);
        string annotation;

        //        vector<vector<Point2f>> _line;
        //        _data.push_back(_line);

        vector<Annotation> list;
        while (getline(streamline, annotation, '|'))
        {
            Annotation tmp;

            istringstream streamannot(annotation);
            string x, y;
            while (getline(streamannot, x, ',') &&
                   getline(streamannot, y, ',') )
            {
                tmp.annotateFrame.push_back(Point2f(atof(x.c_str()),
                                                    atof(y.c_str())));
            }
            list.push_back(tmp);
            streamannot.clear();
        }
        ann.push_back(list);
        streamline.clear();

    }
    file.close();
    return (currentFrameN)?currentFrameN - 1 : currentFrameN;
}

int CSVAnnotateProcess::read(const string &filename)
{

    drawing.clear();
    trackers.clear();
    return parse(filename, annotations);
}

void CSVAnnotateProcess::write(const string &filename, const float scaleX, const float scaleY)
{
    ofstream file;
    file.open(filename);
    for (size_t i = 0; i < annotations.size(); i++)
    {

        for (size_t j = 0; j < annotations[i].size(); j++)
        {
            for (size_t k = 0; k < annotations[i][j].annotateFrame.size(); k++)
            {

                file << annotations[i][j].annotateFrame[k].x * scaleX << ", "
                << annotations[i][j].annotateFrame[k].y * scaleY;

                if ( k != (annotations[i][j].annotateFrame.size() - 1))
                    file << ", ";
            }
            if ( j != (annotations[i].size() - 1))
                file << "|";
        }
        file << endl;
    }
}


void XMLAnnotateProcess::write(const string &filename, const float scaleX, const float scaleY)
{
    ofstream file;
    file.open(filename);
    xml_document<> doc;

    // add the xml declaration
    xml_node<>* decl = doc.allocate_node(node_declaration);
    decl->append_attribute(doc.allocate_attribute("version", "1.0"));
    decl->append_attribute(doc.allocate_attribute("encoding", "utf-8"));
    doc.append_node(decl);

    // create the root node
    xml_node<>* root = doc.allocate_node(node_element, "video");
    doc.append_node(root);
    allocateAttrToNodeXML(doc, root, "totalPeopleAmount", std::to_string(peopleAmount));

    for (size_t i = 0; i < annotations.size(); i++)
    {
        xml_node<> *node = doc.allocate_node(node_element, "frame");
        root->append_node(node);
        // write the frame number
        allocateAttrToNodeXML(doc, node, "frameNumber", std::to_string(i));

        for (size_t j = 0; j < annotations[i].size(); j++)
        {
            xml_node<> *sub_node = doc.allocate_node(node_element, "box");
            node->append_node(sub_node);

            // add the box number of one frame
            // allocateAttrToNodeXML(doc, sub_node, "boxNumber", std::to_string(j));

            // add the person ID
            allocateAttrToNodeXML(doc, sub_node, "personID", std::to_string(annotations[i][j].ID));

            // add the action type for this box
            allocateAttrToNodeXML(doc, sub_node, "actionType", annotations[i][j].actionType);

            for (size_t k = 0; k < annotations[i][j].annotateFrame.size(); k++)
            {
                std::string pointNum = "pointNumber";
                pointNum += std::to_string(k);
                char *pointNumChar = doc.allocate_string(pointNum.c_str());

                std::string pointValue = std::to_string(annotations[i][j].annotateFrame[k].x * scaleX);
                pointValue += ",";
                pointValue += std::to_string(annotations[i][j].annotateFrame[k].y * scaleY);

                // add the pointNumber information
                allocateAttrToNodeXML(doc, sub_node, pointNumChar, pointValue);
            }
        }
    }

    // print out the xml document
    file << doc;
    file.close();
}

void XMLAnnotateProcess::allocateAttrToNodeXML(xml_document<> &doc, xml_node<> *node, const char * name, string value)
{
    // convert string to char
    char *CharTmp = doc.allocate_string(value.c_str());
    xml_attribute<> *attr= doc.allocate_attribute(name, CharTmp);
    node->append_attribute(attr);
}

int XMLAnnotateProcess::parse(const string &filename, vector<vector<Annotation>> &ann)
{
    string input_xml;
    std::string line;
    std::ifstream in(filename);
    // read the file into input_XML
    while (getline(in, line))
        input_xml += line;

    if (input_xml == "")
        return 0;

    vector<char> xml_copy(input_xml.begin(), input_xml.end());
    xml_copy.push_back('\0');

    xml_document<> doc;
    doc.parse<parse_no_data_nodes>(&xml_copy[0]);

    ann.clear();

    int currentFrameN = 0;

    // parse the root node
    xml_node<>* cur_node = doc.first_node("video");
    int peopleAmount = atoi(cur_node->first_attribute()->value());

    // frame loop
    for (xml_node<> *frame_node = cur_node->first_node(); frame_node; frame_node = frame_node->next_sibling())
    {
        vector<Annotation> instance;
        currentFrameN = atoi(frame_node->first_attribute()->value());

        // box loop
        for (xml_node<> *box_node = frame_node->first_node(); box_node; box_node = box_node->next_sibling())
        {
            Annotation tmp;
            vector<Point2f> pts;
            for (xml_attribute<> *point = box_node->first_attribute(); point; point = point->next_attribute())
            {
                if (strcmp(point->name(), "actionType") == 0)
                {
                    tmp.actionType = point->value();
                    continue;
                }

                if (strcmp(point->name(), "mode") == 0)
                {
                    tmp.mode = atoi(point->value());
                    continue;
                }

                if (strcmp(point->name(), "personID") == 0)
                {
                    tmp.ID = atoi(point->value());
                    continue;
                }

                std::string tmp(point->name());
                if (tmp.find("pointNumber") == string::npos) { continue; }
                char *value = point->value();
                // parse the value
                char value_array[50];
                strncpy(value_array, value, sizeof(value_array));
                char *single_axis = strtok(value_array, ",");
                std::vector<char*> xy;
                while (NULL != single_axis)
                {
                    xy.push_back(single_axis);
                    single_axis = strtok(NULL, ",");
                }
                pts.push_back(Point2f(atof(xy[0]), atof(xy[1])));
            }
            tmp.annotateFrame = pts;
            instance.push_back(tmp);
        }
        
        ann.push_back(instance);
    }
    
    return currentFrameN;
}

int XMLAnnotateProcess::read(const string &filename)
{
    drawing.clear();
    trackers.clear();
    return parse(filename, annotations);
}
