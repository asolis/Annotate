
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


int AnnotateProcess::peopleAmount = 0;

bool InputFactory::filenames(const string &folder, vector<string> &filenames)
{
    filenames.clear();
    if (InputFactory::isFolderSequence(folder))
    {
        Files::listImages(folder, filenames);
        
        
        for (size_t i = 0; i < filenames.size(); i++)
        {
           size_t pos =  filenames[i].find_last_of(viva::Files::PATH_SEPARATOR);
           if (pos != string::npos)
               filenames[i] = filenames[i].substr(pos+ 1);
        }
        
        return true;
    }
    return false;
    
}
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
	swap(drawing, annotations[currentFrameN][i].area);
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
        if (ptInsidePolygon(pt, annotations[currentFrameN][i].area))
            return i;
    }
    return found;
}
Ptr<SKCFDCF> AnnotateProcess::initTracker(Mat frame, Rect area) 
{
	Ptr<SKCFDCF> tmp = new SKCFDCF();
	area.width -= 1;
	area.height -= 1;
	tmp->initialize(frame, area);
	return tmp;
}
Point2f AnnotateProcess::vectorPerpendicularToSegment(const Point2f &s, const Point2f &e)
{
    Point2f tmp = e - s;
    return Point2f( -tmp.y, tmp.x);
}



void Draw::displayPolygonInfo(cv::Mat &image,
                              const string &info,
                              const vector<Point2f> &pts,
                              const Scalar &background,
                              const Scalar &foreground)
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
                         background, -1);
        
        putText(image, info, *ref - Point2f(0,2),
                FONT_HERSHEY_SIMPLEX, .3, foreground, 1, CV_AA);
    }
}

void Draw::displayAnnotation(cv::Mat &image,
                             Annotation &ann,
                             const Scalar &background,
                             const Scalar &foreground,
                             int thickness,
                             bool close)
{
    displayPolygon(image, ann.area, background, thickness, close);
    string info = ((ann.tracking)?"[T] ":"") + to_string(ann.ID) +
                  ": " + ann.actionType;
    
    displayPolygonInfo(image, info, ann.area, background, foreground);

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
	help.push_back(" Options:");
    help.push_back(ss.str());
    help.push_back(" (h) : Toggle this help ");
    help.push_back(" (n) : Next frame");
    help.push_back(" (p) : Previous frame");
    help.push_back(" (a) : Accept annotation");
    help.push_back(" (d) : Delete annotation");
    help.push_back(" (b) : Remove last point/annotation");
    help.push_back(" (c) : Clear all annotations");
    help.push_back(" (SPACE) : Pause/play video");
    help.push_back(" (SHIFT + Click) : Select Polygon");
    help.push_back(" (-) : Reduce ratio 0.1");
    help.push_back(" (+) : Increase ratio 0.1");
    help.push_back(" (ESC) : Exit Annotate");


    Rect region(0, 0, (characterWidth * fsize + margin) * 2,
                      help.size()    * fsize + margin);
    region &= Rect(0,0, image.cols, image.rows);

    GaussianBlur(image(region),
                 image(region),
                 Size(0,0), 5);

    for (size_t i = 0, h = fsize; i < help.size(); i++, h+= fsize)
    {
        putText(image, help[i], Point(fsize,h),
                FONT_HERSHEY_SIMPLEX, .5, Color::yellow, 1, CV_AA);
    }
    
    if (actionAnnotating)
    {
        vector<string> actionList;
        actionList.push_back("");
        actionList.push_back("Actions:");
        
        int header = actionList.size();
        
        for (size_t i = 0; i < actionType.size(); i++)
            actionList.push_back(" (" + std::to_string(i) + ") : " + actionType.at(i));
        
        int leftMargin = (characterWidth * fsize + margin * 2);
        
        for (int i = 0, h = fsize; i < actionList.size(); i++, h += fsize)
        {
            if ((i-header) < 0)
            {
                putText(image, actionList[i], Point(leftMargin, h),
                        FONT_HERSHEY_SIMPLEX, .5, Color::yellow, 1, CV_AA);
            }
            else if (currentActionType == actionType.at(i-header))
            {
                putText(image, actionList[i], Point(leftMargin, h),
                        FONT_HERSHEY_SIMPLEX, .5, Color::red, 1, CV_AA);
            }
            else
            {
                putText(image, actionList[i], Point(leftMargin, h),
                        FONT_HERSHEY_SIMPLEX, .5, Color::yellow, 1, CV_AA);
            }
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
			selection = -1;
			if (!annotations[currentFrameN].empty())
			{
				bool deleted = false;
				// remove the empty annotation first 
				for (std::vector<Annotation>::iterator it = annotations[currentFrameN].begin();
					it != annotations[currentFrameN].end();)
				{
					if (it->area.size() == 0)
					{
						deleted = true;
						it = annotations[currentFrameN].erase(it);
					}
					else
					{
						++it;
					}
				}
				// pop up the latest annotation
				if (!deleted)
				{
					annotations[currentFrameN].pop_back();
				}

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

    }
    if (key == 'h' || key == 'H')
    {
        showHelp = !showHelp;
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

    for (int i = 0; i < actionType.size(); i++)
    {
        if (key == ('0' + i))
            currentActionType = actionType.at(i);
    }
};

void AnnotateProcess::remAnnotation()
{
    if (selection >= 0 && selection < annotations[currentFrameN].size())
    {
        annotations[currentFrameN].erase(annotations[currentFrameN].begin() + selection);
    }
    drawing.clear();
    selection = -1;
}
bool AnnotateProcess::annotationIsSelected()
{
    return (selection >= 0 && selection < annotations[currentFrameN].size());
}

void AnnotateProcess::newAnnotation()
{
    if (acceptPolygon(drawing, mode))
    {
        Annotation tmp;
		//Rect area = boundingRect(drawing);
        if (selection < 0)
        {
            tmp.area = drawing;
            tmp.mode = mode;
            tmp.actionType = currentActionType;
            tmp.ID = peopleAmount++;
            tmp.tracking = true;
            annotations[currentFrameN].push_back(tmp);
        }
        else if (selection >= 0 && selection < annotations[currentFrameN].size() )
        {
            annotations[currentFrameN][selection].area = drawing;
            annotations[currentFrameN][selection].mode = mode;
            annotations[currentFrameN][selection].actionType = currentActionType;
			//annotations[currentFrameN][selection].tracker = initTracker(currentFrame, area);
        }
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


string XMLAnnotateProcess::ATTR::VERSION = "version";
string XMLAnnotateProcess::ATTR::FILENAME = "filename";
string XMLAnnotateProcess::ATTR::TYPE     = "type";
string XMLAnnotateProcess::ATTR::ENCODING = "encoding";
string XMLAnnotateProcess::ATTR::FRAMEC   = "frameCount";
string XMLAnnotateProcess::ATTR::ID       = "id";
string XMLAnnotateProcess::ATTR::TARGETC  = "targetCount";
string XMLAnnotateProcess::ATTR::ACTION   = "action";
string XMLAnnotateProcess::ATTR::TARGET1  = "target1";
string XMLAnnotateProcess::ATTR::TARGET2  = "target2";
string XMLAnnotateProcess::ATTR::WIDTH    = "width";
string XMLAnnotateProcess::ATTR::HEIGHT   = "height";

string XMLAnnotateProcess::ATTR::T_FOLDER = "folder";
string XMLAnnotateProcess::ATTR::T_FILE   = "file";

string XMLAnnotateProcess::NODE::SEQ    = "sequence";
string XMLAnnotateProcess::NODE::FRAME  = "frame";
string XMLAnnotateProcess::NODE::TARGET = "target";
string XMLAnnotateProcess::NODE::LOCATION   = "location";
string XMLAnnotateProcess::NODE::MATCHING = "matching";
string XMLAnnotateProcess::NODE::MATCH    = "match";


void XMLAnnotateProcess::writeHeader(xml_document<> &doc)
{
    // add declaration
    xml_node<>* decl = doc.allocate_node(node_declaration);
    decl->append_attribute(attribute(doc, ATTR::VERSION, "1.0"));
    decl->append_attribute(attribute(doc, ATTR::ENCODING, "utf-8"));
    doc.append_node(decl);
}
void XMLAnnotateProcess::writeSequence(xml_document<> &doc)
{
    vector<string> filenames;
    bool folder = InputFactory::isFolderSequence(input);
    if (folder)
    {
        InputFactory::filenames(input, filenames);
        folder = filenames.size() == annotations.size();
    }
    
    // create the sequence node
    xml_node<>* sequence = node(doc, NODE::SEQ);
    sequence->append_attribute(attribute(doc, ATTR::FRAMEC, to_string(annotations.size())));
    sequence->append_attribute(attribute(doc, ATTR::ID, toChar(doc, to_string(ID))));
    sequence->append_attribute(attribute(doc, ATTR::WIDTH, toChar(doc, to_string(width))));
    sequence->append_attribute(attribute(doc, ATTR::HEIGHT, toChar(doc, to_string(height))));
    doc.append_node(sequence);
    
    if (folder)
    {
        sequence->append_attribute(attribute(doc, ATTR::TYPE, ATTR::T_FOLDER));
    }
    else
        sequence->append_attribute(attribute(doc, ATTR::TYPE, ATTR::T_FILE));

    
    
    for (size_t i = 0; i < annotations.size(); i++)
    {
        xml_node<> *frame = node(doc, NODE::FRAME);
        frame->append_attribute(attribute(doc, ATTR::ID, to_string(i)));
        frame->append_attribute(attribute(doc, ATTR::TARGETC, to_string(annotations[i].size())));
        
        if (folder)
            frame->append_attribute(attribute(doc, ATTR::FILENAME, filenames[i]));
            
        for (size_t j = 0; j < annotations[i].size(); j++)
        {
            xml_node<> *target = node(doc, NODE::TARGET);
            
            target->append_attribute(attribute(doc, ATTR::ID, to_string(annotations[i][j].ID)));
            target->append_attribute(attribute(doc, ATTR::ACTION, annotations[i][j].actionType));
            
            xml_node<> *loc   = node(doc, NODE::LOCATION);
            
            stringstream ss;
            for (size_t k = 0; k < annotations[i][j].area.size(); k++)
            {
                bool final = (k == annotations[i][j].area.size() - 1);
                              ss << (annotations[i][j].area[k].x ) << "," <<
                                    (annotations[i][j].area[k].y) << ((final)? "" : ",");
            }
            
            loc->value(doc.allocate_string(toChar(doc, ss.str())));
            
            target->append_node(loc);
            frame->append_node(target);
        }
        
        sequence->append_node(frame);
    }
}

void XMLAnnotateProcess::write(const string &filename)
{
    xml_document<> doc;
    // add declaration
    writeHeader(doc);
    writeSequence(doc);
    ofstream file;
    file.open(filename);
    // print out the xml document
    file << doc;
    file.close();
}


/*
 * Convert string to the right encoding
 */
 char *XMLAnnotateProcess::toChar(xml_document<> &doc, const string &name)
{
    return doc.allocate_string(name.c_str());
}
/**
 * creates an attribute with name and value
 */
 xml_attribute<char> *XMLAnnotateProcess::attribute(xml_document<> &doc, const string &name, const string &value)
{
    return doc.allocate_attribute(toChar(doc, name), toChar(doc, value));
}
/*
 *  creates a node with the name
 */
 xml_node<char> *XMLAnnotateProcess::node(xml_document<> &doc, const string &name)
{
    return doc.allocate_node(node_element, toChar(doc, name));
}


/*parses a string with the following format
 *
 * x1,y1,x2,y2,x3,y3,....xn,yn
 */
 void XMLAnnotateProcess::parseLocation(const string &loc, vector<Point2f> &pts)
{
    istringstream l(loc);
    string x,y;
    while (getline(l, x, ',') && getline(l, y,','))
    {
        pts.push_back(Point2f(atof(x.c_str()), atof(y.c_str())));
    }
}

size_t XMLAnnotateProcess::readSequence(xml_node<> &sequence,
                                        vector<vector<Annotation>> &ann,
                                        vector<string> &filenames,
                                        size_t &sequenceID,
                                        int &width,
                                        int &height)
{
   
    size_t sFC = readAttribute<size_t>(sequence, ATTR::FRAMEC);
    string fT  = readAttribute<string>(sequence, ATTR::TYPE);
    sequenceID = readAttribute<size_t>(sequence, ATTR::ID);
    width      = readAttribute<float>(sequence, ATTR::WIDTH);
    height     = readAttribute<float>(sequence, ATTR::HEIGHT);
    
    ann.resize(sFC);
    filenames.resize(sFC);
    
    size_t maxID = 0;
    
    for (xml_node<> *frame = sequence.first_node(NODE::FRAME.c_str());
         frame;
         frame = frame->next_sibling())
    {
        size_t fID = readAttribute<size_t>(*frame, ATTR::ID);
        if (fT == ATTR::T_FOLDER)
            filenames[fID] = (readAttribute<string>(*frame, ATTR::FILENAME));
        
        for(xml_node<> *target = frame->first_node(NODE::TARGET.c_str());
            target;
            target = target->next_sibling())
        {
            size_t tID        = readAttribute<size_t>(*target, ATTR::ID);
            string tAction = readAttribute<string>(*target, ATTR::ACTION);
            xml_node<char> *loc = target->first_node(NODE::LOCATION.c_str());
            
            maxID = (tID > maxID)? tID : maxID;
            
            if (loc)
            {
                string location(loc->value());
                Annotation a;
                a.tracking = false;
                a.ID = tID;
                a.actionType = tAction;
                parseLocation(location, a.area);
                ann[fID].push_back(a);
            }
        }
    }
    return maxID;
}


size_t XMLAnnotateProcess::parse(const string &filename, vector<vector<Annotation>> &ann)
{
    
    ifstream file(filename);
    vector<char> buffer((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
    buffer.push_back('\0');
    ann.clear();
    
    
    xml_document<> doc;
    xml_node<>* sequence;
    doc.parse<0>(&buffer[0]);
    
    sequence = doc.first_node(NODE::SEQ.c_str());
    
    vector<string> filenames;
    size_t camera;
    int sX, sY;
    return readSequence(*sequence, ann, filenames, camera, sX, sY);
}

int XMLAnnotateProcess::read(const string &filename)
{
    drawing.clear();
    return parse(filename, annotations);
}
