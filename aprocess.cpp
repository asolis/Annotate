
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
    swap(drawing, annotations[currentFrameN][i]);
    swap(mode, modes[currentFrameN][i]);
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
        if (ptInsidePolygon(pt, annotations[currentFrameN][i]))
            return i;
    }
    return found;
}
Point2f AnnotateProcess::vectorPerpendicularToSegment(const Point2f &s, const Point2f &e)
{
    Point2f tmp = e - s;
    return Point2f( -tmp.y, tmp.x);
}

void Draw::displayPolygonNumber(Mat &image, const vector<Point2f> &pts, int number, string actionType)
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
    int characterWidth  = 20;

    if (mode == AXIS_RECT)
        ss << "Axis Align Rectangle ";
    else if (mode == ROTA_RECT)
        ss << "Rotated Rectangle ";
    else
        ss << "Polygon ";

    if (ratio > 0 && mode != POLY)
        ss << "(ratio=" << ratio << ")";

    vector<string> help;
    help.push_back(ss.str());
    help.push_back(" (-) : Reduce ratio 0.1");
    help.push_back(" (+) : Increase ratio 0.1");
    help.push_back(" (h) : Toggle this help ");
    help.push_back(" (n) : Next frame");
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
	int characterWidth = 10;

	vector<string> help;
	help.push_back(" (0) : Nothing");
	help.push_back(" (1) : Ordering");
	help.push_back(" (2) : Picking");
	help.push_back(" (3) : Waiting");
	help.push_back(" (4) : Leaving");

	int coordinateX = image.cols - (characterWidth * fsize + margin);
	int coordinateY = image.rows - (help.size() * fsize + margin);

	Rect region(coordinateX, coordinateY,  
		characterWidth * fsize + margin,
		help.size() * fsize + margin);
	region &= Rect(0, 0, image.cols, image.rows);

	GaussianBlur(image(region),
		image(region),
		Size(0, 0), 5);

	for (size_t i = 0, h = fsize; i < help.size(); i++, h += fsize)
	{
		if (currentActionType == i)
		{
			putText(image, help[i], Point(fsize + coordinateX, h + coordinateY),
				FONT_HERSHEY_SIMPLEX, .5, Color::red, 1, CV_AA);
		}
		else
		{
			putText(image, help[i], Point(fsize + coordinateX, h + coordinateY),
				FONT_HERSHEY_SIMPLEX, .5, Color::yellow, 1, CV_AA);
		}
		
	}
}

Point2f AnnotateProcess::centroid(const vector<Point2f> &corners)
{
    Moments mu = moments(corners);
    return Point2f(mu.m10/mu.m00, mu.m01/mu.m00);
}
