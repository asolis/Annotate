
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

#ifndef __annotate__process__
#define __annotate__process__

#include "viva.h"
#include <iostream>
#include <sstream>
#include <fstream>

using namespace viva;

/**
 *  Color struct.
 *  Custom color codes used by the Draw class
 *  The color are specified in BGR format (i.e., blue, gree, and red)
 */
struct Color
{
    const static Scalar red;
    const static Scalar blue;
    const static Scalar green;
    const static Scalar white;
    const static Scalar purple;
    const static Scalar yellow;
    const static Scalar teal;
    const static Scalar orange;
};


class InputFactory
{
public:
    static bool isVideoFile(const string &sequence)
    {
        return (viva::Files::isFile(sequence));
    }
    static bool isWebFile(const string &sequence)
    {
        return sequence.substr(0,4) == "http";
    }
    static bool isStringSequence(const string &sequence)
    {
        char res0[sequence.length() * 2];
        char res1[sequence.length() * 2];
        sprintf(res0, sequence.c_str(), 0);
        sprintf(res1, sequence.c_str(), 1);
        string path0(res0);
        string path1(res1);
        return (viva::Files::isFile(path0) || viva::Files::isFile(path1));
    }
    static bool isFolderSequence(const string &sequence)
    {
        return (viva::Files::isDir(sequence));
    }


    static Ptr<Input> create(const string &sequence, const Size sz = Size(-1, -1))
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
        return Ptr<Input>();
    }
};


class AnnotateProcess : public ProcessFrame
{
private:

    long currentFrameN;
    Point2f mousePos;
    int thickness;
    int mode;
    float ratio;
    bool showHelp;

    float closestPointToRay(const Point2f &pt, const Point2f &s, const Point2f &e)
    {
        float A = pt.x - s.x;
        float B = pt.y - s.y;
        float C = e.x - s.x;
        float D = e.y - s.y;

        float dot = A*C + B*D;
        float len_sq = C*C + D*D;
        return dot/len_sq;
    }

    Point2f vectorPerpendicularToSegment(const Point2f &s, const Point2f &e)
    {
        Point2f tmp = e - s;
        return Point2f( -tmp.y, tmp.x);
    }

    void displayListPoints(Mat &image, const vector<Point2f> &pts, bool close = true)
    {
        int i = 0;
        for ( i = 0; i < ((int)pts.size() - 1); i++)
        {
            line(image, pts[i], pts[i + 1],
                 Color::red, thickness, CV_AA );
        }
        if (close && pts.size() > 0 )//&& mode != POLY)
            line(image, pts[i], pts[0],
                 Color::red, thickness, CV_AA );
    }

    void displayHelp(Mat &image)
    {
        std::stringstream ss;
        ss <<  " (m) : ";

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
        help.push_back(" (b) : Remove last point");
        help.push_back(" (c) : Clear all points");
        help.push_back(" (SPACE) : Pause/play video");
        help.push_back(" (ESC) : Exit Annotate");


        Rect region(10,10, help.size() * 20 + 10, help.size() * 20 + 10);
        region &= Rect(0,0, image.cols, image.rows);
        GaussianBlur(image(region),
                     image(region),
                     Size(0,0), 5);


        for (size_t i = 0, h = 20; i < help.size(); i++, h+= 20)
        {
            putText(image, help[i], Point(20,h),
                    FONT_HERSHEY_SIMPLEX, .5, Color::yellow, 1, CV_AA);
        }

    }

public:
    enum {
        AXIS_RECT,
        ROTA_RECT,
        POLY
    };

    vector<vector<vector<Point2f>>> annotations;
    vector<Point2f> drawing;

    /*
     * @param ratioYX = the ratio between the rectangular selection. computed as height/width.
     *                  a negative value will remove any ratio constraint.
     *                  a positive value > 1 will make the height longer than the width
     *                  a positive value between 0 and 1 will make the width longer than the height
     *                  a value of 1 will make a square selection
     */

    AnnotateProcess(float ratioYX = -1.f,
                    int   method  = POLY):
                      currentFrameN(-1),
                      mousePos(-1,-1),
                      thickness(2),
                      mode(method),
                      ratio(ratioYX),
                      showHelp(true),
                      annotations(),
                      drawing()
    {}



    virtual void operator()(const size_t frameN, const Mat &frame, Mat &output)
    {
        if (currentFrameN != frameN)
        {
            vector<vector<Point2f>> tmp;
            tmp.push_back(vector<Point2f>());
            annotations.push_back(tmp);
            currentFrameN = frameN;
        }


        frame.copyTo(output);

        if (showHelp)
            displayHelp(output);

        for (int i = 0; i < annotations[currentFrameN].size(); i++)
        {
            if ( i == annotations[currentFrameN].size() - 1)
                displayListPoints(output, annotations[currentFrameN][i], false);
            else
                displayListPoints(output, annotations[currentFrameN][i], true);

        }


        int lastElement  = annotations[currentFrameN].size() - 1;


        if (mode == POLY)
        {
            if (annotations[currentFrameN][lastElement].size() > 0)
            {
                line(output, annotations[currentFrameN][lastElement].back(), mousePos,
                     Color::blue, thickness, CV_AA);
                line(output, mousePos, annotations[currentFrameN][lastElement].front(),
                     Color::red, thickness - 1 , CV_AA);
            }
        }
        
        if (mode == ROTA_RECT)
        {
            int pts = annotations[currentFrameN][lastElement].size();

            if (pts == 1)
            {
                line(output, annotations[currentFrameN][lastElement].front(), mousePos,
                     Color::blue, thickness, CV_AA);
            }
            else if (pts == 2)
            {


                Point2f vec = vectorPerpendicularToSegment( annotations[currentFrameN][lastElement].front(),
                                                           annotations[currentFrameN][lastElement].back());

                float t = closestPointToRay(mousePos,
                                            annotations[currentFrameN][lastElement].back(),
                                            annotations[currentFrameN][lastElement].back()+ vec);

                Point2f p1, p2, p3, p4;
                p1 = annotations[currentFrameN][lastElement].front();
                p2 = annotations[currentFrameN][lastElement].back();
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

                line(output, p1, p2,
                     Color::red, thickness, CV_AA);

                line(output, p1, p4,
                     Color::blue, thickness - 1, CV_AA);

                line(output, p2, p3,
                     Color::blue, thickness - 1, CV_AA);

                line(output, p3, p4,
                     Color::blue, thickness - 1, CV_AA);

                circle(output, p3, thickness, Color::green);

                if (ratio > 0)
                    circle(output,  p1, thickness, Color::yellow);

            }
        }

        if (mode == AXIS_RECT)
        {
            if (annotations[currentFrameN][lastElement].size() == 1)
            {
                Point2f p = mousePos;

                if (ratio > 0)
                {
                    Point2f vec(1.0f, ratio);
                    float t = closestPointToRay(mousePos,
                                                annotations[currentFrameN][lastElement].front(),
                                                annotations[currentFrameN][lastElement].front() + vec);
                    p =  annotations[currentFrameN][lastElement].front() + t * vec;
                }

                rectangle(output, annotations[currentFrameN][lastElement].front(),
                          p,
                          Color::blue,
                          thickness,
                          CV_AA);
                circle(output, p, thickness - 1, Color::green);
            }
        }
    };

    /**
     *Inherited from MouseListener. Check MouseListener class for details
     */
    virtual void mouseInput(int event, int x, int y, int flags){};
    /**
     *Inherited from MouseListener. Check MouseListener class for details
     */
    virtual void leftButtonDown(int x, int y, int flags)
    {
        int lastElement  = annotations[currentFrameN].size() - 1;

        if (mode == AXIS_RECT && annotations[currentFrameN][lastElement].size() == 1)
        {
            Point2f p = mousePos;
            if (ratio > 0)
            {
                Point2f vec(1.0f, ratio);
                float t = closestPointToRay(mousePos,
                                            annotations[currentFrameN][lastElement].front(),
                                            annotations[currentFrameN][lastElement].front() + vec);
                p = annotations[currentFrameN][lastElement].front() + t * vec;
            }
            vector<Point2f> tmp = {annotations[currentFrameN][lastElement].front(),p};
            Rect area = boundingRect(tmp);
            area.width -= 1;
            area.height -= 1;

            Point2f tr = area.tl() + Point(area.width, 0);
            Point2f br = area.br();
            Point2f bl = area.tl() + Point(0, area.height);
            annotations[currentFrameN][lastElement].pop_back();
            annotations[currentFrameN][lastElement].push_back(area.tl());
            annotations[currentFrameN][lastElement].push_back(tr);
            annotations[currentFrameN][lastElement].push_back(br);
            annotations[currentFrameN][lastElement].push_back(bl);

            annotations[currentFrameN].push_back(vector<Point2f>());

        }
        else if (mode == ROTA_RECT && annotations[currentFrameN][lastElement].size() == 2)
        {

            Point2f p1, p2, p3, p4;
            p1 = annotations[currentFrameN][lastElement].front();
            p2 = annotations[currentFrameN][lastElement].back();


            Point2f vec = vectorPerpendicularToSegment( p1, p2);
            float t = closestPointToRay(mousePos, p2, p2 + vec);
            p3 = p2 + t * vec;

            if (ratio > 0)
            {
                Point2f vec_ = p1 - p2;
                float mag      = cv::norm(vec_);
                double magP2P3 = cv::norm(p3 - p2);
                vec_ = (vec_/mag) * (1/ratio) * magP2P3;
                p1 = p2 + vec_;
                annotations[currentFrameN][lastElement].front() = p1;
            }
            p4 = p1 + t * vec;


            annotations[currentFrameN][lastElement].push_back(p3);
            annotations[currentFrameN][lastElement].push_back(p4);

            annotations[currentFrameN].push_back(vector<Point2f>());

        }
        else if (mode == ROTA_RECT && annotations[currentFrameN][lastElement].size() == 1)
        {
            annotations[currentFrameN][lastElement].push_back(Point2f(x,y));
            swap(annotations[currentFrameN][lastElement].front(), annotations[currentFrameN][lastElement].back());
        }
        else
        {
            annotations[currentFrameN][lastElement].push_back(Point2f(x,y));
        }
    };
    /**
     *Inherited from MouseListener. Check MouseListener class for details
     */
    virtual void rightButtonDown(int x, int y, int flags){};
    /**
     *Inherited from MouseListener. Check MouseListener class for details
     */
    virtual void middleButtonDown(int x, int y, int flags){};
    /**
     *Inherited from MouseListener. Check MouseListener class for details
     */
    virtual void mouseMove(int x, int y, int flags)
    {
        mousePos = Point2f(x,y);
    };

    /**
     * Inherited from KeyboardListerner. Check KeyboardListener class for details.
     */
    virtual void keyboardInput(int key)
    {
        int lastElement  = annotations[currentFrameN].size() - 1;

        if (key == 'b' || key == 'B')
        {

            if (annotations[currentFrameN][lastElement].empty() && lastElement > 0)
            {
                annotations[currentFrameN]b.pop_back();
            }
            if (mode == AXIS_RECT && annotations[currentFrameN][lastElement].size() == 4)
            {
                annotations[currentFrameN][lastElement].pop_back();
                annotations[currentFrameN][lastElement].pop_back();
            }
            if (mode == ROTA_RECT && annotations[currentFrameN][lastElement].size() == 4)
            {
                annotations[currentFrameN][lastElement].pop_back();
            }
            else if (mode == ROTA_RECT && annotations[currentFrameN][lastElement].size() == 2)
            {
                swap(annotations[currentFrameN][lastElement][0], annotations[currentFrameN][lastElement][1]);
            }

            annotations[currentFrameN][lastElement].pop_back();
        }
        if (key == 'c' || key == 'C')
            annotations[currentFrameN][lastElement].clear();

        if (key == 'h' || key == 'H')
            showHelp = !showHelp;

        if (key == 'm' || key == 'M')
        {
            annotations[currentFrameN][lastElement].clear();
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
    };


    virtual void writeAnnotations(const string &filename, const float scaleX = 1.0f, const float scaleY = 1.0f)
    {
        ofstream file;
        file.open(filename);
        for (size_t i = 0; i < annotations.size(); i++)
        {
//            if (mode == AXIS_RECT)
//            {
//                if (annotations[i].size() > 0)
//                {
//                    Rect area = boundingRect(annotations[i]);
//                    area.width -= 1;
//                    area.height -= 1;
//                    Point2f tl = area.tl();
//                    Point2f tr = area.tl() + Point(area.width, 0);
//                    Point2f br = area.br();
//                    Point2f bl = area.tl() + Point(0, area.height);
//
//                    file << tl.x * scaleX  << ", " << tl.y * scaleY  << ", "
//                         << tr.x * scaleX  << ", " << tr.y * scaleY  << ", "
//                         << br.x * scaleX  << ", " << br.y * scaleY  << ", "
//                         << bl.x * scaleX  << ", " << bl.y * scaleY ;
//                }
//            }
//            else if (mode == ROTA_RECT)
//            {
//                if (annotations[i].size() > 0)
//                {
//                    Point2f tl = annotations[i][0];
//                    Point2f tr = annotations[i][1];
//                    Point2f br = annotations[i][2];
//                    Point2f bl = annotations[i][3];
//
//                    file << tl.x * scaleX  << ", " << tl.y * scaleY  << ", "
//                         << tr.x * scaleX  << ", " << tr.y * scaleY  << ", "
//                         << br.x * scaleX  << ", " << br.y * scaleY  << ", "
//                         << bl.x * scaleX  << ", " << bl.y * scaleY ;
//                }
//
//            }
//            else if (mode == POLY)
//            {
            for (size_t j = 0; j < annotations[i].size(); j++)
            {
                for (size_t k = 0; k < annotations[i][j].size(); k++)
                {

                    file << annotations[i][j][k].x * scaleX << ", "
                         << annotations[i][j][k].y * scaleY;
                    
                    if ( k != (annotations[i][j].size() - 1))
                        file << ", ";
                }
                file << "|";
            }
//            }n
            file << endl;
        }
    }

};


#endif