
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
    static bool isVideoFile(const string &sequence);
    static bool isWebFile(const string &sequence);
    static bool isStringSequence(const string &sequence);
    static bool isFolderSequence(const string &sequence);
    static bool isCameraID(const string &s);
    static Ptr<Input> create(const string &sequence, const Size sz = Size(-1, -1));
};


class AnnotateProcess : public ProcessFrame
{
private:
    float closestPointToRay(const Point2f &pt, const Point2f &s, const Point2f &e);
    bool  acceptPolygon(const vector<Point2f> &polyg, int m);
    void  swapPolygon(int i);
    bool  ptInsidePolygon(const Point2f &pt, const vector<Point2f> &polygon);
    int   findIndexOfPolygonContainingPt(const Point2f &pt);
    Point2f vectorPerpendicularToSegment(const Point2f &s, const Point2f &e);
    void displayPolygon(Mat &image,
                        const vector<Point2f> &pts,
                        const Scalar &color = Color::red,
                        bool close = true);
    void getRRect(const Point2f &mPos, float ratio,
                          Point2f &p1,
                          Point2f &p2,
                          Point2f &p3,
                  Point2f &p4);
    void getARect(const Point2f &mPos, float ratio,
                          Point2f &p1,
                          Point2f &p2,
                          Point2f &p3,
                  Point2f &p4);
    void helpHUD(Mat &image);

    Point2f centroid(const vector<Point2f> &corners);


    long currentFrameN;
    Point2f mousePos;
    Point2f mouseShift;
    int thickness;
    int mode;
    float ratio;
    bool showHelp;
    bool continuity;

public:

    enum
    {
        AXIS_RECT,
        ROTA_RECT,
        POLY
    };

    vector<vector<vector<Point2f>>> annotations;
    vector<vector<int>> modes;
    vector<Point2f> drawing;

    /*
     * @param ratioYX = the ratio between the rectangular selection. computed as height/width.
     *                  a negative value will remove any ratio constraint.
     *                  a positive value > 1 will make the height longer than the width
     *                  a positive value between 0 and 1 will make the width longer than the height
     *                  a value of 1 will make a square selection
     */

    AnnotateProcess(float ratioYX = -1.f,
                    int   method  = POLY,
                    bool  cont    =  true):
                      currentFrameN(-1),
                      mousePos(-1,-1),
                      mouseShift(-1,-1),
                      thickness(2),
                      mode(method),
                      ratio(ratioYX),
                      showHelp(true),
                      continuity(cont),
                      annotations(),
                      modes(),
                      drawing()
    {}


    virtual void operator()(const size_t frameN, const Mat &frame, Mat &output)
    {
        if (currentFrameN != frameN)
        {
            if (continuity && frameN > 0)
            {
                annotations.push_back(annotations[annotations.size() - 1]);
                modes.push_back(modes[modes.size() - 1]);
            }
            else
            {
                vector<vector<Point2f>> tmp;
                annotations.push_back(tmp);

                vector<int> tmp2;
                modes.push_back(tmp2);
            }
            currentFrameN = frameN;
        }


        frame.copyTo(output);

        if (showHelp)
            helpHUD(output);

        for (int i = 0; i < annotations[currentFrameN].size(); i++)
            displayPolygon(output, annotations[currentFrameN][i], Color::yellow, true);

        displayPolygon(output, drawing, Color::red, mode != POLY);


        if (mode == POLY)
        {
            if (drawing.size() > 0)
            {
                line(output, drawing.back(), mousePos,
                     Color::blue, thickness, CV_AA);
                line(output, mousePos, drawing.front(),
                     Color::red, thickness - 1 , CV_AA);
            }
        }
        
        if (mode == ROTA_RECT)
        {
            int pts = drawing.size();

            if (pts == 1)
            {
                line(output, drawing.front(), mousePos,
                     Color::blue, thickness, CV_AA);
            }
            else if (pts == 2)
            {


                vector<Point2f> _rRect(4);
                _rRect[0] = drawing.back();
                _rRect[1] = drawing.front();
                getRRect(mousePos, ratio, _rRect[0], _rRect[1], _rRect[2], _rRect[3]);

                for (size_t i = 0; i < _rRect.size(); i++)
                {
                    line(output, _rRect[i], _rRect[(i + 1)% 4],
                         (i==0)? Color::red : Color::blue,
                         (i==0)? thickness  : thickness - 1, CV_AA);
                }

                circle(output, _rRect[2], thickness, Color::green);

                if (ratio > 0)
                    circle(output,  _rRect[0], thickness, Color::yellow);

            }
        }

        if (mode == AXIS_RECT)
        {
            if (drawing.size() == 1)
            {
                vector<Point2f> _aRect(4);
                _aRect[0] = drawing.front();
                getARect(mousePos, ratio, _aRect[0], _aRect[1], _aRect[2], _aRect[3]);
                rectangle(output,_aRect[0],
                          _aRect[2],
                          Color::blue,
                          thickness,
                          CV_AA);
                circle(output, _aRect[2], thickness - 1, Color::green);
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
        if  (flags &  EVENT_FLAG_CTRLKEY  || flags & EVENT_FLAG_SHIFTKEY)
        {
            mouseShift = Point2f(x,y);
            int found = findIndexOfPolygonContainingPt(mouseShift);
            if (found >= 0)
                swapPolygon(found);

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

    /**
     * Inherited from KeyboardListerner. Check KeyboardListener class for details.
     */
    virtual void keyboardInput(int key)
    {
        if (key == 'b' || key == 'B')
        {
            if (drawing.empty())
            {
                if (!annotations[currentFrameN].empty())
                    annotations[currentFrameN].pop_back();
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
                drawing.pop_back();
        }
        if (key == 'c' || key == 'C')
        {
            drawing.clear();
            while (!annotations[currentFrameN].empty())
                annotations[currentFrameN].pop_back();
        }
        if (key == 'h' || key == 'H')
            showHelp = !showHelp;

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
    };

    virtual void newAnnotation()
    {
        if (acceptPolygon(drawing, mode))
        {
            annotations[currentFrameN].push_back(drawing);
            modes[currentFrameN].push_back(mode);
            drawing.clear();
        }
    }

    virtual void writeAnnotations(const string &filename, const float scaleX = 1.0f, const float scaleY = 1.0f)
    {
        ofstream file;
        file.open(filename);
        for (size_t i = 0; i < annotations.size(); i++)
        {

            for (size_t j = 0; j < annotations[i].size(); j++)
            {
                for (size_t k = 0; k < annotations[i][j].size(); k++)
                {

                    file << annotations[i][j][k].x * scaleX << ", "
                         << annotations[i][j][k].y * scaleY;
                    
                    if ( k != (annotations[i][j].size() - 1))
                        file << ", ";
                }
                if ( j != (annotations[i].size() - 1))
                    file << "|";
            }
            file << endl;
        }
    }

};


#endif