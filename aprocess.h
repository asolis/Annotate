
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
#include "skcfdcf.h"
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>

using namespace viva;
using namespace rapidxml;
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
    static string  findInputGroundTruth(const string &sequence, const string &defaultname);
};

class Draw
{
public:
    static void displayPolygon(Mat &image,
                               const vector<Point2f> &pts,
                               const Scalar &color = Color::red,
                               int thickness = 2,
                               bool close = true);
	static void displayPolygonNumber(Mat &image,
							  const vector<Point2f> &pts,
							  int number);
    static void displayPolygonNumberNAction(Mat &image,
                              const vector<Point2f> &pts,
                              int number,
							  string actionType);
};

struct Annotation
{
	vector<Point2f> annotateFrame;
	int mode;
	string actionType;
	int ID;
};

class AnnotateProcess : public ProcessFrame
{
protected:
    float closestPointToRay(const Point2f &pt, const Point2f &s, const Point2f &e);
    bool  acceptPolygon(const vector<Point2f> &polyg, int m);
    void  swapPolygon(int i);
    bool  ptInsidePolygon(const Point2f &pt, const vector<Point2f> &polygon);
    int   findIndexOfPolygonContainingPt(const Point2f &pt);
    Point2f vectorPerpendicularToSegment(const Point2f &s, const Point2f &e);

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
	void helpActionHUB(Mat &image);

    Point2f centroid(const vector<Point2f> &corners);


    long currentFrameN;
	int totalFrame;
    Point2f mousePos;
    Point2f mouseShift;
    int thickness;
    int mode;
    int selection;
	int peopleAmount;
	string currentActionType;
    float ratio;
    bool showHelp;
	bool showActionHelp;
    bool continuity;
    bool tracking;
	bool actionAnnotating;


public:

    enum
    {
        AXIS_RECT,
        ROTA_RECT,
        POLY
    };

	vector<vector<Annotation>> annotations;
	vector<std::string> actionType;

    vector<Point2f> drawing;

    vector<Ptr<SKCFDCF>> trackers;
    Mat currentFrame;

    /*
     * @param ratioYX = the ratio between the rectangular selection. computed as height/width.
     *                  a negative value will remove any ratio constraint.
     *                  a positive value > 1 will make the height longer than the width
     *                  a positive value between 0 and 1 will make the width longer than the height
     *                  a value of 1 will make a square selection
     */

    AnnotateProcess(float ratioYX = -1.f,
                    int   method  = POLY,
                    bool  cont    =  true,
                    bool  track   =  true,
					bool  action  =  false,
					int totalFrameN = 0):
                      currentFrameN(-1),
                      totalFrame(totalFrameN),
                      mousePos(-1,-1),
                      mouseShift(-1,-1),
                      thickness(2),
                      mode(method),
                      selection(-1),
                      ratio(ratioYX),
                      showHelp(true),
					  showActionHelp(false),
                      continuity(cont),
                      tracking(track),
                      annotations(),
                      drawing(),
                      trackers(),
					  currentActionType(""),
                      currentFrame(),
		              peopleAmount(0),
		              actionAnnotating(action)
    {}


	virtual void operator()(const size_t frameN, const Mat &frame, Mat &output)
	{
		if (frameN >= annotations.size())
		{
			// current frame doesn't have any annotation information
			// create a new annotation base on the previous annotation information
			
			/** copy annotations from previous frame to current frame**/
			if ((continuity || tracking) && frameN > 0)
			{
				annotations.push_back(annotations[annotations.size() - 1]);
			}
			/** create new set of empty annotations for the current frame**/
			else
			{
				vector<Annotation> tmp;
				annotations.push_back(tmp);

				trackers.clear();
			}

			currentFrameN = frameN;
			frame.copyTo(currentFrame);

			/** if tracking is selected, propose a new position for the previous
			frame annotations **/

			if (tracking)
			{
				for (size_t i = 0; i < trackers.size(); i++)
				{
					trackers[i]->processFrame(frame);
					Point2f deltha;
					float scale;
					trackers[i]->getTransformation(deltha, scale);

					for (size_t p = 0; p < annotations[currentFrameN][i].annotateFrame.size(); p++)
					{
						annotations[currentFrameN][i].annotateFrame[p] += deltha;
					}
				}
			}

		}
		//else
		//{
		//	currentFrameN = frameN;
		//	int previousFrameAnnoSize = annotations.at(currentFrameN).size();
		//	int currentFrameAnnoSize = previousFrameAnnoSize;
		//	try
		//	{
		//		previousFrameAnnoSize = annotations.at(currentFrameN - 1).size();
		//	}
		//	catch (const std::exception&){}				
		//	if (previousFrameAnnoSize > currentFrameAnnoSize && tracking)
		//	{
		//		int gap = previousFrameAnnoSize - currentFrameAnnoSize;
		//		// track the new annotation
		//		for (int i = 0; i < gap; i++)
		//		{
		//			annotations[currentFrameN].push_back(annotations[currentFrameN - 1].at(currentFrameAnnoSize + i - 1));
		//		}

		//		// propose a new position for the previous annotation frame
		//		for (size_t i = 0; i < trackers.size(); i++)
		//		{
		//			trackers[i]->processFrame(frame);
		//			Point2f deltha;
		//			float scale;
		//			trackers[i]->getTransformation(deltha, scale);

		//			for (size_t p = 0; p < annotations[currentFrameN][i + currentFrameAnnoSize].annotateFrame.size(); p++)
		//			{
		//				annotations[currentFrameN][i + currentFrameAnnoSize].annotateFrame[p] += deltha;
		//			}
		//		}

		//	}
		//}

		currentFrameN = frameN;
		frame.copyTo(output);

		if (showHelp)
			helpHUD(output);

		if (showActionHelp)
			helpActionHUB(output);


		// display the existed annotation
		for (int i = 0; i < annotations[currentFrameN].size(); i++)
		{
			Draw::displayPolygon(output, annotations[currentFrameN][i].annotateFrame, Color::yellow, thickness, true);
			Draw::displayPolygonNumberNAction(output, annotations[currentFrameN][i].annotateFrame,
				annotations[currentFrameN][i].ID, annotations[currentFrameN][i].actionType);
		}

		// display the currennt drawing
		Draw::displayPolygon(output, drawing, Color::red, thickness, mode != POLY);


		if (mode == POLY)
		{
			if (drawing.size() > 0)
			{
				line(output, drawing.back(), mousePos,
					Color::blue, thickness - 1, CV_AA);
				line(output, mousePos, drawing.front(),
					Color::red, thickness - 1, CV_AA);
			}
		}

		if (mode == ROTA_RECT)
		{
			int pts = drawing.size();

			if (pts == 1)
			{
				line(output, drawing.front(), mousePos,
					Color::blue, thickness - 1, CV_AA);
			}
			else if (pts == 2)
			{


				vector<Point2f> _rRect(4);
				_rRect[0] = drawing.back();
				_rRect[1] = drawing.front();
				getRRect(mousePos, ratio, _rRect[0], _rRect[1], _rRect[2], _rRect[3]);

				for (size_t i = 0; i < _rRect.size(); i++)
				{
					line(output, _rRect[i], _rRect[(i + 1) % 4],
						(i == 0) ? Color::red : Color::blue,
						(i == 0) ? thickness : thickness - 1, CV_AA);
				}

				circle(output, _rRect[2], thickness, Color::green);

				if (ratio > 0)
					circle(output, _rRect[0], thickness, Color::yellow);

			}
		}

		if (mode == AXIS_RECT)
		{
			if (drawing.size() == 1)
			{
				vector<Point2f> _aRect(4);
				_aRect[0] = drawing.front();
				getARect(mousePos, ratio, _aRect[0], _aRect[1], _aRect[2], _aRect[3]);
				rectangle(output, _aRect[0],
					_aRect[2],
					Color::blue,
					thickness - 1,
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
    virtual void leftButtonDown(int x, int y, int flags);
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
    virtual void mouseMove(int x, int y, int flags);
    /**
     * Inherited from KeyboardListerner. Check KeyboardListener class for details.
     */
    virtual void keyboardInput(int key);

    virtual void newTracker();

    virtual void remTracker(int i);

    virtual void remAnnotation();

    virtual void newAnnotation();

    virtual bool readActionTypeFile(const string &filename);

    virtual void write(const string &filename, const float scaleX = 1.0f, const float scaleY = 1.0f) = 0;

    virtual int read(const string &filename) = 0;
};

class CSVAnnotateProcess : public AnnotateProcess
{
public:


    /*
     * @param ratioYX = the ratio between the rectangular selection. computed as height/width.
     *                  a negative value will remove any ratio constraint.
     *                  a positive value > 1 will make the height longer than the width
     *                  a positive value between 0 and 1 will make the width longer than the height
     *                  a value of 1 will make a square selection
     */

    CSVAnnotateProcess(float ratioYX = -1.f,
                    int   method  = POLY,
                    bool  cont    =  true,
                    bool  track   =  true,
                    bool  action  =  false,
                    int totalFrameN = 0):AnnotateProcess(ratioYX, method, cont, track, action, totalFrameN)
    {}

    static int parse(const string &filename, vector<vector<Annotation>> &annotation);
    virtual int read(const string &filename);
    virtual void write(const string &filename, const float scaleX = 1.0f, const float scaleY = 1.0f);

};

class XMLAnnotateProcess: public AnnotateProcess
{
public:
    /*
     * @param ratioYX = the ratio between the rectangular selection. computed as height/width.
     *                  a negative value will remove any ratio constraint.
     *                  a positive value > 1 will make the height longer than the width
     *                  a positive value between 0 and 1 will make the width longer than the height
     *                  a value of 1 will make a square selection
     */

    XMLAnnotateProcess(float ratioYX = -1.f,
                    int   method  = POLY,
                    bool  cont    =  true,
                    bool  track   =  true,
                    bool  action  =  false,
                       int totalFrameN = 0): AnnotateProcess(ratioYX, method, cont, track, action, totalFrameN)
    {}

    virtual void write(const string &filename, const float scaleX = 1.0f, const float scaleY = 1.0f);
    virtual void allocateAttrToNodeXML(xml_document<> &doc, xml_node<> *node, const char * name, string value);
    static int parse(const string &filename, vector<vector<Annotation>> &annotation);
    virtual int read(const string &filename);
};
#endif