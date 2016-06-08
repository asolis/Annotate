
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
private:
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
                      currentFrame(),
					  currentActionType(""),
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
    virtual void leftButtonDown(int x, int y, int flags)
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

    virtual void newTracker()
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

    virtual void remTracker(int i)
    {
        if (tracking && i >= 0 && i < trackers.size())
        {
            trackers.erase(trackers.begin() + i);
        }
    }

    virtual void remAnnotation()
    {
        if (selection >= 0 && selection < annotations[currentFrameN].size())
        {
            annotations[currentFrameN].erase(annotations[currentFrameN].begin() + selection);
            trackers.erase(trackers.begin() + selection);
        }
        drawing.clear();
        selection = -1;
    }

    virtual void newAnnotation()
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

    virtual void writeAnnotations(const string &filename, const float scaleX = 1.0f, const float scaleY = 1.0f)
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

	virtual void writeXMLAnnotations(const string &filename, const float scaleX = 1.0f, const float scaleY = 1.0f)
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

	virtual void allocateAttrToNodeXML(xml_document<> &doc, xml_node<> *node, const char * name, string value)
	{
		// convert string to char
		char *CharTmp = doc.allocate_string(value.c_str());
		xml_attribute<> *attr= doc.allocate_attribute(name, CharTmp);
		node->append_attribute(attr);
	}

	virtual int readXMLAnnotationFile(const string &filename) 
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

		annotations.clear();
		drawing.clear();
		trackers.clear();

		// parse the root node
		xml_node<>* cur_node = doc.first_node("video");
		peopleAmount = atoi(cur_node->first_attribute()->value());

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

			annotations.push_back(instance);
		}

		return currentFrameN;
	}

	virtual bool readActionTypeFile(const string &filename)
	{
		string line;
		ifstream myfile(filename);
		if (myfile.is_open())
		{
			while (getline(myfile, line))
			{
				actionType.push_back(line);
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

	static void parseAnnotations(const string &filename, vector<vector<vector<Point2f>>> &_data)
    {
        ifstream file;
        file.open(filename);
        string line;
        long lcount = 0;

        _data.clear();
        while (file)
        {
            if (!getline(file, line)) break;

            istringstream streamline(line);
            string annotation;

            vector<vector<Point2f>> _line;
            _data.push_back(_line);

            while (getline(streamline, annotation, '|'))
            {
                istringstream streamannot(annotation);
                vector<Point2f> pts;
                string x, y;
                while (getline(streamannot, x, ',') &&
                       getline(streamannot, y, ',') )
                {
                    pts.push_back(Point2f(atof(x.c_str()),
                                          atof(y.c_str())));
                }
                _data[lcount].push_back(pts);
                streamannot.clear();
            }
            lcount++;
            streamline.clear();
            
        }
        file.close();
    }

	static void parseXMLAnnotations(const string &filename, vector<vector<vector<Point2f>>> &_data)
	{		
		string input_xml;
		std::string line;
		std::ifstream in(filename);
		// read the file into input_XML
		while (getline(in, line))
			input_xml += line;

		vector<char> xml_copy(input_xml.begin(), input_xml.end());
		xml_copy.push_back('\0');

		xml_document<> doc;
		doc.parse<parse_no_data_nodes>(&xml_copy[0]);

		long lcount = 0;
		_data.clear();
		
		// parse the root node
		xml_node<>* cur_node = doc.first_node("video");

		// frame loop
		for (xml_node<> *frame_node = cur_node->first_node(); frame_node; frame_node = frame_node->next_sibling())
		{
			vector<vector<Point2f>> _line;
			_data.push_back(_line);
			// box loop
			for (xml_node<> *box_node = frame_node->first_node(); box_node; box_node = box_node->next_sibling())
			{
				vector<Point2f> pts;
				for (xml_attribute<> *point = box_node->first_attribute(); point; point = point->next_attribute())
				{
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
				_data[lcount].push_back(pts);
			}
			lcount++;
		}
	}
};


#endif