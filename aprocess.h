
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

#include "clp.hpp"
#include "viva.h"
#include "skcfdcf.h"
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <map>

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
struct Annotation
{
    int ID;
    bool tracking;
    int mode;
    string action;
    vector<Point2f> area;
    
};
class XMLAnnotateProcess;
class InputFactory
{
public:
    static bool isVideoFile(const string &sequence);
    static bool isWebFile(const string &sequence);
    static bool isStringSequence(const string &sequence);
    static bool isFolderSequence(const string &sequence);
    static bool isCameraID(const string &s);
    static bool filenames(const string &folder, vector<string> &filenames);
    static Ptr<Input> create(const string &sequence, const Size sz = Size(-1, -1));
    static Ptr<Input> create(const string &sequence, vector<string> &filenames, const Size sz = Size(-1, -1));
    static string  findInputGroundTruth(const string &sequence, const string &defaultname);
    
    static int getMode(CommandLineParserExt &parser);
    
    static void load(CommandLineParserExt &parser,
              vector<Ptr<Input>> &inputs,
                     vector<Ptr<XMLAnnotateProcess>> &processes);
    
    static void initialize(CommandLineParserExt &parser,
                    vector<string> &actions,
                    vector<Ptr<Input>> &inputs,
                    vector<Ptr<XMLAnnotateProcess>> &processes);
    
    
    
    
    
    static void write(const string &filename,
               vector<string> &actions,
                      vector<Ptr<XMLAnnotateProcess>> &processes);
    
};
class Draw
{
public:
    static void displayPolygon(Mat &image,
                               const vector<Point2f> &pts,
                               const Scalar &color = Color::red,
                               int thickness = 2,
                               bool close = true);
    
    static void displayAnnotation(Mat &image,
                                  Annotation &ann,
                                  const Scalar &background = Color::yellow,
                                  const Scalar &foreground = Color::red,
                                  int thickness = 2,
                                  bool close = true);
    
    static void displayPolygonInfo(cv::Mat &image,
                                   const string &info,
                                   const vector<Point2f> &pts,
                                   const Scalar &background = Color::yellow,
                                   const Scalar &foreground = Color::red);
};
class Geometry
{
public:
    /*
     * Centroid of a list of points
     */
    static Point2f centroid(const vector<Point2f> &corners);

    /*
     * Get axis align rectangle defined by point and ratio
     */
    static void getARect(const Point2f &mPos, float ratio,
                  Point2f &p1, Point2f &p2, Point2f &p3, Point2f &p4);
    /*
     * Get rotated rectangle defined by vector and ratio
     */
    static void getRRect(const Point2f &mPos, float ratio,
                  Point2f &p1, Point2f &p2, Point2f &p3, Point2f &p4);

    /*
     * Closest point to a Ray
     */
    static float closestPointToRay(const Point2f &pt, const Point2f &s, const Point2f &e);

    /*
     * Point inside polygon
     */
    static bool  ptInsidePolygon(const Point2f &pt, const vector<Point2f> &polygon);

    /*
     * vector perpendicular to segment (s,e)
     */
    static Point2f vectorPerpendicularToSegment(const Point2f &s, const Point2f &e);
};
class AnnotateProcess : public ProcessFrame
{
protected:
    bool  acceptPolygon(const vector<Point2f> &polyg, int m);
    bool  isAnnotationSelected();
    int   findAnnotationIndexContaining(const Point2f &pt);
    void  swapPolygon(int i);


    Ptr<SKCFDCF> initTracker(Mat frame, Rect area);
    void helpHUD(Mat &image);

    Mat currentFrame;
    long currentFrameN;
	int  totalNumberOfFrames;

    /*
     * used for annotation selection 
     * and moving
     */
    Point2f mousePos;
    Point2f mouseShift;
    int selection;

    /*
     * used for drawing
     */
    int thickness;
    float ratio;
    bool showHelp;

    /*
     * options
     */
    bool tracking;
	bool annotatingActions;
    
 
public:
    static int currentAnnotationID;

    enum
    {
        AXIS_RECT,
        ROTA_RECT,
        POLY
    };


	vector<vector<Annotation>> annotations;
    Annotation draw;

    vector<std::string> actions;

    map<int, Ptr<SKCFDCF>> trackers;
    /*
     * @param ratioYX = the ratio between the rectangular selection. computed as height/width.
     *                  a negative value will remove any ratio constraint.
     *                  a positive value > 1 will make the height longer than the width
     *                  a positive value between 0 and 1 will make the width longer than the height
     *                  a value of 1 will make a square selection
     */

    AnnotateProcess(float ratioYX = -1.f,
                    int   method  = AXIS_RECT,
                    bool  track   =  true,
					bool  action  =  false,
					int totalFrameN = 0):
                      currentFrame(),
                      currentFrameN(-1),
                      totalNumberOfFrames(totalFrameN),
                      mousePos(-1,-1),
                      mouseShift(-1,-1),
                      selection(-1),
                      thickness(2),
                      ratio(ratioYX),
                      showHelp(true),
                      tracking(track),
		              annotatingActions(action),
                      annotations(totalFrameN),
                      draw(),
                      actions(),
                      trackers()

    {
        draw.mode = method;
        draw.action = "";
    }

    void setAnnotationAspectRation(float ratioYX);
    void setAnnotationMode(int m);
    void enableTracking(bool flag = true);
    void setActions(vector<string> &actns);
    void setNumberOfFrames(int frameCount);
    void setAnnotations(vector<vector<Annotation>> &ann);

    virtual void operator()(const size_t frameN, const Mat &frame, Mat &output);
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

    virtual void remAnnotation();

    virtual void newAnnotation();

    static bool readActionTypeFile(const string &filename,
                                   vector<string> &actions);
};
struct ATTR
{
    static string VERSION;
    static string FILENAME;
    static string TYPE;
    static string ENCODING;
    static string FRAMEC;
    static string ID;
    static string TARGETC;
    static string ACTION;
    static string TARGET1;
    static string TARGET2;


    static string WIDTH;
    static string HEIGHT;

    static string T_FOLDER;
    static string T_FILE;
};
struct NODE
{
    static string SEQ;
    static string ACTIONS;
    static string ACTION;
    static string FRAME;
    static string TARGET;
    static string LOCATION;
    static string MATCHING;
    static string MATCH;
};
class XMLAnnotateProcess: public AnnotateProcess
{
    /*
     * Convert string to the right encoding
     */
    static char *toChar(xml_document<> &doc, const string &name);
    /**
     * creates an attribute with name and value
     */
    static xml_attribute<char> *attribute(xml_document<> &doc,
                                          const string &name,
                                          const string &value);
    /*
     *  creates a node with the name
     */
    static xml_node<char> *node(xml_document<> &doc, const string &name);


    static xml_attribute<char> *hasAttribute(xml_node<> &node, const string &name)
    {
        return node.first_attribute(name.c_str());
    }
    /*
     * reads an attribute from the node and converts it to type T
     */
    template<class T>
    static T readAttribute(xml_node<> &node, const string &name)
    {
        xml_attribute<char> *attr = hasAttribute(node, name);
        T _tmp = T();
        if (attr)
        {
            istringstream ss(attr->value());
            ss >> _tmp;
        }
        return _tmp;
    }
    /*parses a string with the following format
     *
     * x1,y1,x2,y2,x3,y3,....xn,yn
     */
    static void parseLocation(const string &loc, vector<Point2f> &pts);

    
    
public:
    

    
    string input;
    int width;
    int height;
    int ID;
    
    /*
     * @param ratioYX = the ratio between the rectangular selection. computed as height/width.
     *                  a negative value will remove any ratio constraint.
     *                  a positive value > 1 will make the height longer than the width
     *                  a positive value between 0 and 1 will make the width longer than the height
     *                  a value of 1 will make a square selection
     */
    
    XMLAnnotateProcess(const string &_input,
                       int inputWidth,
                       int inputHeight,
                       int sequenceID,
                       float ratioYX = -1.f,
                       int   method  =  AXIS_RECT,
                       bool  track   =  true,
                       bool  action  =  false,
                       int totalFrameN = 0)
        : AnnotateProcess(ratioYX, method, track, action, totalFrameN),
         input(_input), width(inputWidth), height(inputHeight), ID(sequenceID)
    {}

    static void readActions(xml_node<> &actions, vector<string> &actns);

    static  void writeHeader(xml_document<> &doc);
    static  void writeActions(xml_document<> &doc, const vector<string> &actions);
    void writeSequence(xml_document<> &doc);
    void write(const string &filename);

    
    size_t read(const string &filename);
    size_t read(xml_document<> &doc);

    static void readSequenceMetadata(xml_node<> &sequence,
                                     size_t &sequenceID,
                                     size_t &frameCount,
                                     string &type,
                                     int &width,
                                     int &height);
    static void readSequenceFilenames(xml_node<> &sequence,
                                      vector<string> &filenames);
    static size_t readSequenceAnnotations(xml_node<> &sequence,
                                        vector<vector<Annotation>> &ann);

    static void readXML(const string &filename, xml_document<> &doc)
    {
        ifstream file(filename);
        vector<char> buffer((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
        buffer.push_back('\0');
        doc.parse<0>(&buffer[0]);
        file.close();
    }
    
};
#endif