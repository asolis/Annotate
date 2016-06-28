
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


struct AnnotationPreview
{
    int ID;
    int fromFrameN;
    Mat preview;
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
    
    static int getMode(CommandLineParserExt &parser);
    
    static void load(CommandLineParserExt &parser,
                     vector<string> &actions,
                     vector<pair<Point,Point>> &matches,
                     vector<Ptr<Input>> &inputs,
                     vector<Ptr<XMLAnnotateProcess>> &processes);
    
    static void initialize(CommandLineParserExt &parser,
                    vector<string> &actions,
                    vector<Ptr<Input>> &inputs,
                    vector<Ptr<XMLAnnotateProcess>> &processes);
    
    static void write(const string &filename,
                      vector<string> &actions,
                      vector<pair<Point,Point>> &matches,
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
    
    static void drawText(Mat &img,
                         const string &text,
                         const Scalar &color,
                         const Point &textOrg,
                         double fontScale,
                         int thickness);
    
    static void drawRectangle(Mat &img, const Rect &rect, const Scalar &color, int mode = CV_FILLED)
    {
        rectangle(img, rect.tl(), rect.br(), color, mode);
    }
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
    void  clearSelection();

    Ptr<SKCFDCF> initTracker(Mat frame, Rect area);
    void helpHUD(Mat &image);
    void removeIDFromFrameNumber(int ID, size_t frameNumber);
    void forceTracking();

public:
    Mat currentFrame;
    long currentFrameN;
	int  totalNumberOfFrames;
    
protected:
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
    map<int, AnnotationPreview> previews;
    
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
                      trackers(),
                      previews()

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
    
    virtual void createPreview(const Annotation &ann, const Mat &image, size_t frameNumber);

    static bool readActionTypeFile(const string &filename,
                                   vector<string> &actions);
    
    bool annotationExists(int ID, int frameN);
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
    static string FROM_SEQ;
    static string TO_SEQ;
    static string FROM_ID;
    static string TO_ID;

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
    
    
    static  void readActions(xml_node<> &actions, vector<string> &actns);
    static  void readMatching(xml_node<> &matching, vector<pair<Point,Point>> &matchs);
    static  void writeHeader(xml_document<> &doc);
    static  void writeActions(xml_document<> &doc, const vector<string> &actions);
    static  void writeMatching(xml_document<> &doc, const vector<pair<Point,Point>> &matchs);
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
    static void readXML(const string &filename, xml_document<> &doc);
    
};



class MatchingProcess: public ProcessFrame
{
    enum {
        EMPTY = 0,
        FIRST = 1,
        SECOND = 2
    };
    
    
    /**
     * board drawing sizes
     *
     */
public:
    static int _cols;
    static int _rows;
    static int _padd;
    static int _header;
    static int maxAnnotations;
    
private:
    Point _hover; // board index of hover location
    pair<Point, Point> _match; //board indeces of current matching
    
    int _state;
    vector<vector<int>> displayedIds;
    vector<int> displayedMatches;
    
public:
    
    vector<Ptr<XMLAnnotateProcess>> process;
    vector<pair<Point,Point>> matches;
    
public:
    MatchingProcess(const vector<Ptr<XMLAnnotateProcess>> &p,
                    const vector<pair<Point,Point>> &mchs)
        : _hover(-1,-1),
          _match(make_pair<Point, Point>(Point(-1,-1), Point(-1,-1))),
          _state(EMPTY),
         displayedIds(p.size()),
         process(p),
         matches(mchs)
    {}
    
    virtual ~MatchingProcess(){}
    
    virtual Point boardIndexToPixelCoord(const Point &index)
    {
        return Point(index.x * (_cols + _padd),
                     index.y * (_rows + _padd + _header));
    }
    
    virtual int drawPreview(size_t row, size_t col,
                                      const AnnotationPreview &preview,
                                      Mat &img, int vShift = 0, bool showHover = true)
    {
        int startCols = col * (_cols + _padd);
        int endCols   = startCols + _cols;
        
        int startRow  = vShift + row * (_rows + _padd + _header);
        
        int startText = startRow  + 2;
        int startImag = startRow  + _header;
        int endImag   = startImag + _rows;
        
        preview.preview.copyTo(img(Range(startImag, endImag),
                                   Range(startCols, endCols)));
        
        Draw::drawRectangle(img, Rect(startCols, startRow,
                                      _cols, _header),
                            Color::yellow);
        
        Draw::drawText(img, "ID: "+to_string(preview.ID),
                       Color::red, Point(startCols, startText), .3, 1);
        
        if (col == _hover.x && row == _hover.y && showHover)
        {
            Draw::drawRectangle(img, Rect(startCols, startRow,
                                          _cols, _rows + _header),
                                Color::red, 0);
        }

        return preview.ID;
    }
    virtual void displayRow(size_t index,  Mat &img)
    {
        int count = 0;
        
        displayedIds[index].clear();
        
        map<int, AnnotationPreview>::reverse_iterator rit;
        for ( rit  = process[index]->previews.rbegin();
              (rit != process[index]->previews.rend()) && (count < maxAnnotations);
              ++rit)
        {
            if (rit->second.fromFrameN > process[index]->currentFrameN )
            {
                continue;
            }
            else
            {
                int _id = drawPreview(index, count, rit->second, img);
                displayedIds[index].push_back(_id);
                count++;
            }
        }

    }
    virtual int getIDfromBoardIndex(const Point &index)
    {
        int _id = -1;
        if (index.y >= 0 && index.y < displayedIds.size())
        {
            if (index.x >= 0 && index.x < displayedIds[index.y].size())
            {
                _id = displayedIds[index.y][index.x];
            }
        }
        return _id;
    }
    virtual void operator()(const size_t frameN, const Mat &frame, Mat &output)
    {
        
        int baseline = process.size() * (_rows + _padd + _header);
        int height   = baseline + 2   * (_rows + _padd + _header) + _header;
        
        output = Mat(height,
                     maxAnnotations * (_cols + _padd), CV_8UC3, Scalar::all(255));
        
        for (size_t i = 0; i < process.size(); i++)
        {
            displayRow(i, output);
        }
        
        
        //draws link from first selection and _hover
        if (_state >= FIRST)
        {
            line(output,
                 boardIndexToPixelCoord(_match.first) + Point(_cols/2, _rows/2) ,
                 boardIndexToPixelCoord(_hover)      + Point(_cols/2, _rows/2),
                 Color::red, 1, CV_AA);
        }
        
        
        Draw::drawText(output, "Matches: ", Color::red, Point(10, baseline), .4, 1);
        
        int col = 0;
        displayedMatches.clear();
        for (size_t i = 0; i < matches.size() && col < maxAnnotations; i++)
        {
            int fpIdx = matches[i].first.x;
            int fpID  = matches[i].first.y;
            
            int spIdx = matches[i].second.x;
            int spID  = matches[i].second.y;
            
            auto fFound =  find(displayedIds[fpIdx].begin(), displayedIds[fpIdx].end(), fpID);
            auto sFound =  find(displayedIds[spIdx].begin(), displayedIds[spIdx].end(), spID);
            
            if (fFound != displayedIds[fpIdx].end() &&
                sFound != displayedIds[spIdx].end())
            {
            
                AnnotationPreview &first  = process[fpIdx]->previews.at(fpID);
                AnnotationPreview &second = process[spIdx]->previews.at(spID);
            
                drawPreview(process.size(), col, first, output, _header, false);
                drawPreview(process.size()+1, col, second, output,_header, false);
                displayedMatches.push_back(i);
                col++;
            }
        }
    };
    virtual Point getBoardIndexFromCoordinates(int x, int y)
    {
        return Point (x / (_cols + _padd),
                      y / (_rows + _padd + _header));
    }
    
    /**
     *Inherited from MouseListener. Check MouseListener class for details
     */
    virtual void mouseInput(int event, int x, int y, int flags){};
    /**
     *Inherited from MouseListener. Check MouseListener class for details
     */
    virtual void leftButtonDown(int x, int y, int flags)
    {
        _state = (++_state)%3;
        if (_state == FIRST)
        {
            _match.first  = getBoardIndexFromCoordinates(x, y);
            int firstID   = getIDfromBoardIndex(_match.first);
            if (firstID == -1)
                _state = 0;
        }
        if (_state == SECOND)
        {
            _match.second  = getBoardIndexFromCoordinates(x, y);
            
            int firstID = getIDfromBoardIndex(_match.first);
            int seconID = getIDfromBoardIndex(_match.second);
            
            if (firstID != -1 && seconID != -1 && seconID != firstID)
            {
                matches.push_back(make_pair(Point(_match.first.y, firstID),
                                            Point(_match.second.y, seconID)));
                
            }
            _state = EMPTY;
            _match.first  = Point(-1,-1);
            _match.second = Point(-1,-1);
            
        }
    };
    /**
     *Inherited from MouseListener. Check MouseListener class for details
     */
    virtual void rightButtonDown(int x, int y, int flags)
    {
        Point coord = getBoardIndexFromCoordinates(x, y);
        
        if (coord.y >= process.size() && coord.x < displayedMatches.size())
        {
            matches.erase(matches.begin() + displayedMatches[coord.x]);
        }
    };
    /**
     *Inherited from MouseListener. Check MouseListener class for details
     */
    virtual void middleButtonDown(int x, int y, int flags){};
    /**
     *Inherited from MouseListener. Check MouseListener class for details
     */
    virtual void mouseMove(int x, int y, int flags)
    {
        _hover = getBoardIndexFromCoordinates(x, y);
    };
    
    /**
     * Inherited from KeyboardListerner. Check KeyboardListener class for details.
     */
    virtual void keyboardInput(int key){};
    
};


#endif