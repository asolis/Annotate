
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

#include "mprocessor.hpp"



void MultipleProcess::mouseCallback(int event, int x, int y, int flags, void *ptr)
{
    ProcessListenerWrapper *w   = (ProcessListenerWrapper*)ptr;
    MouseListener   *listener   = w->processListener;
    MultipleProcess *processor  = w->processor;

    processor->setActiveWindow(w->identifier);

    if (listener)
    {
        listener->mouseInput(event, x, y, flags);

        if (event == EVENT_LBUTTONDOWN)
        {
            listener->leftButtonDown(x, y, flags);
        }
        else if (event == EVENT_RBUTTONDOWN)
        {
            listener->rightButtonDown(x, y, flags);
        }
        else if (event == EVENT_MBUTTONDOWN)
        {
            listener->middleButtonDown(x, y, flags);
        }
        else if (event == EVENT_MOUSEMOVE)
        {
            listener->mouseMove(x, y, flags);
        }
    }
}

void MultipleProcess::run()
{

    int FLAGS = CV_GUI_NORMAL | CV_WINDOW_AUTOSIZE;

    if (_input.size() != _process.size())
        return;

    if (_showOutput)
    {
        for (size_t i = 0; i < _input.size(); i++)
        {
            namedWindow(_windowName + to_string(i), FLAGS);
        }

    }
    if (matching)
        namedWindow(_matchinName, FLAGS);
    
    
    vector<Ptr<ProcessListenerWrapper>> wrappers;
    if (_mListener && _process.size())
    {
        for (size_t i = 0; i < _process.size(); i++)
        {
            Ptr<ProcessListenerWrapper> w = new ProcessListenerWrapper(i, _process[i], this);
            wrappers.push_back(w);
            cv::setMouseCallback(_windowName + to_string(i), MultipleProcess::mouseCallback, wrappers[i]);
        }
    }
    if (_mListener && matching)
        cv::setMouseCallback(_matchinName, Processor::mouseCallback, matching);
        
    if (!_input.size() && !_process.size())
        return;


    vector<long> frameN;
    Mat freezeFrame;
    bool freezed = true;
    bool running = true;
    int key = Keys::NONE;

    // initialize the freezeFrame
    bool allSequencesReady = true;
    vector<Mat>  freezedFrames;
    vector<bool> hasFrame;

    for (size_t i = 0; i < _input.size(); i++)
    {
        Mat tmp;
        bool _retrieved = _input[i]->getFrame(tmp, _startFrame);
        allSequencesReady = allSequencesReady && _retrieved;
        freezedFrames.push_back(tmp);
        hasFrame.push_back(_retrieved);
		long frame = 0;
		frame += _startFrame;
		frameN.push_back(frame);
    }

    while (running && allSequencesReady)
    {

        vector<Mat> frame(_input.size()), frameOut(_input.size());
        if (!freezed || key == Keys::n)
        {
            for( size_t i = 0; i < _input.size(); i++)
            {
                hasFrame[i] = _input[i]->getFrame(frame[i], 1);
                freezedFrames[i] = frame[i];
				frameN[i]++;
            }
        }
        else if (!freezed || key == Keys::p)
        {
            for( size_t i = 0; i < _input.size(); i++)
            {
                if (frameN[i] > 0)
                {
                    hasFrame[i] = _input[i]->getFrame(frame[i], -1);
                    freezedFrames[i] = frame[i];
                    frameN[i]--;
                }
            }
        }
        else
        {
            for( size_t i = 0; i < _input.size(); i++)
            {
                frame[i] = freezedFrames[i];
            }
        }

        bool allSequencesFinished = false;
        for (size_t i = 0; i < _input.size(); i++)
        {
            allSequencesFinished = allSequencesFinished || hasFrame[i];
        }

        if (allSequencesFinished)
        {

            for (size_t i = 0; i < _input.size(); i++)
            {

                if (_process.size())
                    _process[i]->operator()(frameN[i], frame[i], frameOut[i]);

                if (_showOutput && !frameOut[i].empty())
                    cv::imshow(_windowName + to_string(i), frameOut[i]);
                if (_output.size())
                    _output[i]->writeFrame(frameOut[i]);
            }
            if (matching)
            {
                Mat tmp, moutput;
                matching->operator()(0, tmp, moutput);
                cv::imshow(_matchinName, moutput);
            }
            key = Keys::NONE;

            try
            {
                key = waitKey(1);
            }
            catch (...)
            {
                //...
            }

            if (key == Keys::ESC)
            {
                running = false;
            }
            if (key == Keys::SPACE || _pause)
            {
                _pause = false;
                for (size_t i = 0; i < _input.size(); i++)
                {
                    freezedFrames[i] = frame[i];
                }
                freezed = !freezed;
            }
            if (_kListener && _process.size() && key != Keys::NONE)
            {
                for (size_t i = 0; i < _input.size(); i++)
                {
                    if (_activeWindow == i)
                        _process[i]->keyboardInput(key);
                }
            }
            
        }
        else
        {
            break;
        }
    }
    
    
    destroyAllWindows();
}
