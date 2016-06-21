
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

#ifndef __mprocessor__process__
#define __mprocessor__process__

#include "viva.h"
using namespace viva; 

/**
	* SeekProcessor class.
	* Entry point of a vivalib program that allows to seek to sequence frames online.
	* The input class needs to be capable of this feature for the seek Processor to work
	* We need to define an Input which process the seek argument in the getFrame method
	* This class will not work will multithreads as the Processor Class
	*/
class SeekProcessor
{
private:
    Ptr<Input>   _input;
    Ptr<ProcessFrame> _process;
    Ptr<Output>  _output;

    function<void(const size_t frameN, const Mat &frame, Mat &output)> _functor;

    string _inputWindowName;
    string _outputWindowName;

    bool _showInput;
    bool _showOutput;

    bool _mListener;
    bool _kListener;

    size_t _inputBufferSize;
    size_t _outputBufferSize;

    bool _pause;

    static void mouseCallback(int event, int x, int y, int flags, void *ptr);

public:

    SeekProcessor() :
    _input(new CameraInput(-1)),
    _process(new ProcessFrame()),
    _output(Ptr<Output>()),
    _functor(nullptr),
    _inputWindowName("Input"),
    _outputWindowName("Process Output"),
    _showInput(false),
    _showOutput(true),
    _mListener(false),
    _kListener(false),
    _inputBufferSize(10),
    _outputBufferSize(10),
    _pause(false)
    {}

    SeekProcessor(int argc, const char * argv[]) : SeekProcessor()
    {

    }
    void setInputBufferSize(size_t size)
    {
        _inputBufferSize = size;
    }

    void setOutputBufferSize(size_t size)
    {
        _outputBufferSize = size;
    }

    void showInput(bool show = true)
    {
        _showInput = show;
    }

    void showOutput(bool show = true)
    {
        _showOutput = show;
    }
    void setInputWindowName(const string &name)
    {
        _inputWindowName = name;
    }
    void setOutputWindowName(const string &name)
    {
        _outputWindowName = name;
    }
    void setInput(Ptr<Input> &input)
    {
        _input = input;
    }
    void setOutput(Ptr<Output> &output)
    {
        _output = output;
    }
    void listenToMouseEvents()
    {
        _mListener = true;
    }
    void listenToKeyboardEvents()
    {
        _kListener = true;
    }
    void setProcess(Ptr<ProcessFrame> &process)
    {
        _process = process;
    }
    /**
     * Makes to pause the sequence at the fist frame. Waiting for the user
     * to hit the SPACE key in the keyboard to continue the execution of the sequence.
     * It's usefull when manual user selection is needed before running the algorithm
     */
    void startPaused()
    {
        _pause = true;
    }

    /**
     * Set a process
     */
    void setProcess(function<void(const size_t frameN, const Mat &frame, Mat &output)> functor)
    {
        _functor = functor;
    }

    /**
     * Method to execute after defining the Processor's Input, ProcessFrame, and Output(optional)
     * The input, process and output will run in three different threads and
     * will comunicate using BuffedChannels between them.
     * To exit you should press the ESC key. The SPACE key will allow to pause and/or continue the video
     * sequence
     */
    void run(int startFrame = 0);
};

class MultipleProcess;

struct ProcessListenerWrapper
{
    int identifier;
    MouseListener *processListener;
    MultipleProcess *processor;

    ProcessListenerWrapper(const int ID, MouseListener *process, MultipleProcess *proc):
    identifier(ID), processListener(process), processor(proc)
    {}
};

/**
 * MultipleProcess class.
 * Entry point of a vivalib program that allows to seek to sequence frames online.
 * The input class needs to be capable of this feature for the seek Processor to work
 * We need to define an Input which process the seek argument in the getFrame method
 * This class will not work will multithreads as the Processor Class
 */
class MultipleProcess
{
private:
    vector<Ptr<Input>>  _input;
	vector<int>			_startFrame;
    vector<Ptr<ProcessFrame>> _process;
    vector<Ptr<Output>>  _output;
    string _windowName;

    bool _showInput;
    bool _showOutput;

    bool _mListener;
    bool _kListener;

    size_t _inputBufferSize;
    size_t _outputBufferSize;

    bool _pause;
    int _activeWindow;

    static void mouseCallback(int event, int x, int y, int flags, void *ptr);

public:

    MultipleProcess() :
    _input(),
    _process(),
    _output(),
    _windowName("Sequence"),
    _showInput(false),
    _showOutput(true),
    _mListener(false),
    _kListener(false),
    _inputBufferSize(10),
    _outputBufferSize(10),
    _pause(false),
    _activeWindow(0)
    {}

    MultipleProcess(int argc, const char * argv[]) : MultipleProcess()
    {

    }
    void setActiveWindow(size_t ID)
    {
        _activeWindow = ID;
    }
    void setInputBufferSize(size_t size)
    {
        _inputBufferSize = size;
    }

    void setOutputBufferSize(size_t size)
    {
        _outputBufferSize = size;
    }

    void showInput(bool show = true)
    {
        _showInput = show;
    }

    void showOutput(bool show = true)
    {
        _showOutput = show;
    }
    void setWindowPrefix(const string &name)
    {
        _windowName = name;
    }
    void setInput(vector<Ptr<Input>> &input)
    {
        _input = input;
    }
	void setStartFrame(vector<int> &startFrame)
	{
		_startFrame = startFrame;
	}
    void setOutput(vector<Ptr<Output>> &output)
    {
        _output = output;
    }
    void listenToMouseEvents()
    {
        _mListener = true;
    }
    void listenToKeyboardEvents()
    {
        _kListener = true;
    }
    void setProcess(vector<Ptr<ProcessFrame>> &process)
    {
        _process = process;
    }
    /**
     * Makes to pause the sequence at the fist frame. Waiting for the user
     * to hit the SPACE key in the keyboard to continue the execution of the sequence.
     * It's usefull when manual user selection is needed before running the algorithm
     */
    void startPaused()
    {
        _pause = true;
    }

    /**
     * Method to execute after defining the Processor's Input, ProcessFrame, and Output(optional)
     * The input, process and output will run in three different threads and
     * will comunicate using BuffedChannels between them.
     * To exit you should press the ESC key. The SPACE key will allow to pause and/or continue the video
     * sequence
     */
    void run();
};


#endif