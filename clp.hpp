

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

#ifndef __clp__process__
#define __clp__process__

#include "opencv2/opencv.hpp"
#include <vector>
#include <sstream>
#include <string>

using namespace cv;
using namespace std;
/**
 **  Class to extend the functionality of OpenCV CommandLineParser
 **  Allows to parse an arbitrary number of positional arguments
 **  example:
 **   executable a.txt b.txt c.txt ...
 **/
class CommandLineParserExt
{
    CommandLineParser _clp;
    vector<std::string> pos_args;
    /**
     ** Trimming a string
     **/
    std::string trim(const std::string &s)
    {
        auto front = s.find_first_not_of(' ');
        auto last  = s.find_last_not_of(' ');
        return s.substr(front, (last - front + 1));
    }
    /**
     ** Converts one-based index to zero-based index
     ** while checking the boundaries
     **/
    int zeroBaseIdx(int index) const
    {
        if (index <= 1)
            index = 0;
        else if (index >= n_positional_args())
            index = n_positional_args() - 1;
        else
            index--;
        return index;
    }
public:
    CommandLineParserExt(int argc, const char* const argv[], const std::string& keys) :
    _clp(argc, argv, keys)
    {
        for (int i = 1; i < argc; ++i)
        {
            std::string s(argv[i]);
            s = trim(s);
            if (s[0] == '-') continue;

            pos_args.push_back(s);
        }
    }

    bool check() const { return _clp.check(); }
    bool has(const std::string& name) const { return _clp.has(name); }

    template<typename T>
    T get(const String& name, bool space_delete = true) const
    {
        return _clp.get<T>(name, space_delete);
    }

    template<typename T>
    T get(int index, bool space_delete = true) const
    {
        int idx = zeroBaseIdx(index);
        stringstream ss;
        ss << pos_args[idx];
        T t;
        ss >> t;
        return t;
    }

    void printMessage() const
    {
        _clp.printMessage();
    }

    int n_positional_args() const
    {
        return pos_args.size();
    }
};

#endif