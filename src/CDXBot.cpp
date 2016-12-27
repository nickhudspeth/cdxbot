/************************************************************************
Title:    cdxbot.cpp
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     cdxbot.cpp
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file cdxbot.h.

LICENSE:
    Copyright (C) 2016 Nicholas Morrow

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

************************************************************************/

/**********************    INCLUDE DIRECTIVES    ***********************/
#include "CDXBot.h"


/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/


/*******************    FUNCTION IMPLEMENTATIONS    ********************/



int CDXBot::parseHLMDFile(const char *fname) {
    std::ifstream infile(fname);
    std::string line;
    std::vector<std::string> v;
    std::string s;
    struct action a;
    while(std::getline(infile, line)) {
        if(line.empty()) {
            return -1;
            break;
        }
        /* Strip whitespace from string */
        std::string::iterator end_pos = std::remove(line.begin(), line.end(), ' ');
        line.erase(end_pos, line.end());
        // Assemble an action structure from the parsed tokens in the line.
        if(!split(line, d, v)) {
            // a.fp = getActionPointer(v[0]);
            a.cmd = v[0];
            for(size_t i = 1; i < v.size(); i++) {
                a.args.push_back(stof(v[i]));
            }
        }
    }
    for(size_t i = 0; i < v.size(); i++) {
        std::cout << v[i] << std::endl;
    }
    return 0;
}

int CDXBot::split(const std::string &s, char c, std::vector<std::string> &v) {
    std::string::size_type i = 0;
    std::string::size_type j = s.find(c);

    while(j != std::string::npos) {
        v.push_back(s.substr(i, j-1));
        i = ++j;
        j = s.find(c, j);
        if(j == std::string::npos) {
            v.push_back(s.substr(i,s.length()));
        }
    }
    return 0;
}

void CDXBot::setRunStatus(unsigned int s) {
    if(s > 0) {
        _runStatus = 1;
    } else {
        _runStatus = 0;
    }
}
