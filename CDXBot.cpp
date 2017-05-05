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

    /* TODO: nam - Add provision to ignore commented lines - Mon 10 Apr 2017 02:18:09 PM MDT */

    std::ifstream infile(fname);
    std::string line;
    std::vector<std::string> v;
    std::string s;
    if(!infile.good()) {
        return -1;
    }
    while(std::getline(infile, line)) {
        struct action a;
        /* Strip rest of line following a '#' character to allow comments.*/
        int p = line.find('#');
        if(p != std::string::npos) {
            line.erase(p);
        }
        if(!line.empty()) {
            /* Strip whitespace from string */
            std::string::iterator end_pos = std::remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());
            // Assemble an action structure from the parsed tokens in the line.
            // a.fp = getActionPointer(v[0]);
            typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
            boost::char_separator<char> sep(d);
            tokenizer v(line,sep);

            for(tokenizer::iterator iter = v.begin(); iter!=v.end(); ++iter)
                if(iter == v.begin()) {
                    a.cmd = *iter;
                } else {
                    a.args.push_back(std::stof(*iter));
                }
            actionMap.push_back(a);
        }
    }
    return 0;
}


void CDXBot::setRunStatus(unsigned int s) {
    if(s > 0) {
        _runStatus = 1;
        ROS_INFO_STREAM("RunStatus set to 1.");
    } else {
        _runStatus = 0;
        ROS_INFO_STREAM("RunStatus set to 0.");
    }
}

int CDXBot::getNextAction(struct action &a) {
    // if(actionMap.begin() != actionMap.end()) {
    // a.cmd = actionMap[0].cmd;
    // a.args = actionMap[0].args;
    // actionMap.erase(actionMap.begin());
    // return 0;

    // } else {
    // return -1;
    // }
    unsigned int idx = getActionIndex();
    if(idx < (actionMap.size() - 1)) {
        a.cmd = actionMap[idx].cmd;
        a.args = actionMap[idx].args;
        setActionIndex(++idx);
        return 0;
    } else {
        return -1;
    }
}
