#pragma once

#include <map>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include "GToken.h"
#include "../utils/StreamOperatorUtils.h"

namespace cnsee {

inline GProgram TokenizeGCode(const std::string& name, std::istream &is)
{
    GProgram program;
    program.name = name;

    GLine line;
    line.line_number = 0;

    GToken token;

    int bracket = 0;

    while(is.good()) {
        std::getline(is, line.raw_line);

        for(size_t i=0; i < line.raw_line.length(); ++i)
        {
            const int c = line.raw_line[i];

            if(c=='(') {
                ++bracket;
            }else if(c==')') {
                --bracket;
            }else if(bracket > 0) {
                // ignore input
            }else if(c=='%' || c=='#' || c=='*') {
                // Treat checksums '*..' like end of line comments!
                continue;
            }else if(std::isspace(c)) {
                if(token.letter) {
                    // Add token to line
                    line.tokens.push_back(token);
                    token.letter = 0;
                }
            }else {
                if(std::isdigit(c) || c == '-' || c == '.') {
                    const char* s = line.raw_line.c_str() + i;
                    const char* e = s + line.raw_line.length() - i;
                    const char* fe = ConsumeFloat(s, e, token.number);
                    i+= fe-s-1;
                }else if(token.letter){
                    // start new token
                    line.tokens.push_back(token);
                    token.letter = (char)c;
                }else{
                    // Set letter for next token
                    token.letter = (char)c;
                }
            }
        }

        if(token.letter) {
            // Add token to line
            line.tokens.push_back(token);
            token.letter = 0;
        }

        // Add line to program
        program.lines.push_back(line);

        // Initialise for next line
        ++line.line_number;
        line.tokens.clear();
    }

    if(!is.eof()) {
        pango_print_warn("Aborted due to error bit on stream.\n");
    }

    return program;
}

inline GProgram TokenizeGCode(const std::string& filename) {
    std::ifstream file(filename);
    if(file.is_open()) {
        return TokenizeGCode(filename, file);
    }else{
        throw std::runtime_error("Unable to open file: " + filename);
    }
}

}
