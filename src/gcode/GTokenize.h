#pragma once

#include <map>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include "GToken.h"
#include "../utils/StreamOperatorUtils.h"

namespace cnsee {

inline GLine TokenizeLine(const std::string& str, int line_number)
{
    GLine line;
    line.raw_line = str;
    line.line_number = line_number;

    GToken token;
    int bracket = 0;

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
    }

    return line;
}

inline GProgram TokenizeProgram(const std::string& name, std::istream &is)
{
    GProgram program;
    program.name = name;

    int line_number = 0;

    while(is.good()) {
        std::string raw_line;
        std::getline(is, raw_line);
        program.lines.push_back(TokenizeLine(raw_line, line_number++));
    }

    if(!is.eof()) {
        pango_print_warn("Aborted due to error bit on stream.\n");
    }

    return program;
}

inline GProgram TokenizeProgram(const std::string& filename) {
    std::ifstream file(filename);
    if(file.is_open()) {
        return TokenizeProgram(filename, file);
    }else{
        throw std::runtime_error("Unable to open file: " + filename);
    }
}

}
