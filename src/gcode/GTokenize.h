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
        }else if(bracket > 0 || std::isspace(c)) {
            // ignore input
        }else if(c=='%' || c=='#' || c=='*') {
            // Treat checksums '*..' like end of line comments!
            break;
        }else {
            if(std::isdigit(c) || c == '-' || c == '.') {
                if(!token.letter) {
                    std::cerr << "Received number before charector when parsing token. Ignoring line " << line_number << std::endl;
                    line.tokens.clear();
                    return line;
                }
                const char* s = line.raw_line.c_str() + i;
                const char* e = s + line.raw_line.length() - i;
                const char* fe = ConsumeFloat(s, e, token.number);
                i+= fe-s-1;
            }else{
                // We have received the start of a new symbol
                if(token.letter){
                    if(std::isfinite(token.number)) {
                        // Push the completed previous token
                        line.tokens.push_back(token);
                    }else{
                        // The previous token was incomplete
                        std::cerr << "Incomplete token '" << token.letter << "'. Ignoring line " << line_number << std::endl;
                        line.tokens.clear();
                        return line;
                    }
                }
                // start new token
                token.letter = (char)c;
                token.number = std::numeric_limits<float>::quiet_NaN();
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
