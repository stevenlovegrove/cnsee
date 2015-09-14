#pragma once

namespace cnsee
{
    // Consume up to, but not including delim from input stream
    inline void ConsumeUntil(std::istream& is, char delim)
    {
        while(is.good() && is.peek() != delim) {
            is.get();
        }
    }

    // Consume comments and whitespace from input stream. Stop before newline.
    inline void ConsumeCommentsAndWhitespace(std::istream &is) {
        int bracket = 0;

        while(is.good()) {
            const char c = is.peek();
            if(c=='\n') {
                return;
            }else if(c=='%') {
                ConsumeUntil(is,'\n');
                return;
            }else if(c=='(') {
                ++bracket;
            }else if(c==')') {
                --bracket;
            }else if(std::isspace(c)) {
                // Allow space to be consumed
            }else if(bracket == 0) {
                return;
            }
            is.get();
        }
    }

}