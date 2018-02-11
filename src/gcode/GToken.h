#pragma once

#include <string>
#include <fstream>

namespace cnsee {

struct GToken
{
    GToken()
        : letter(0), number(0.0f)
    {
    }

    GToken(char l, float n)
        : letter(l), number(n)
    {
    }

    GToken(const std::string& str)
    {
        letter = str[0];
        number = (float)atof(str.c_str()+1);
    }

    char letter;
    float number;
};

struct GLine
{
    std::string raw_line;
    size_t line_number;
    std::vector<GToken> tokens;
};

struct GProgram
{
    std::string name;
    std::vector<GLine> lines;
};

std::ostream& operator<<(std::ostream& os, const GToken& token)
{
    os << token.letter << token.number;
    return os;
}

std::istream& operator>>(std::istream& is, GToken& token)
{
    while(is.good() && std::isspace(is.peek())) is.get();
    token.letter = (char)is.get();
    is >> token.number;
    return is;
}

}
