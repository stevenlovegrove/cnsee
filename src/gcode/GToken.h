#pragma once

#include <string>
#include <fstream>
#include <deque>

namespace cnsee {

struct GToken
{
    GToken()
        : letter(0), number(0.0f)
    {
    }

    GToken(char l, double n)
        : letter(l), number(n)
    {
    }

    GToken(const std::string& str)
    {
        letter = str[0];
        number = atof(str.c_str()+1);
    }

    char letter;
    double number;
};

struct GLine
{
    size_t line_number;
    std::string raw_line;
    std::vector<GToken> tokens;

    bool Contains(const GToken& t)
    {
        return std::find_if(tokens.begin(), tokens.end(), [&t](const GToken&o){
            return t.letter == o.letter && t.number == o.number;
        }) != tokens.end();
    }

    bool Contains(const char letter)
    {
        return std::find_if(tokens.begin(), tokens.end(), [&letter](const GToken&o){
            return letter == o.letter;
        }) != tokens.end();
    }
};

struct GProgram
{
    std::string name;
    std::deque<GLine> lines;
};

std::ostream& operator<<(std::ostream& os, const GToken& token)
{
    os << token.letter << token.number;
    return os;
}

std::ostream& operator<<(std::ostream& os, const GLine& line)
{
    for(const auto& t : line.tokens) os << t;
    os << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const GProgram& prog)
{
    for(const auto& l : prog.lines) {
        if(l.tokens.size()) {
            os << l;
        }
    }
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
