#pragma once

#include <istream>
#include <cctype>

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

    template<typename T>
    inline const char* ConsumeFloat(char const* start, char const* end, T& f)
    {
        T s = 1;
        T v = 0;
        T dec = 0;

        int c = *start;
        while( start != end && (std::isdigit(c) || c == '.' || c == '-' || c == '+' ) )
        {
            if( c=='+') {
                // ignore
            }else if( c=='-') {
                s = -1;
            }else if( c=='.') {
                dec = 1;
            }else{
                const T d = T(c - '0');
                v = 10.0f*v + d;
                if(dec) dec *= 10;
            }
            ++start;
            c = *start;
        }

        f = dec ? s*v/dec : s*v;
        return start;
    }

    inline double ConsumeFloat(std::istream &is)
    {
        double s = 1;
        double v = 0;
        double dec = 0;

        int c = is.peek();
        while(is.good() && (std::isdigit(c) || c == '.' || c == '-' || c == '+' ) )
        {
            if( c=='+') {
                // ignore
            }else if( c=='-') {
                s = -1;
            }else if( c=='.') {
                dec = 1;
            }else{
                const double d = double(c - '0');
                v = 10.0*v + d;
                if(dec) dec *= 10.0;
            }
            is.get();
            c = is.peek();
        }

        return dec ? s*v/dec : s*v;
    }
}

namespace Eigen
{
    template<typename Derived>
    inline std::istream& operator>>( std::istream& is, Eigen::MatrixBase<Derived>& mat)
    {
        size_t rows = mat.rows();
        for( size_t r = 0; r < rows-1; r++ ) {
            is >> mat[r];
            is.get();
        }
        is >> mat[rows-1];
        return is;
    }
}
