#ifndef MASTER_GA_CONVERSIONS_HPP
#define MASTER_GA_CONVERSIONS_HPP
#include "rapidcsv.h"
#include "ga/typedefinitions.hpp"

namespace rapidcsv
{
    template<>
    void Converter<chromosomeT>::ToVal(const std::string& pStr, chromosomeT& pVal) const
    {
        pVal.clear();
        pVal.reserve(pStr.size());
        for(auto c:pStr){
            pVal.push_back(c=='1');
        }
    }

    template<>
    void Converter<chromosomeT>::ToStr(const chromosomeT& pVal, std::string& pStr) const
    {
        pStr.clear();
        pStr.reserve(pStr.size());
        for(auto b:pVal){
            if(b)
                pStr.push_back('1');
            else
                pStr.push_back('0');
        }
    }
}


#endif //MASTER_GA_CONVERSIONS_HPP
