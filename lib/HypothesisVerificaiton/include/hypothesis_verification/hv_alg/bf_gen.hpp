#ifndef MASTER_BF_GEN_HPP
#define MASTER_BF_GEN_HPP
#include <bitset>
#include <hypothesis_verification/typedefinitions.hpp>

class BF_GEN {
public:
    int n_max;
    int N = 0;
    int n=-1;
    const std::bitset<64> bs;
    chromosomeT chromosome;

    BF_GEN(unsigned int N):N(N){
        chromosome.resize(N);
        n_max = static_cast<int>(std::pow(2,N));
    }
    bool next(){
        n++;
        if(n==n_max)
            return false;

        std::bitset<64> bs((n >> 1) ^ n);

        for(int i = 0;i<N;i++)
            chromosome[i]=bs[i];
        return true;
    }
};


#endif //MASTER_BF_GEN_HPP
