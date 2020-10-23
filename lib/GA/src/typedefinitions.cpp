#include "ga/typedefinitions.hpp"

std::ostream &operator<<(std::ostream &os, chromosomeT chromosome) {
    for (bool gene:chromosome) {
        os << gene;
    }
    return os;
};

std::ostream &operator<<(std::ostream &os, populationT population) {
    os << "Population:\n";
    for (chromosomeT chromosome:population) {
        os << "\t" << chromosome << "\n";
    }
    os << "\n";
    return os;
};