#include <iostream>
#include <chronometer.h>
#include "datautil/csv_doc.hpp"
#include "datautil/ga_conversions.hpp"
#include "matplot/matplot.h"

using namespace matplot;

int main(int argc, char** argv) {
    std::string results_path = argv[1];
    std::string cost_history_path_ga = argv[2];

    rapidcsv::CSVRReadDoc result_doc(results_path,rapidcsv::LabelParams(0,0));
    rapidcsv::CSVRReadDoc cost_history_doc(cost_history_path_ga,rapidcsv::LabelParams(-1,0));

    std::cout<<result_doc.GetRowCount()<<" "<<result_doc.GetColumnCount()<<" "<<cost_history_doc.GetRowCount() <<" "<<cost_history_doc.GetColumnCount() <<std::endl;

    int gaidx= result_doc.GetColumnIdx("GA_Chromosome");
    int baidx= result_doc.GetColumnIdx("BA_Chromosome");
    for(int i = 0;i<result_doc.GetRowCount();i++) {
        chromosomeT gaChromosome = result_doc.GetCell<chromosomeT>(gaidx,i);
        chromosomeT baChromosome = result_doc.GetCell<chromosomeT>(baidx,i);
        std::cout<<"GA chromosom: \t"<<gaChromosome<<"\n";
        std::cout<<"BA chromosom: \t"<<baChromosome<<std::endl;
    }

    std::vector<std::string> names;
    for(int i = 0;i<cost_history_doc.GetRowCount();i++) {
        names.emplace_back("ch "+std::to_string(i));
        std::vector<double> costhist = cost_history_doc.GetRow<double>(i);
        plot(costhist);
        hold(true);
    }
    hold(false);
    title("GA cost history");
    xlabel("itteration #");
    ylabel("Cost");
    legend(names);
    show();
}
