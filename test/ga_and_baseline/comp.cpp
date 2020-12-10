#include <iostream>
#include <chronometer.h>
#include <algorithm>
#include "datautil/csv_doc.hpp"
#include "datautil/ga_conversions.hpp"
#include "matplot/matplot.h"


using namespace matplot;

void to_row_percent(std::vector<int> &tp,std::vector<int> &fp,std::vector<int> &tn,std::vector<int> &fn,std::vector<double> &tp_pct,std::vector<double> &fp_pct,std::vector<double> &tn_pct,std::vector<double> &fn_pct){
    for(int i = 0;i<tp.size();i++){
        int ocs =  tp[i] +fp[i] +tn[i] +fn[i];
        tp_pct.push_back(tp[i]/static_cast<double>(ocs));
        fp_pct.push_back(fp[i]/static_cast<double>(ocs));
        tn_pct.push_back(tn[i]/static_cast<double>(ocs));
        fn_pct.push_back(fn[i]/static_cast<double>(ocs));
    }
}

int main(int argc, char** argv) {
    std::string folder_name = argv[1];
    std::string results_path = folder_name+argv[2];
    std::string cost_history_path_ga = folder_name+argv[3];
    std::string cost_history_path_ba = folder_name+argv[4];

    rapidcsv::CSVRReadDoc result_doc(results_path,rapidcsv::LabelParams(0,0));
    rapidcsv::CSVRReadDoc ga_cost_history_doc(cost_history_path_ga, rapidcsv::LabelParams(-1, 0));
    rapidcsv::CSVRReadDoc ba_cost_history_doc(cost_history_path_ba, rapidcsv::LabelParams(-1, 0));

    std::vector<chromosomeT> ga_chromosomes= result_doc.GetColumn<chromosomeT>(result_doc.GetColumnIdx("GA_Chromosome"));
    std::vector<chromosomeT> ba_chromosomes= result_doc.GetColumn<chromosomeT>(result_doc.GetColumnIdx("BA_Chromosome"));
    std::vector<int> ga_tp = result_doc.GetColumn<int>(result_doc.GetColumnIdx("#ga_tp"));
    std::vector<int> ga_fp = result_doc.GetColumn<int>(result_doc.GetColumnIdx("#ga_fp"));
    std::vector<int> ga_tn = result_doc.GetColumn<int>(result_doc.GetColumnIdx("#ga_tn"));
    std::vector<int> ga_fn = result_doc.GetColumn<int>(result_doc.GetColumnIdx("#ga_fn"));
    std::vector<int> ba_tp = result_doc.GetColumn<int>(result_doc.GetColumnIdx("#ba_tp"));
    std::vector<int> ba_fp = result_doc.GetColumn<int>(result_doc.GetColumnIdx("#ba_fp"));
    std::vector<int> ba_tn = result_doc.GetColumn<int>(result_doc.GetColumnIdx("#ba_tn"));
    std::vector<int> ba_fn = result_doc.GetColumn<int>(result_doc.GetColumnIdx("#ba_fn"));
    std::vector<double> ga_tp_pct,ga_fp_pct,ga_tn_pct,ga_fn_pct,ba_tp_pct,ba_fp_pct,ba_tn_pct,ba_fn_pct;
    to_row_percent(ga_tp,ga_fp,ga_tn,ga_fn,ga_tp_pct,ga_fp_pct,ga_tn_pct,ga_fn_pct);
    to_row_percent(ba_tp,ba_fp,ba_tn,ba_fn,ba_tp_pct,ba_fp_pct,ba_tn_pct,ba_fn_pct);

    std::map<std::string,std::vector<std::string>> row_groups;
    std::vector<std::string> row_names = result_doc.GetRowNames();
    std::vector<int> start_idxs;
    std::string name;
    for (int i = 0; i<row_names.size();i++){
        auto & rn = row_names[i];
        auto pos = rn.find("_");
        if(pos != rn.npos) {
            name = rn.substr(0, pos);
            row_groups[name].emplace_back(rn);
        }
    }


    tiledlayout(1,row_groups.size());
    for(auto &row_group:row_groups) {
            nexttile();
            int ridx_start = result_doc.GetRowIdx(row_group.second[0]);
            int ridx_end = result_doc.GetRowIdx(row_group.second.back());
            std::vector<std::vector<double>> bar_data;// = {{2, 2, 2, 2}, {2, 5, 8, 11}, {3, 6, 9, 12}};

            int ga_tp_sum = std::accumulate(ga_tp.begin()+ridx_start,ga_tp.begin()+ridx_end+1,0);
            int ga_fp_sum = std::accumulate(ga_fp.begin()+ridx_start,ga_fp.begin()+ridx_end+1,0);
            int ga_tn_sum = std::accumulate(ga_tn.begin()+ridx_start,ga_tn.begin()+ridx_end+1,0);
            int ga_fn_sum = std::accumulate(ga_fn.begin()+ridx_start,ga_fn.begin()+ridx_end+1,0);
            int ba_tp_sum = std::accumulate(ba_tp.begin()+ridx_start,ba_tp.begin()+ridx_end+1,0);
            int ba_fp_sum = std::accumulate(ba_fp.begin()+ridx_start,ba_fp.begin()+ridx_end+1,0);
            int ba_tn_sum = std::accumulate(ba_tn.begin()+ridx_start,ba_tn.begin()+ridx_end+1,0);
            int ba_fn_sum = std::accumulate(ba_fn.begin()+ridx_start,ba_fn.begin()+ridx_end+1,0);
            bar_data.resize(4);
            bar_data[0].emplace_back(ga_tp_sum);
            bar_data[1].emplace_back(ga_fp_sum);
            bar_data[2].emplace_back(ga_tn_sum);
            bar_data[3].emplace_back(ga_fn_sum);
            bar_data[0].emplace_back(ba_tp_sum);
            bar_data[1].emplace_back(ba_fp_sum);
            bar_data[2].emplace_back(ba_tn_sum);
            bar_data[3].emplace_back(ba_fn_sum);
            bar(bar_data);
            title(row_group.first+" Sum");
            xticklabels(std::vector<std::string>{"GA","BA"});
    }
    save(folder_name+"results_sum.svg");

    tiledlayout(1,row_groups.size());
    for(auto &row_group:row_groups) {
        nexttile();
        int ridx_start = result_doc.GetRowIdx(row_group.second[0]);
        int ridx_end = result_doc.GetRowIdx(row_group.second.back());
        std::vector<std::vector<double>> bar_data;// = {{2, 2, 2, 2}, {2, 5, 8, 11}, {3, 6, 9, 12}};

        double ga_tp_sum = std::accumulate(ga_tp.begin()+ridx_start,ga_tp.begin()+ridx_end+1,0)/(double)(ridx_end-ridx_start);
        double ga_fp_sum = std::accumulate(ga_fp.begin()+ridx_start,ga_fp.begin()+ridx_end+1,0)/(double)(ridx_end-ridx_start);
        double ga_tn_sum = std::accumulate(ga_tn.begin()+ridx_start,ga_tn.begin()+ridx_end+1,0)/(double)(ridx_end-ridx_start);
        double ga_fn_sum = std::accumulate(ga_fn.begin()+ridx_start,ga_fn.begin()+ridx_end+1,0)/(double)(ridx_end-ridx_start);
        double ba_tp_sum = std::accumulate(ba_tp.begin()+ridx_start,ba_tp.begin()+ridx_end+1,0)/(double)(ridx_end-ridx_start);
        double ba_fp_sum = std::accumulate(ba_fp.begin()+ridx_start,ba_fp.begin()+ridx_end+1,0)/(double)(ridx_end-ridx_start);
        double ba_tn_sum = std::accumulate(ba_tn.begin()+ridx_start,ba_tn.begin()+ridx_end+1,0)/(double)(ridx_end-ridx_start);
        double ba_fn_sum = std::accumulate(ba_fn.begin()+ridx_start,ba_fn.begin()+ridx_end+1,0)/(double)(ridx_end-ridx_start);
        bar_data.resize(4);
        bar_data[0].emplace_back(ga_tp_sum);
        bar_data[1].emplace_back(ga_fp_sum);
        bar_data[2].emplace_back(ga_tn_sum);
        bar_data[3].emplace_back(ga_fn_sum);
        bar_data[0].emplace_back(ba_tp_sum);
        bar_data[1].emplace_back(ba_fp_sum);
        bar_data[2].emplace_back(ba_tn_sum);
        bar_data[3].emplace_back(ba_fn_sum);
        bar(bar_data);
        title(row_group.first+" Average");
        xticklabels(std::vector<std::string>{"GA","BA"});
    }
    save(folder_name+"results_avr.svg");

    tiledlayout(1,row_groups.size());
    for(auto &row_group:row_groups) {
        nexttile();
        int ridx_start = result_doc.GetRowIdx(row_group.second[0]);
        int ridx_end = result_doc.GetRowIdx(row_group.second.back());
        std::vector<std::vector<double>> bar_data;// = {{2, 2, 2, 2}, {2, 5, 8, 11}, {3, 6, 9, 12}};

        double ga_tp_sum = std::accumulate(ga_tp_pct.begin()+ridx_start,ga_tp_pct.begin()+ridx_end+1,0.0)/(double)(ridx_end-ridx_start);
        double ga_fp_sum = std::accumulate(ga_fp_pct.begin()+ridx_start,ga_fp_pct.begin()+ridx_end+1,0.0)/(double)(ridx_end-ridx_start);
        double ga_tn_sum = std::accumulate(ga_tn_pct.begin()+ridx_start,ga_tn_pct.begin()+ridx_end+1,0.0)/(double)(ridx_end-ridx_start);
        double ga_fn_sum = std::accumulate(ga_fn_pct.begin()+ridx_start,ga_fn_pct.begin()+ridx_end+1,0.0)/(double)(ridx_end-ridx_start);
        double ba_tp_sum = std::accumulate(ba_tp_pct.begin()+ridx_start,ba_tp_pct.begin()+ridx_end+1,0.0)/(double)(ridx_end-ridx_start);
        double ba_fp_sum = std::accumulate(ba_fp_pct.begin()+ridx_start,ba_fp_pct.begin()+ridx_end+1,0.0)/(double)(ridx_end-ridx_start);
        double ba_tn_sum = std::accumulate(ba_tn_pct.begin()+ridx_start,ba_tn_pct.begin()+ridx_end+1,0.0)/(double)(ridx_end-ridx_start);
        double ba_fn_sum = std::accumulate(ba_fn_pct.begin()+ridx_start,ba_fn_pct.begin()+ridx_end+1,0.0)/(double)(ridx_end-ridx_start);
        bar_data.resize(4);
        bar_data[0].emplace_back(ga_tp_sum);
        bar_data[1].emplace_back(ga_fp_sum);
        bar_data[2].emplace_back(ga_tn_sum);
        bar_data[3].emplace_back(ga_fn_sum);
        bar_data[0].emplace_back(ba_tp_sum);
        bar_data[1].emplace_back(ba_fp_sum);
        bar_data[2].emplace_back(ba_tn_sum);
        bar_data[3].emplace_back(ba_fn_sum);
        bar(bar_data);
        title(row_group.first+" Percentage Average");
        xticklabels(std::vector<std::string>{"GA","BA"});
    }
    save(folder_name+"results_avr_pct.svg");


/*    std::vector<double> costhist;


    std::vector<std::string> names;
    names.emplace_back("ch "+std::to_string(0));
    costhist = ga_cost_history_doc.GetRow<double>(0);
    plot(costhist);
    hold(on);
    for(int i = 1; i < ga_cost_history_doc.GetRowCount(); i++) {
        names.emplace_back("ch "+std::to_string(i));
        costhist = ga_cost_history_doc.GetRow<double>(i);
        plot(costhist);
    }
    hold(off);
    title("GA cost history");
    xlabel("itteration #");
    ylabel("Cost");
    show();

    names.clear();
    names.emplace_back("ch "+std::to_string(0));
    costhist = ba_cost_history_doc.GetRow<double>(0);
    plot(costhist);
    hold(on);
    for(int i = 1; i < ba_cost_history_doc.GetRowCount(); i++) {
        names.emplace_back("ch "+std::to_string(i));
        costhist = ba_cost_history_doc.GetRow<double>(i);
        plot(costhist);
    }
    hold(off);
    title("BA cost history");
    xlabel("itteration #");
    ylabel("Cost");
    show();*/
}
