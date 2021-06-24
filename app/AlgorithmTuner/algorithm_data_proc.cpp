#include "algorithm_data_proc.hpp"


AlgorithmDataProc::AlgorithmDataProc() : t_thresh(1), r_thresh(1) {
}

AlgorithmDataProc::AlgorithmDataProc(double t_thresh, double r_thresh) : t_thresh(t_thresh), r_thresh(r_thresh) {
}

void AlgorithmDataProc::set_column_names(rapidcsv::CSVDocPtr &csvDoc) {
    column_name_indices.clear();
    int col_i = csvDoc->GetColumnCount();
    for (auto &name:column_names) {
        csvDoc->SetColumnName(col_i, name);
        column_name_indices[name] = col_i++;
    }
}

void AlgorithmDataProc::t_r_dist(chromosomeT &chromosome, std::vector<int> &correct_oc_i, std::vector<double> &t_dists,
                                 std::vector<double> &r_dists, double &t_dist_avr, double &r_dist_avr,
                                 double &t_dist_std,
                                 double &r_dist_std) {
    int n_correct_active_genes = 0;
    t_dist_avr = 0.0;
    r_dist_avr = 0.0;
    t_dist_std = 0.0;
    r_dist_std = 0.0;

    for (int i = 0; i < correct_oc_i.size(); i++) {
        if (chromosome[correct_oc_i[i]]) {
            n_correct_active_genes++;
            t_dist_avr += t_dists[i];
            r_dist_avr += r_dists[i];
        }
    }

    t_dist_avr /= n_correct_active_genes;
    r_dist_avr /= n_correct_active_genes;

    for (int i = 0; i < correct_oc_i.size(); i++) {
        if (chromosome[correct_oc_i[i]]) {
            t_dist_std += std::pow((t_dists[i] - t_dist_avr), 2);
            r_dist_std += std::pow((r_dists[i] - r_dist_avr), 2);
        }
    }
    t_dist_std = std::sqrt(t_dist_std / n_correct_active_genes);
    r_dist_std = std::sqrt(r_dist_std / n_correct_active_genes);

}

void AlgorithmDataProc::append_processed_data_to_doc(rapidcsv::CSVDocPtr &csvDoc, int row_i, std::string &algName,
                                                     DatasetObjectPtr &objPtr, int dpI, HVResult &hvResult,int rep) {
    DataPoint &dp = objPtr->data_points[dpI];
    std::vector<int> correct_oc_i;
    std::vector<double> t_dists, r_dists;
    tu::find_correct_ocs(dp.ocs, dp.gts, t_thresh, r_thresh, correct_oc_i, t_dists, r_dists,
                         objPtr->symmetry_transforms);

    int tp = 0, tn = 0, fp = 0, fn = 0;
    tu::getFPTN(tp, tn, fp, fn, hvResult.chromosome, correct_oc_i);


//    double t_dist_avr = std::accumulate(t_dists.begin(),t_dists.end(),0.0)/t_dists.size();
//    double r_dist_avr = std::accumulate(r_dists.begin(),r_dists.end(),0.0)/r_dists.size();
//    double t_dist_std = std::sqrt(std::accumulate( t_dists.begin(),t_dists.end(),0.0,[t_dist_avr](const double &a,const double &b){return a+std::pow(t_dist_avr-b,2);} ) / t_dists.size());
//    double r_dist_std = std::sqrt(std::accumulate( r_dists.begin(),r_dists.end(),0.0,[r_dist_avr](const double &a,const double &b){return a+std::pow(r_dist_avr-b,2);} ) / r_dists.size());

    double t_dist_avr, r_dist_avr, t_dist_std, r_dist_std;

    t_r_dist(hvResult.chromosome, correct_oc_i, t_dists, r_dists, t_dist_avr, r_dist_avr, t_dist_std, r_dist_std);


    double accuracy = static_cast<double>( tp + tn) / static_cast<double>(tp + tn + fp + fn);
    double f1 = static_cast<double>( 2 * tp) / static_cast<double>(2 * tp + fp + fn);
    double recall = static_cast<double>(tp) / static_cast<double>(tp + fn);
    double precision = static_cast<double>( tp) / static_cast<double>(tp + fp);
    if (std::isnan(accuracy)) accuracy = 0;
    if (std::isnan(recall)) recall = 0;
    if (std::isnan(precision)) precision = 0;
    if (std::isnan(f1)) f1 = 0;

    csvDoc->SetCell(column_name_indices["algName"], row_i, algName);
    csvDoc->SetCell(column_name_indices["objName"], row_i, objPtr->name);
    csvDoc->SetCell(column_name_indices["dpI"], row_i, dpI);
    csvDoc->SetCell(column_name_indices["rep"], row_i, rep);
    csvDoc->SetCell(column_name_indices["chromosome"], row_i, hvResult.chromosome);
    csvDoc->SetCell(column_name_indices["tp"], row_i, tp);
    csvDoc->SetCell(column_name_indices["tn"], row_i, tn);
    csvDoc->SetCell(column_name_indices["fp"], row_i, fp);
    csvDoc->SetCell(column_name_indices["fn"], row_i, fn);
    csvDoc->SetCell(column_name_indices["precision"], row_i, precision);
    csvDoc->SetCell(column_name_indices["recall"], row_i, recall);
    csvDoc->SetCell(column_name_indices["accuracy"], row_i, accuracy);
    csvDoc->SetCell(column_name_indices["f1"], row_i, f1);
    csvDoc->SetCell(column_name_indices["time"], row_i, hvResult.time);
    csvDoc->SetCell(column_name_indices["cost"], row_i, hvResult.cost);
    csvDoc->SetCell(column_name_indices["t_dist_avr"], row_i, t_dist_avr);
    csvDoc->SetCell(column_name_indices["r_dist_avr"], row_i, r_dist_avr);
    csvDoc->SetCell(column_name_indices["t_dist_std"], row_i, t_dist_std);
    csvDoc->SetCell(column_name_indices["r_dist_std"], row_i, r_dist_std);
}




