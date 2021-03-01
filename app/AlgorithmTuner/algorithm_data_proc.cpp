#include "algorithm_data_proc.hpp"


AlgorithmDataProc::AlgorithmDataProc():t_thresh(1), r_thresh(1){
}

AlgorithmDataProc::AlgorithmDataProc(double t_thresh, double r_thresh):t_thresh(t_thresh), r_thresh(r_thresh){
}

void AlgorithmDataProc::set_column_names(rapidcsv::CSVDocPtr &csvDoc) {
    column_name_indices.clear();
    int col_i = csvDoc->GetColumnCount();
    for(auto &name:column_names){
        csvDoc->SetColumnName(col_i, name);
        column_name_indices[name]=col_i++;
    }
}
void AlgorithmDataProc::append_processed_data_to_doc(rapidcsv::CSVDocPtr &csvDoc,int row_i, std::string &algName,DatasetObjectPtr &objPtr,int dpI, HVResult &hvResult) {
    DataPoint &dp = objPtr->data_points[dpI];
    std::vector<int> correct_oc_i;
    std::vector<double> t_dists,r_dists;
    tu::find_correct_ocs(dp.ocs, dp.gts, t_thresh, r_thresh,correct_oc_i,t_dists,r_dists,objPtr->symmetry_transforms);
    double t_dist_avr = std::accumulate(t_dists.begin(),t_dists.end(),0.0)/t_dists.size();
    double r_dist_avr = std::accumulate(r_dists.begin(),r_dists.end(),0.0)/r_dists.size();
    double t_dist_std = std::sqrt(std::accumulate( t_dists.begin(),t_dists.end(),0.0,[t_dist_avr](const double &a,const double &b){return a+std::pow(t_dist_avr-b,2);} ) / t_dists.size());
    double r_dist_std = std::sqrt(std::accumulate( r_dists.begin(),r_dists.end(),0.0,[r_dist_avr](const double &a,const double &b){return a+std::pow(r_dist_avr-b,2);} ) / r_dists.size());

    int tp=0,tn=0,fp=0,fn=0;
    tu::getFPTN(tp, tn, fp, fn, hvResult.chromosome, correct_oc_i);

    double accuracy  = static_cast<double>( tp + tn) / static_cast<double>(tp + tn + fp + fn);
    double f1  = static_cast<double>( 2*tp) / static_cast<double>(2*tp + fp + fn);
    double recall  = static_cast<double>(tp) / dp.gts.size();
    double precision  = static_cast<double>( tp) / static_cast<double>(tp + fp);
    if(std::isnan(accuracy)) accuracy = 0;
    if(std::isnan(recall)) recall = 0;
    if(std::isnan(precision)) precision = 0;
    if(std::isnan(f1)) f1 = 0;

    //Write values to doc
    //{"algName","objName","dpI","chromosome","tp","tn","fp","fn","precision","recall","accuracy","f1","time","cost","t_dist_avr","r_dist_avr","t_dist_std","r_dist_std"};

    csvDoc->SetCell(column_name_indices["algName"],row_i,algName);
    csvDoc->SetCell(column_name_indices["objName"],row_i,objPtr->name);
    csvDoc->SetCell(column_name_indices["dpI"],row_i,dpI);
    csvDoc->SetCell(column_name_indices["chromosome"],row_i,hvResult.chromosome);
    csvDoc->SetCell(column_name_indices["tp"],row_i,tp);
    csvDoc->SetCell(column_name_indices["tn"],row_i,tn);
    csvDoc->SetCell(column_name_indices["fp"],row_i,fp);
    csvDoc->SetCell(column_name_indices["fn"],row_i,fn);
    csvDoc->SetCell(column_name_indices["precision"],row_i,precision);
    csvDoc->SetCell(column_name_indices["recall"],row_i,recall);
    csvDoc->SetCell(column_name_indices["accuracy"],row_i,accuracy);
    csvDoc->SetCell(column_name_indices["f1"],row_i,f1);
    csvDoc->SetCell(column_name_indices["time"],row_i,hvResult.time);
    csvDoc->SetCell(column_name_indices["cost"],row_i,hvResult.cost);
    csvDoc->SetCell(column_name_indices["t_dist_avr"],row_i,t_dist_avr);
    csvDoc->SetCell(column_name_indices["r_dist_avr"],row_i,r_dist_avr);
    csvDoc->SetCell(column_name_indices["t_dist_std"],row_i,t_dist_std);
    csvDoc->SetCell(column_name_indices["r_dist_std"],row_i,r_dist_std);


/*    uniqAlgNames = algName;
    uniqObjNames = objName;
    sort(uniqAlgNames.begin(), uniqAlgNames.end() );
    uniqAlgNames.erase(unique(uniqAlgNames.begin(), uniqAlgNames.end() ), uniqAlgNames.end() );
    sort(uniqObjNames.begin(), uniqObjNames.end() );
    uniqObjNames.erase(unique(uniqObjNames.begin(), uniqObjNames.end() ), uniqObjNames.end() );*/
}




/*
size_t AlgorithmDataProc::begin_index(std::string key, std::vector<std::string> &keys) {
    return std::distance(keys.begin(),std::find(keys.begin(),keys.end(),key));
}

size_t AlgorithmDataProc::end_index(std::string key, std::vector<std::string> &keys) {
    return std::distance(keys.begin(),std::find(keys.rbegin(),keys.rend(),key).base());
}



matplot::figure_handle  AlgorithmDataProc::bar_plot(std::string value_name) {
   using namespace matplot;
    auto f = figure(true);
    auto ax = f->current_axes();

    std::vector<double> valVec = derivedCSVDocPtr->GetColumn<double>(value_name);

    std::vector<std::vector<double>> y;
    for(auto &alg:uniqAlgNames){
        y.emplace_back();
        for(auto &obj:uniqObjNames){
            y.back().emplace_back(get_avr(valVec,alg,obj));
        }
    }

    auto bar_handle = ax->bar(y);
    std::string xl = "Object Type(";
    for(auto &alg:uniqAlgNames)
        xl+=alg+", ";
    xl = xl.substr(0,xl.npos-2)+")";
    ax->xlabel(xl);
    ax->x_axis().ticklabels(uniqObjNames);

    std::vector<std::string> legends;
    for(auto &alg:uniqAlgNames)
        legends.emplace_back(alg + " obj avr");
    auto legend_handle = legend(ax);

    f->draw();
    return f;

}


bool AlgorithmDataProc::get_begin_end_index(size_t &bi, size_t &ei, std::string alg_key, std::string obj_key) {
    {
        if(obj_key!=""){
            bi = std::max(begin_index(alg_key, algName), begin_index(obj_key, objName));
            ei = std::min(end_index(alg_key, algName), end_index(obj_key, objName));
        }else{
            bi = begin_index(alg_key, algName);
            ei = end_index(alg_key, algName);
        }
        return( bi!=ei && bi < algName.size());
    }
}

double AlgorithmDataProc::get_sum(std::vector<double> &vals, std::string alg_key, std::string obj_key) {
        size_t bi,ei;
        if(get_begin_end_index(bi,ei,alg_key,obj_key))
            return std::accumulate(vals.begin()+bi,vals.begin()+ei,static_cast<double>(0));
        else
            return -1;
}

double AlgorithmDataProc::get_avr(std::vector<double> &vals, std::string alg_key, std::string obj_key) {
        size_t bi,ei;
        if(get_begin_end_index(bi,ei,alg_key,obj_key))
            return std::accumulate(vals.begin()+bi,vals.begin()+ei,static_cast<double>(0))/static_cast<double>(ei-bi);
        else
            return -1;
}

int AlgorithmDataProc::get_sum(std::vector<int> &vals, std::string alg_key, std::string obj_key) {
    size_t bi,ei;
    if(get_begin_end_index(bi,ei,alg_key,obj_key))
        return std::accumulate(vals.begin()+bi,vals.begin()+ei,static_cast<int>(0));
    else
        return -1;
}

double AlgorithmDataProc::get_avr(std::vector<int> &vals, std::string alg_key, std::string obj_key) {
    size_t bi,ei;
    if(get_begin_end_index(bi,ei,alg_key,obj_key))
        return std::accumulate(vals.begin()+bi,vals.begin()+ei,static_cast<double>(0))/static_cast<double>(ei-bi);
    else
        return -1;
}
*/






