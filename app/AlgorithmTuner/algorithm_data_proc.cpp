//
// Created by jens on 1/22/21.
//

#include "algorithm_data_proc.hpp"


AlgorithmDataProc::AlgorithmDataProc(): rawDataMapAlgObjVec(rawDataMapAlgObjVecT()) {
}

AlgorithmDataProc::AlgorithmDataProc(rawDataMapAlgObjVecT newrawDataMapAlgObjVec, double t_thresh, double r_thresh)
        : rawDataMapAlgObjVec(newrawDataMapAlgObjVec) {

    derivedCSVDocPtr = std::make_shared<rapidcsv::CSVDoc>("",rapidcsv::LabelParams(0, -1));
    staticCSVDocPtr = std::make_shared<rapidcsv::CSVDoc>("",rapidcsv::LabelParams(-1, 0));

    size_t ndps = 0;
    for (auto &alg_pair:rawDataMapAlgObjVec) {
        for (auto &obj_pair:alg_pair.second) {
            ndps+=obj_pair.second.size();
        }
    }
    algName.reserve(ndps);
    objName.reserve(ndps);
    nOCVec.reserve(ndps);
    chromosome.reserve(ndps);
    time.reserve(ndps);
    tp.reserve(ndps);
    tn.reserve(ndps);
    fp.reserve(ndps);
    fn.reserve(ndps);
    accuracy.reserve(ndps);
    precision.reserve(ndps);
    recall.reserve(ndps);
    uniqAlgNames.reserve(ndps);
    uniqObjNames.reserve(ndps);

    for (auto &alg_pair:rawDataMapAlgObjVec) {
        for (auto &obj_pair:alg_pair.second) {
            for (auto &rawData:obj_pair.second) {
                std::vector<bool> true_ocs = tu.get_true_ocs(rawData.dp.ocs, rawData.dp.gts, t_thresh, r_thresh,obj_pair.first->symmetry_transforms);
                trueOCVec.emplace_back(true_ocs);
                algName.emplace_back(alg_pair.first);
                objName.emplace_back(obj_pair.first->name);

                nOCVec.emplace_back(rawData.dp.ocs.size());
                chromosome.emplace_back(rawData.chromsome);
                ocVec.emplace_back(rawData.dp.ocs);
                gtVec.emplace_back(rawData.dp.gts);
                time.emplace_back(rawData.time);

                tpIVec.emplace_back();
                tnIVec.emplace_back();
                fpIVec.emplace_back();
                fnIVec.emplace_back();
                std::vector<int> &tpI = tpIVec.back();
                std::vector<int> &tnI = tnIVec.back();
                std::vector<int> &fpI = fpIVec.back();
                std::vector<int> &fnI = fnIVec.back();
                getFPTN(tpI, tnI, fpI, fnI, rawData.chromsome, true_ocs);
                tp.emplace_back(tpI.size());
                tn.emplace_back(tnI.size());
                fp.emplace_back(fpI.size());
                fn.emplace_back(fnI.size());
                int &tpVal = tp.back();
                int &tnVal = tn.back();
                int &fpVal = fp.back();
                int &fnVal = fn.back();

                accuracy.emplace_back(static_cast<double>( tpVal + tnVal) / static_cast<double>(tpVal + tnVal + fpVal + fnVal));
                recall.emplace_back(static_cast<double>(tpVal) / gtVec.back().size());
                precision.emplace_back(static_cast<double>( tpVal) / static_cast<double>(tpVal + fpVal));
                if(std::isnan(accuracy.back())) accuracy.back() = 0;
                if(std::isnan(recall.back())) recall.back() = 0;
                if(std::isnan(precision.back())) precision.back() = 0;
            }
        }
    }
    uniqAlgNames = algName;
    uniqObjNames = objName;
    sort(uniqAlgNames.begin(), uniqAlgNames.end() );
    uniqAlgNames.erase(unique(uniqAlgNames.begin(), uniqAlgNames.end() ), uniqAlgNames.end() );
    sort(uniqObjNames.begin(), uniqObjNames.end() );
    uniqObjNames.erase(unique(uniqObjNames.begin(), uniqObjNames.end() ), uniqObjNames.end() );

    update_data();
}
AlgorithmDataProc::AlgorithmDataProc(std::string derived_data_path, std::string static_data_path) {
    derivedCSVDocPtr = std::make_shared<rapidcsv::CSVRReadDoc>(derived_data_path,rapidcsv::LabelParams(0, -1));
    staticCSVDocPtr = std::make_shared<rapidcsv::CSVRReadDoc>(static_data_path,rapidcsv::LabelParams(-1, 0));

    // Derived Data
    algName=derivedCSVDocPtr->GetColumn<std::string>("algName");
    objName=derivedCSVDocPtr->GetColumn<std::string>("objName");
    dpIndex = derivedCSVDocPtr->GetColumn<int>("dpIndex");
    chromosome =  derivedCSVDocPtr->GetColumn<chromosomeT>("chromosome");
    tp = derivedCSVDocPtr->GetColumn<int>("tp");
    tn = derivedCSVDocPtr->GetColumn<int>("tn");
    fp = derivedCSVDocPtr->GetColumn<int>("fp");
    fn = derivedCSVDocPtr->GetColumn<int>("fn");
    accuracy = derivedCSVDocPtr->GetColumn<double>("accuracy");
    precision = derivedCSVDocPtr->GetColumn<double>("precision");
    recall = derivedCSVDocPtr->GetColumn<double>("recall");
    time = derivedCSVDocPtr->GetColumn<double>("time");

    // Static Data
    DatasetType =  staticCSVDocPtr->GetCell<std::string>(0,"DatasetType");
    DatasetPath =  staticCSVDocPtr->GetCell<std::string>(0,"DatasetPath");
    uniqAlgNames =  staticCSVDocPtr->GetRow<std::string>("uniqAlgNames");
    uniqObjNames =  staticCSVDocPtr->GetRow<std::string>("uniqObjNames");
    t_thresh =  staticCSVDocPtr->GetCell<double>(0,"t_thresh");
    r_thresh =  staticCSVDocPtr->GetCell<double>(0,"r_thresh");
}


void AlgorithmDataProc::update_data() {
    derivedCSVDocPtr->clear();
    derivedCSVDocPtr->SetColumnName(0,"algName");
    derivedCSVDocPtr->SetColumnName(1,"objName");
    derivedCSVDocPtr->SetColumnName(2,"dpIndex");
    derivedCSVDocPtr->SetColumnName(3,"chromosome");
    derivedCSVDocPtr->SetColumnName(4,"tp");
    derivedCSVDocPtr->SetColumnName(5,"tn");
    derivedCSVDocPtr->SetColumnName(6,"fp");
    derivedCSVDocPtr->SetColumnName(7,"fn");
    derivedCSVDocPtr->SetColumnName(8,"accuracy");
    derivedCSVDocPtr->SetColumnName(9,"precision");
    derivedCSVDocPtr->SetColumnName(10,"recall");
    derivedCSVDocPtr->SetColumnName(11,"time");

    derivedCSVDocPtr->SetColumn(0, algName);
    derivedCSVDocPtr->SetColumn(1, objName);
    derivedCSVDocPtr->SetColumn(2, dpIndex);
    derivedCSVDocPtr->SetColumn(3, chromosome);
    derivedCSVDocPtr->SetColumn(4, tp);
    derivedCSVDocPtr->SetColumn(5, tn);
    derivedCSVDocPtr->SetColumn(6, fp);
    derivedCSVDocPtr->SetColumn(7, fn);
    derivedCSVDocPtr->SetColumn(8, accuracy);
    derivedCSVDocPtr->SetColumn(9, precision);
    derivedCSVDocPtr->SetColumn(10, recall);
    derivedCSVDocPtr->SetColumn(11, time);

    staticCSVDocPtr->clear();
    staticCSVDocPtr->SetRowName(0,"DatasetType");
    staticCSVDocPtr->SetRowName(1,"DatasetPath");
    staticCSVDocPtr->SetRowName(2,"uniqAlgNames");
    staticCSVDocPtr->SetRowName(3,"uniqObjNames");
    staticCSVDocPtr->SetRowName(4,"t_thresh");
    staticCSVDocPtr->SetRowName(5,"r_thresh");

    staticCSVDocPtr->SetRow(0,std::vector{DatasetType});
    staticCSVDocPtr->SetRow(1,std::vector{DatasetPath});
    staticCSVDocPtr->SetRow(2,uniqAlgNames);
    staticCSVDocPtr->SetRow(3,uniqObjNames);
    staticCSVDocPtr->SetRow(4,std::vector{t_thresh});
    staticCSVDocPtr->SetRow(5,std::vector{r_thresh});
}


void AlgorithmDataProc::save_data(std::string derived_data_filename, std::string static_data_filename) {
    derivedCSVDocPtr->Save(derived_data_filename);
    staticCSVDocPtr->Save(static_data_filename);
}

void AlgorithmDataProc::getFPTN(std::vector<int> &tp, std::vector<int> &tn, std::vector<int> &fp, std::vector<int> &fn,
                                chromosomeT chromosome, chromosomeT correct_ocs) {
    for (int i = 0; i < correct_ocs.size(); i++) {
        if (correct_ocs[i]) {
            if (chromosome[i]) {
                tp.emplace_back(i);
            } else {
                fn.emplace_back(i);
            }
        } else {
            if (chromosome[i]) {
                fp.emplace_back(i);
            } else {
                tn.emplace_back(i);
            }
        }
    }
}

void AlgorithmDataProc::getFPTN(int &tp, int &tn, int &fp, int &fn, chromosomeT chromosome, chromosomeT correct_ocs) {
    {
        for (int i = 0; i < correct_ocs.size(); i++) {
            if (correct_ocs[i]) {
                if (chromosome[i]) {
                    tp++;
                } else {
                    fn++;
                }
            } else {
                if (chromosome[i]) {
                    fp++;
                } else {
                    tn++;
                }
            }
        }
    }
}

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






