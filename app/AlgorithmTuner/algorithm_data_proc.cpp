//
// Created by jens on 1/22/21.
//

#include "algorithm_data_proc.hpp"
#include "datautil/ga_conversions.hpp"
#include "datautil/csv_doc.hpp"


AlgorithmDataProc::AlgorithmDataProc(): rawDataMapAlgObjVec(rawDataMapAlgObjVecT()) {
}

AlgorithmDataProc::AlgorithmDataProc(rawDataMapAlgObjVecT newrawDataMapAlgObjVec, double t_thresh, double r_thresh)
        : rawDataMapAlgObjVec(newrawDataMapAlgObjVec) {
    generate_data(t_thresh, r_thresh);

}


void AlgorithmDataProc::generate_data(double t_thresh, double r_thresh) {
    size_t ndps = 0;
    for (auto &alg_pair:rawDataMapAlgObjVec) {
        for (auto &obj_pair:alg_pair.second) {
            ndps+=obj_pair.second.size();
        }
    }
    algNameVec.clear();
    algNameVec.reserve(ndps);
    objNameVec.clear();
    objNameVec.reserve(ndps);
    nOCVec.clear();
    nOCVec.reserve(ndps);
    chromosomeVec.clear();
    chromosomeVec.reserve(ndps);
    timeVec.clear();
    timeVec.reserve(ndps);
    tpVec.clear();
    tpVec.reserve(ndps);
    tnVec.clear();
    tnVec.reserve(ndps);
    fpVec.clear();
    fpVec.reserve(ndps);
    fnVec.clear();
    fnVec.reserve(ndps);
    accuracyVec.clear();
    accuracyVec.reserve(ndps);
    precisionVec.clear();
    precisionVec.reserve(ndps);
    recallVec.clear();
    recallVec.reserve(ndps);
    algNameUniqueVec.clear();
    algNameUniqueVec.reserve(ndps);
    objNameUniqueVec.clear();
    objNameUniqueVec.reserve(ndps);

    for (auto &alg_pair:rawDataMapAlgObjVec) {
        for (auto &obj_pair:alg_pair.second) {
            for (auto &rawData:obj_pair.second) {
                std::vector<bool> true_ocs = tu.get_true_ocs(rawData.dp.ocs, rawData.dp.gts, t_thresh, r_thresh,obj_pair.first->symmetry_transforms);
                algNameVec.emplace_back(alg_pair.first);
                objNameVec.emplace_back(obj_pair.first->name);

                nOCVec.emplace_back(rawData.dp.ocs.size());
                chromosomeVec.emplace_back(rawData.chromsome);
                ocVec.emplace_back(rawData.dp.ocs);
                gtVec.emplace_back(rawData.dp.gts);
                trueOCVec.emplace_back(true_ocs);
                timeVec.emplace_back(rawData.time);

                tpIVec.emplace_back();
                tnIVec.emplace_back();
                fpIVec.emplace_back();
                fnIVec.emplace_back();
                std::vector<int> &tpI = tpIVec.back();
                std::vector<int> &tnI = tnIVec.back();
                std::vector<int> &fpI = fpIVec.back();
                std::vector<int> &fnI = fnIVec.back();
                getFPTN(tpI, tnI, fpI, fnI, rawData.chromsome, true_ocs);
                tpVec.emplace_back(tpI.size());
                tnVec.emplace_back(tnI.size());
                fpVec.emplace_back(fpI.size());
                fnVec.emplace_back(fnI.size());
                int &tp = tpVec.back();
                int &tn = tnVec.back();
                int &fp = fpVec.back();
                int &fn = fnVec.back();

                accuracyVec.emplace_back(static_cast<double>( tp + tn) / static_cast<double>(tp + tn + fp + fn));
                recallVec.emplace_back(static_cast<double>(tp) /gtVec.back().size());
                precisionVec.emplace_back(static_cast<double>( tp) / static_cast<double>(tp + fp));
                if(std::isnan(accuracyVec.back())) accuracyVec.back() = 0;
                if(std::isnan(recallVec.back())) recallVec.back() = 0;
                if(std::isnan(precisionVec.back())) precisionVec.back() = 0;
            }
        }
    }
    algNameUniqueVec = algNameVec;
    objNameUniqueVec = objNameVec;
    sort( algNameUniqueVec.begin(), algNameUniqueVec.end() );
    algNameUniqueVec.erase( unique( algNameUniqueVec.begin(), algNameUniqueVec.end() ), algNameUniqueVec.end() );
    sort( objNameUniqueVec.begin(), objNameUniqueVec.end() );
    objNameUniqueVec.erase( unique( objNameUniqueVec.begin(), objNameUniqueVec.end() ), objNameUniqueVec.end() );
}


void AlgorithmDataProc::save_data(std::string filename) {
    rapidcsv::CSVWriteDoc csvWriteDoc(filename,
                                      rapidcsv::LabelParams(0, -1));
    csvWriteDoc.SetColumnName(0,"AlgorithmName");
    csvWriteDoc.SetColumnName(1,"ObjectName");
    csvWriteDoc.SetColumnName(2,"tp");
    csvWriteDoc.SetColumnName(3,"tn");
    csvWriteDoc.SetColumnName(4,"fp");
    csvWriteDoc.SetColumnName(5,"nOC");
    csvWriteDoc.SetColumnName(5,"fn");
    csvWriteDoc.SetColumnName(6,"Accuracy");
    csvWriteDoc.SetColumnName(8,"Precision");
    csvWriteDoc.SetColumnName(9,"Recall");
    csvWriteDoc.SetColumnName(10,"Time");
    csvWriteDoc.SetColumnName(11,"chromosomeVec");
    csvWriteDoc.SetColumnName(12,"trueOCVec");

    csvWriteDoc.SetColumn(0,algNameVec);
    csvWriteDoc.SetColumn(1,objNameVec);
    csvWriteDoc.SetColumn(2,tpVec);
    csvWriteDoc.SetColumn(3,tnVec);
    csvWriteDoc.SetColumn(4,fpVec);
    csvWriteDoc.SetColumn(5,nOCVec);
    csvWriteDoc.SetColumn(6,fnVec);
    csvWriteDoc.SetColumn(7,accuracyVec);
    csvWriteDoc.SetColumn(8,precisionVec);
    csvWriteDoc.SetColumn(9,recallVec);
    csvWriteDoc.SetColumn(10,timeVec);
    csvWriteDoc.SetColumn(11,chromosomeVec);
    csvWriteDoc.SetColumn(12,trueOCVec);
    csvWriteDoc.Save();
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

matplot::figure_handle  AlgorithmDataProc::bar_plot(std::vector<int> &valVec) {
    std::vector<double> vec = VecIToVecD(valVec);
    return bar_plot(vec);
}

matplot::figure_handle  AlgorithmDataProc::bar_plot(std::vector<double> &valVec) {
    using namespace matplot;
    auto f = figure(true);
    auto ax = f->current_axes();

    std::vector<std::vector<double>> y;
    for(auto &alg:algNameUniqueVec){
        y.emplace_back();
        for(auto &obj:objNameUniqueVec){
            y.back().emplace_back(get_avr(valVec,alg,obj));
        }
    }

    auto bar_handle = ax->bar(y);
    std::string xl = "Object Type(";
    for(auto &alg:algNameUniqueVec)
        xl+=alg+", ";
    xl = xl.substr(0,xl.npos-2)+")";
    ax->xlabel(xl);
    ax->x_axis().ticklabels(objNameUniqueVec);

    std::vector<std::string> legends;
    for(auto &alg:algNameUniqueVec)
        legends.emplace_back(alg + " obj avr");
    auto legend_handle = legend(ax);

    f->draw();
    return f;

}

bool AlgorithmDataProc::get_begin_end_index(size_t &bi, size_t &ei, std::string alg_key, std::string obj_key) {
    {
        if(obj_key!=""){
            bi = std::max(begin_index(alg_key, algNameVec), begin_index(obj_key, objNameVec));
            ei = std::min(end_index(alg_key, algNameVec), end_index(obj_key, objNameVec));
        }else{
            bi = begin_index(alg_key, algNameVec);
            ei = end_index(alg_key, algNameVec);
        }
        return( bi!=ei &&  bi<algNameVec.size());
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





