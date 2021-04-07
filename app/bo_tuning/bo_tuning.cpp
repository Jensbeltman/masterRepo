#include "iostream"
#include "sstream"
#include "bayesopt/bayesopt.hpp"
#include "chronometer.h"
#include "dataset/scape/ScapeDataset.hpp"
#include "hypothesis_verification/hv_alg/ga.hpp"
#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollisionVariants.hpp"
#include "dataset/transform_utility.hpp"

using namespace bayesopt;

DatasetPtr load_dataset(bool non_maxima_supression = true, double t_thresh = 0.5, double r_thresh = 0.5) {
    ScapeDatasetPtr scapeDataPtr = std::make_shared<ScapeDataset>(
            "/home/jens/masterData/ScapeDatasetNew/Scape/Full_Dataset",
            "/home/jens/masterData/ScapeDatasetNew/Data from Scape Recognition", false);
    // Non maxima supression on dataset.
    if (non_maxima_supression) {
        for (auto &obj:scapeDataPtr->objects)
            for (auto &dp:obj->data_points)
                tu::non_maximum_supression(dp, t_thresh, r_thresh, obj->symmetry_transforms);
    }
    return scapeDataPtr;
}

void initializeGEs(DatasetPtr dataPtr, std::string object_name, std::vector<GeneticEvaluatorLRPtr> &dpGEs,
                   double t_thresh = 5, double r_thresh = 5, double vg_leaf_size = 3,
                   double nn_inlier_threshold = 2.5) {
    // Load dataset
    GeneticEvaluatorLRPtr geneticEvaluatorLRPtrBase = std::make_shared<GeneticEvaluatorLR>();
    geneticEvaluatorLRPtrBase->vg_leaf_size = 3;
    geneticEvaluatorLRPtrBase->nn_inlier_threshold = 2.5;

    std::vector<GeneticEvaluatorLRPtr> objGEs;
    for (auto &obj:dataPtr->objects) {
        if (obj->name == object_name) {
            GeneticEvaluatorLRPtr geneticEvaluatorLRPtr = std::make_shared<GeneticEvaluatorLR>(
                    *geneticEvaluatorLRPtrBase);
            geneticEvaluatorLRPtr->init_object(obj);
            objGEs.emplace_back(geneticEvaluatorLRPtr);
        }
    }

    for (auto &ge : objGEs) {
        for (auto &dp: ge->datasetObjectPtr->data_points) {
            if (dp.gts.size() > 1) {
                GeneticEvaluatorLRPtr geneticEvaluatorLRPtr = std::make_shared<GeneticEvaluatorLR>(*ge);
                geneticEvaluatorLRPtr->init_datapoint(dp);
                dpGEs.emplace_back(geneticEvaluatorLRPtr);
            }
        }
    }
}

class GALROptimization : public ContinuousModel {
public:
    Chronometer chronometer;
    std::vector<GeneticEvaluatorLRPtr> dpGEs;
    GA ga;
    double t_thresh, r_thresh;

    GALROptimization(std::vector<GeneticEvaluatorLRPtr> &dpGEs, size_t input_dimension, bopt_params param,
                     double t_thresh = 5, double r_thresh = 5) :
            ContinuousModel(input_dimension, param),
            t_thresh(t_thresh),
            r_thresh(r_thresh),
            dpGEs(dpGEs) {
        chronometer.tic();// Intialize timer
    }

    double evaluateSample(const boost::numeric::ublas::vector<double> &query) {
        int tp = 0, tn = 0, fp = 0, fn = 0;
        for (auto &dpGE: dpGEs) {
            // Set parameters
            dpGE->score_w = query[0];
            dpGE->visiblePointsFrac_w = query[1];
            dpGE->visibleInlierFrac_w = query[2];
            dpGE->penetration_w = query[3];
            dpGE->intersectingInliersFrac_w = query[4];
            dpGE->intercept = query[5];

            ga.geneticEvaluatorPtr = dpGE;
            ga.n_genes = dpGE->dp.ocs.size();
            chronometer.tic();
            HVResult hvResult = ga.solve();
            std::vector<int> correct_oc_idcs;
            tu::find_correct_ocs(dpGE->dp.ocs, dpGE->dp.gts, t_thresh, r_thresh, correct_oc_idcs,
                                 dpGE->datasetObjectPtr->symmetry_transforms);
            tu::getFPTN(tp, tn, fp, fn, hvResult.chromosome, correct_oc_idcs);
        }
        double f1 = (2.0 * tp) / (2.0 * tp + fp + fn);
        // std::cout << "Time: " << chronometer.toc() << std::endl;
        return 1 - f1;
    };
};


bool is_number(const std::string s) {
    long double ld;
    return ((std::istringstream(s) >> ld >> std::ws).eof());
}


void seperate_by(std::string s, std::string delim, std::vector<std::string> &svec) {
    size_t pos = 0;
    while ((pos = s.find(delim)) != std::string::npos) {
        svec.emplace_back(s.substr(0, pos));
        s.erase(0, pos + delim.length());
    }
    svec.emplace_back(s);
}

void get_train_validation_kfold(int k, std::vector<GeneticEvaluatorLRPtr> &train_data,
                                std::vector<GeneticEvaluatorLRPtr> &val_data,
                                std::vector<std::pair<std::vector<GeneticEvaluatorLRPtr>::iterator, std::vector<GeneticEvaluatorLRPtr>::iterator>> &folds) {
    int n_folds = folds.size();
    if(n_folds==1){
        val_data.insert(val_data.end(), folds[0].first, folds[0].second);
        train_data.insert(train_data.end(), folds[0].first, folds[0].second);
    }
    else{
        for (int i = 0; i < n_folds; i++) {
            if (i == k) {
                val_data.insert(val_data.end(), folds[i].first, folds[i].second);
            } else {
                train_data.insert(train_data.end(), folds[i].first, folds[i].second);
            }
        }
    }
}

template<typename T>
double average(std::vector<T> values) {
    return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}


int main(int argc, char *argv[]) {

    std::cout << argc << std::endl;
    std::map<std::string, double> nmap = {{"individual",          1.0},
                                          {"k_fold",              5},
                                          {"t_thresh",            5},
                                          {"r_thresh",            5},
                                          {"vg_leaf_size",        3},
                                          {"nn_inlier_threshold", 2.5},
                                          {"n_iterations",        150},
                                          {"n_iter_relearn",      5}};

    std::string object_names_str = "AngleTubes:Ears:Conrods:Gameboys";
    std::vector<std::string> object_names;
    if (argc > 1) {
        for (int i = 1; i < argc; i++) {
            std::cout << argv[i] << std::endl;
            auto itt = nmap.find(argv[i]);
            if (itt != nmap.end()) {
                if (++i < argc) {
                    if (is_number(argv[i])) {
                        itt->second = std::stod(argv[i]);
                    }
                }
            } else if (argv[i] == "object_names") {
                if (++i < argc)
                    object_names_str = argv[i];
            }
        }
    }

    bopt_params bopt_params = initialize_parameters_to_default();
    bopt_params.l_type = L_MCMC;
    bopt_params.n_iterations = nmap["n_iterations"];
    bopt_params.n_iter_relearn = nmap["n_iter_relearn"];
    bopt_params.load_save_flag = 2;
    size_t dim = 6;
    std::vector<std::string> dim_names = {"score_w", "visiblePointsFrac_w", "visibleInlierFrac_w", "penetration_w",
                                          "intersectingInliersFrac_w", "intercept"};

    seperate_by(object_names_str, ":", object_names);

    DatasetPtr datasetPtr = load_dataset(true, nmap["t_thresh"], nmap["r_thresh"]);

    //Define bounds and prepare result.
    boost::numeric::ublas::vector<double> bestPoint(dim);
    boost::numeric::ublas::vector<double> lowerBound(dim);
    lowerBound[0] = 0;
    lowerBound[1] = 0;
    lowerBound[2] = 0;
    lowerBound[3] = -10;
    lowerBound[4] = -30;
    lowerBound[5] = -20;
    boost::numeric::ublas::vector<double> upperBound(dim);
    upperBound[0] = 1;
    upperBound[1] = 20;
    upperBound[2] = 15;
    upperBound[3] = 0;
    upperBound[4] = 0;
    upperBound[5] = 0;


    std::map<std::string, std::vector<GeneticEvaluatorLRPtr>> dpGEs_map;
    std::map<std::string, std::vector<std::pair<std::vector<GeneticEvaluatorLRPtr>::iterator, std::vector<GeneticEvaluatorLRPtr>::iterator>>> dpGEs_kfold_map;
    int k_fold = static_cast<int>(nmap["k_fold"]);
    for (auto &name : object_names) {
        dpGEs_map[name] = std::vector<GeneticEvaluatorLRPtr>();
        initializeGEs(datasetPtr, name, dpGEs_map[name], nmap["t_thresh"], nmap["r_thresh"], nmap["vg_leaf_size"],
                      nmap["nn_inlier_threshold"]);


        int n_dpGEs = dpGEs_map[name].size();
        int step = std::ceil(n_dpGEs / nmap["k_fold"]);
        std::vector<GeneticEvaluatorLRPtr>::iterator begin_itt, end_itt;
        for (int i = 0; i < k_fold; i++) {
            begin_itt = dpGEs_map[name].begin() + i * step;
            end_itt = ((i + 1) * step) < n_dpGEs ? dpGEs_map[name].begin() + (i + 1) * step : dpGEs_map[name].end();
            dpGEs_kfold_map[name].emplace_back(std::make_pair(begin_itt, end_itt));
        }
    }

    if (int(nmap["individual"])) {
        std::cout << "Optimizing Objects " << object_names_str << " individualy" << std::endl;

        std::map<std::string, std::vector<std::shared_ptr<GALROptimization>>> objOptimizationsTrain;
        std::map<std::string, std::vector<double>> objOptimizationValidations;

        for (auto &name : object_names) {
            std::vector<GeneticEvaluatorLRPtr> train, val;
            for (int k = 0; k < k_fold; k++) {
                get_train_validation_kfold(k, train, val, dpGEs_kfold_map[name]);

                std::string fn = ("/home/jens/masterRepo/test/tuning/bayesopt_" + name + ".dat");
                bopt_params.load_filename = &fn[0];
                objOptimizationsTrain[name].emplace_back(std::make_shared<GALROptimization>(train, dim, bopt_params,
                                                                                            nmap["t_thresh"],
                                                                                            nmap["r_thresh"]));
                objOptimizationsTrain[name].back()->setBoundingBox(lowerBound, upperBound);
                objOptimizationsTrain[name].back()->optimize(bestPoint);

                auto val_optim = make_shared<GALROptimization>(val, dim, bopt_params, nmap["t_thresh"],
                                                               nmap["r_thresh"]);
                objOptimizationValidations[name].emplace_back(
                        val_optim->evaluateSample(objOptimizationsTrain[name].back()->getFinalResult()));
            }
        }
        for (int i = 0; i < object_names.size(); i++) {
            auto &name = object_names[i];
            auto &train_optimizations = objOptimizationsTrain[name];
            std::vector<double> &valueAtMinimumValidation = objOptimizationValidations[name];
            std::vector<double> valueAtMinimumTrain;


            int val_i = 0;
            for (int k = 0; k < k_fold; k++) {
                auto &optimization = train_optimizations[k];
                valueAtMinimumTrain.emplace_back(optimization->getValueAtMinimum());
                std::cout << name << std::setw(32) << std::left << " Train Value At minimum "
                          << valueAtMinimumTrain.back()
                          << std::endl;
                std::cout << name << std::setw(32) << std::left << " Validation Value At minimum "
                          << valueAtMinimumValidation[k]
                          << std::endl;

            }
            std::cout << name << std::setw(40) << std::left << " Train Value At minimum"<<", min:"
                      << std::setprecision(3) << *std::min_element(valueAtMinimumTrain.begin(), valueAtMinimumTrain.end()) << ", avr:"
                      << std::setprecision(3) << average(valueAtMinimumTrain)<<", max:"
                      << std::setprecision(3) << *std::max_element(valueAtMinimumTrain.begin(), valueAtMinimumTrain.end())
                      << std::endl;
            std::cout << name << std::setw(40) << std::left << " Validation Value At minimum"<<", min:"
                      << std::setprecision(3) << *std::min_element(valueAtMinimumValidation.begin(), valueAtMinimumValidation.end()) << ", avr:"
                      << std::setprecision(3) << average(valueAtMinimumValidation)<<", max:"
                      << std::setprecision(3) << *std::max_element(valueAtMinimumValidation.begin(), valueAtMinimumValidation.end())
                      << std::endl;


            int besti = std::distance(valueAtMinimumValidation.begin(),
                                      std::min_element(valueAtMinimumValidation.begin(), valueAtMinimumValidation.end()));
            for (int i = 0; i < dim; i++) {
                std::cout << name << "," << "GELR" << "," << dim_names[i] << ","
                          << train_optimizations[besti]->getFinalResult()[i] << std::endl;
            }
        }
    } else {
        std::cout << "Optimizing Objects " << object_names_str << " together" << std::endl;
        std::vector<std::shared_ptr<GALROptimization>> objOptimizationsTrain;
        std::vector<double> objOptimizationValidations;

        for (int k = 0; k < k_fold; k++) {
            std::vector<GeneticEvaluatorLRPtr> train, val;
            for (auto &name : object_names) {
                get_train_validation_kfold(k, train, val, dpGEs_kfold_map[name]);
            }

            objOptimizationsTrain.emplace_back(std::make_shared<GALROptimization>(train, dim, bopt_params, nmap["t_thresh"], nmap["r_thresh"]));
            objOptimizationsTrain.back()->setBoundingBox(lowerBound, upperBound);
            objOptimizationsTrain.back()->optimize(bestPoint);
            objOptimizationValidations.emplace_back(std::make_shared<GALROptimization>(val, dim, bopt_params, nmap["t_thresh"], nmap["r_thresh"])->evaluateSample(objOptimizationsTrain.back()->getFinalResult()));
        }

        std::vector<double> &valueAtMinimumValidation = objOptimizationValidations;
        std::vector<double> valueAtMinimumTrain;

        for (int k = 0; k < k_fold; k++) {
            auto &optim = objOptimizationsTrain[k];
            valueAtMinimumTrain.emplace_back(optim->getValueAtMinimum());
            std::cout << std::setw(32) << std::left << " Train Value At minimum "
                      << valueAtMinimumTrain.back()
                      << std::endl;
            std::cout << std::setw(32) << std::left << " Validation Value At minimum "
                      << valueAtMinimumValidation[k]
                      << std::endl;
        }

        std::cout << std::setw(40) << std::left << " Train Value At minimum"<<", min:"
                  << std::setprecision(3) << *std::min_element(valueAtMinimumTrain.begin(), valueAtMinimumTrain.end()) << ", avr:"
                  << std::setprecision(3) << average(valueAtMinimumTrain)<<", max:"
                  << std::setprecision(3) << *std::max_element(valueAtMinimumTrain.begin(), valueAtMinimumTrain.end())
                  << std::endl;
        std::cout << std::setw(40) << std::left << " Validation Value At minimum"<<", min:"
                  << std::setprecision(3) << *std::min_element(valueAtMinimumValidation.begin(), valueAtMinimumValidation.end()) << ", avr:"
                  << std::setprecision(3) << average(valueAtMinimumValidation)<<", max:"
                  << std::setprecision(3) << *std::max_element(valueAtMinimumValidation.begin(), valueAtMinimumValidation.end())
                  << std::endl;


        int besti = std::distance(valueAtMinimumValidation.begin(),std::min_element(valueAtMinimumValidation.begin(), valueAtMinimumValidation.end()));

        for (auto &name:object_names) {
            GALROptimization objOptim(dpGEs_map[name], dim, bopt_params, nmap["t_thresh"], nmap["r_thresh"]);
            double obj_value_at_min = objOptim.evaluateSample(objOptimizationsTrain[besti]->getFinalResult());
            std::cout<< name <<" Value At minimum: "<<obj_value_at_min<<std::endl;
            for (int i = 0; i < dim; i++) {
                std::cout << name << "," << "GELR" << "," << dim_names[i] << ","
                          << objOptimizationsTrain[besti]->getFinalResult()[i] << std::endl;
            }
        }
    }

    return 0;
}
