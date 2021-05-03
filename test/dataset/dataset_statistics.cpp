#include <iostream>
#include "hypothesis_verification/evaluator/collision_checking.hpp"
#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollisionVariants.hpp"
#include "dataset/scape/ScapeDataset.hpp"
#include "dataset/sileane/SileaneDataset.hpp"
#include "datautil/csv_doc.hpp"
#include "datautil/ga_conversions.hpp"

std::set<int>::iterator is_in_cluster(int query, std::vector<std::set<int>> &clusters){
    for(auto &cluster:clusters){
        auto itt = cluster.find(query);
        if(!=cluster.end())
            return true;
    }
    return false;
}

int main(int argc,char*argv[]){
    ScapeDatasetPtr scapeDataset = std::make_shared<ScapeDataset>(
            "/home/jens/masterData/ScapeDatasetNew/Scape/Full_Dataset",
            "/home/jens/masterData/ScapeDatasetNew/Data from Scape Recognition");

    rapidcsv::CSVDoc csvDoc("/home/jens/masterRepo/test/dataset/dataset_statistics.csv",rapidcsv::LabelParams(0,-1));

    GeneticEvaluatorLR geneticEvaluatorLr;
    for(auto & obj:scapeDataset->objects){
        geneticEvaluatorLr.init_object(obj);
        for(auto &dp:obj->data_points){
            geneticEvaluatorLr.init_datapoint(dp);

            std::vector<int> n_collision(dp.ocs.size(),0);
            std::vector<bool> disabled_coll;
            std::vector<std::set<int>> clusters;

            for(int i = 0;i<geneticEvaluatorLr.collisions.pairs.size();i++) {
                auto &pair = geneticEvaluatorLr.collisions.pairs[i];
                auto &penetration = geneticEvaluatorLr.collisions.distances[i];
                n_collision[pair]
                if(!disabled_coll[i]){
                    auto &coll_pair = geneticEvaluatorLr.collisions.pairs[i];
                    if(is_in_cluster(coll_pair.first) {

                    }
                    auto &p =  geneticEvaluatorLr.collisions.pairs[i]);
                    for(auto &cluster:clusters){
                        for(auto idx:cluster){
                            if
                        }
                    }
                }
            }

        }
    }


    return 0;
}