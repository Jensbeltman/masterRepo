#include "iostream"
#include "hypothesis_verification/evaluator/collision_checking.hpp"
#include "dataset/scape/ScapeDataset.hpp"
#include "dataset/sileane/SileaneDataset.hpp"
#include "datautil/csv_doc.hpp"

int main(){
    ScapeDatasetPtr  scapeDataset =  std::make_shared<ScapeDataset>("/home/jens/masterData/ScapeDatasetNew/Scape/Full_Dataset","/home/jens/masterData/ScapeDatasetNew/Data from Scape Recognition");
    SileaneDatasetPtr sileaneDataset =  std::make_shared<SileaneDataset>("/home/jens/masterData/Siléane-Dataset");

    std::vector<DatasetPtr> datasets = {scapeDataset,sileaneDataset};

    rapidcsv::CSVDoc csvDoc;
    csvDoc.SetColumnName(0,"datasetName");
    csvDoc.SetColumnName(1,"objName");
    csvDoc.SetColumnName(2,"dpI");
    csvDoc.SetColumnName(3,"nGTs");
    csvDoc.SetColumnName(4,"GTIndex_1");
    csvDoc.SetColumnName(5,"GTIndex_2");
    csvDoc.SetColumnName(6,"penetration");


    int ndps = 0;
    for(auto &ds:datasets)
        for(auto &obj:ds->objects)
            for(auto &dp:obj->data_points)
                ndps++;

    int current_dp = 0;
    int rowIdx = 0;
    for(auto &ds:datasets){
        for(auto &obj:ds->objects){
            auto meshPtr = obj->get_mesh();
            auto &dps = obj->data_points;
            for(int dpi = 0; dpi < dps.size(); dpi++){
                auto &dp = dps[dpi];
                Collisions collisions = get_collisions(dp.gts,meshPtr);
                for(int i = 0;i<collisions.distances.size();i++,rowIdx++){
                    std::pair<int,int> &collPair = collisions.pairs[i];
                    double &penetration = collisions.distances[i];
                    csvDoc.SetCell(csvDoc.GetColumnIdx("datasetName"),rowIdx,ds->name);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("objName"),rowIdx,obj->name);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("dpI"),rowIdx,dpi);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("nGTs"),rowIdx,dp.gts.size());
                    csvDoc.SetCell(csvDoc.GetColumnIdx("GTIndex_1"),rowIdx,collPair.first);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("GTIndex_2"),rowIdx,collPair.second);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("penetration"),rowIdx,penetration);
                }
                std::cout<<"\r"<<current_dp++<<"/"<<ndps<<" - "<<rowIdx<<std::flush;
            }
        }
    }

    csvDoc.Save("/home/jens/masterRepo/test/dataset/gt_coll_data.csv");
}