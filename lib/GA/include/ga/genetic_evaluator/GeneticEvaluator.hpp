#ifndef MASTER_GENETICEVALUATOR_HPP
#define MASTER_GENETICEVALUATOR_HPP

#include "ga/typedefinitions.hpp"
#include "dataset/Dataset.hpp"

class GeneticEvaluator {
public:
    GeneticEvaluator();

    std::string type = "GeneticEvaluator";

    DataPoint dp;
    DatasetObjectPtr datasetObjectPtr;

    PointCloudT::Ptr pc;//point cloud data
    PointCloudT::Ptr pcm;//mesh point cloud

    virtual double evaluate_chromosome(chromosomeT &chromosome) = 0;
    virtual void initialise_object(DatasetObjectPtr &datasetObjectPtr, int datapoint_n = 0)=0;
    virtual void initialise_object(DatasetObjectPtr &datasetObjectPtr, DataPoint &datapoint_n)=0;

    virtual void initialise_datapoint(int datapoint_n)=0;
    virtual void initialise_datapoint(DataPoint &datapoint_n)=0;
};
typedef std::shared_ptr<GeneticEvaluator> GeneticEvaluatorPtr;


#endif //MASTER_GENETICEVALUATOR_HPP
