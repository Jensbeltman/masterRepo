#ifndef MASTER_LOGGING_HPP
#define MASTER_LOGGING_HPP
#include <ga/ga.hpp>
#include <fstream>


void result_write(GAResult &result, std::string output_path= "./ga_results.json") {
    nlohmann::json j = result;
    std::ofstream out_file(output_path);
    if (!out_file.is_open())
        std::cout << "Json file "<< output_path << " not opened" << std::endl;
    out_file << j.dump();
    out_file.close();
}

#endif //MASTER_LOGGING_HPP
