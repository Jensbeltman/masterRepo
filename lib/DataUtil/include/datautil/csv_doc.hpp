#ifndef MASTER_CSV_DOC_HPP
#define MASTER_CSV_DOC_HPP

#include "datautil/rapidcsv.h"
#include "filesystem"

namespace rapidcsv {
    class CSVDoc : public Document {
    public:
        template<typename T>
        void SetRowNameAndValue(size_t pRowIdx, const std::string &pRowName, const std::vector<T> &pRow) {
            SetRow(pRowIdx, pRow);
            SetRowName(pRowIdx, pRowName);
        };
    };

    class CSVRReadDoc : public CSVDoc {
    public:
        explicit CSVRReadDoc(const std::string &pPath = std::string(),
                             const LabelParams &pLabelParams = LabelParams(),
                             const SeparatorParams &pSeparatorParams = SeparatorParams(),
                             const ConverterParams &pConverterParams = ConverterParams()) {
            mPath = pPath;
            mLabelParams = pLabelParams;
            mSeparatorParams = pSeparatorParams;
            mConverterParams = pConverterParams;
            // Make sure that file exists and clears if so.
            if (!std::filesystem::exists(pPath)) {
                std::ofstream newfile(pPath, std::ios_base::out);
                newfile.close();
            }
            ReadCsv();
        }
    };

    class CSVWriteDoc : public CSVDoc {
    public:
        explicit CSVWriteDoc(const std::string &pPath = std::string(),
                             const LabelParams &pLabelParams = LabelParams(),
                             const SeparatorParams &pSeparatorParams = SeparatorParams(),
                             const ConverterParams &pConverterParams = ConverterParams()) {
            mPath = pPath;
            mLabelParams = pLabelParams;
            mSeparatorParams = pSeparatorParams;
            mConverterParams = pConverterParams;

            // Make sure that file exists and clears if so.
            if (!std::filesystem::exists(pPath))
                std::cout << "File " << pPath << " already exist. Will be overwritten\n";

            std::ofstream newfile(pPath, std::ios_base::out | std::ios_base::trunc);
            newfile.close();
        }
    };
}
#endif //MASTER_CSV_DOC_HPP