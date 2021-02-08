#ifndef MASTER_CSV_DOC_HPP
#define MASTER_CSV_DOC_HPP

#include "datautil/rapidcsv.h"
#include "filesystem"

namespace rapidcsv {
    class CSVDoc : public Document {
    public:
        explicit CSVDoc(const std::string &pPath = std::string(),
                             const LabelParams &pLabelParams = LabelParams(),
                             const SeparatorParams &pSeparatorParams = SeparatorParams(),
                             const ConverterParams &pConverterParams = ConverterParams()) {
            mPath = pPath;
            mLabelParams = pLabelParams;
            mSeparatorParams = pSeparatorParams;
            mConverterParams = pConverterParams;
        };
        template<typename T>
        void SetRowNameAndValue(size_t pRowIdx, const std::string &pRowName, const std::vector<T> &pRow) {
            SetRow(pRowIdx, pRow);
            SetRowName(pRowIdx, pRowName);
        };

        template<typename T>
        void AppendRowNameAndValue(size_t pRowIdx, const std::string &pRowName, const std::vector<T> &pRow) {
            size_t next_row = GetRowCount();
            SetRow(next_row, pRow);
            SetRowName(next_row, pRowName);
        };


        void clear(){
            mData.clear();
            mColumnNames.clear();
            mRowNames.clear();
        }
    };

    typedef std::shared_ptr<CSVDoc> CSVDocPtr;

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
            // Make sure that file exists if not creates a new file.
            if (!std::filesystem::exists(pPath))
                std::cout<<"File " + pPath + "does not exist the CSVDoc might fail."<<std::endl;
            else
                ReadCsv();
        }
    };
    typedef std::shared_ptr<CSVRReadDoc> CSVRReadDocPtr;

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
/*
            // Make sure that file exists and clears if so.
            if (!mPath.empty()) {
                if (std::filesystem::exists(pPath))
                    std::cout << "File " << pPath << " already exist. Will be overwritten\n";

                std::ofstream newfile(pPath, std::ios_base::out | std::ios_base::trunc);
                newfile.close();
            }*/
        }
    };
    typedef std::shared_ptr<CSVWriteDoc> CSVWriteDocPtr;

}
#endif //MASTER_CSV_DOC_HPP