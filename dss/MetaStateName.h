//
// Created by chiheb on 18/12/24.
//

#ifndef METASTATENAME_H
#define METASTATENAME_H


namespace dss {
    class MetaStateName {
    public:
        MetaStateName()=default;
        void setSCCName(const std::string &,const int);
        std::string& getSCCName(const int);
    private:
        std::vector<std::string> m_name;
        SCC* m_scc;
    };
}


#endif //METASTATENAME_H
