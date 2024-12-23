//
// Created by chiheb on 23/12/24.
//

#ifndef MANAGETRANSITIONFUSIONSET_H
#define MANAGETRANSITIONFUSIONSET_H


namespace dss {
    class ManageTransitionFusionSet {
        struct fusion_set_t {
            std::set<uint32_t> set_modules;
        };
    public:
        ManageTransitionFusionSet()=default;
        ~ManageTransitionFusionSet()=default;
        void add_fusion_set(const std::string& fusion_set_name,const uint32_t module);

    private:
        std::map<std::string,std::set<uint32_t>> ml_fusion_sets;
    };
}


#endif //MANAGETRANSITIONFUSIONSET_H
