//
// Created by chiheb on 23/12/24.
//

#ifndef MANAGETRANSITIONFUSIONSET_H
#define MANAGETRANSITIONFUSIONSET_H


namespace dss {
    class ManageTransitionFusionSet {
        struct fusion_set_t {
            uint32_t module;
            bool enabled {false};
            bool operator<(const fusion_set_t &o) const {
                return module < o.module;
            }
        };
    public:
        ManageTransitionFusionSet()=default;
        ~ManageTransitionFusionSet()=default;
        ManageTransitionFusionSet(const ManageTransitionFusionSet&)=delete;
        ManageTransitionFusionSet& operator=(const ManageTransitionFusionSet&)=delete;
        void add_fusion_set(const std::string& fusion_set_name,const uint32_t module);
        uint32_t getFusionSetsCount() const;
        bool isFusionEnabled(const std::string &) const;
        std::vector<std::string> getEnabledFusionSets() const;
        void enableFusion(const std::string &,uint32_t);
        void enableSetFusion(const set<string> &,uint32_t);
        void reset();
        bool isFusionSetSyncedOnModule(const std::string &name,uint32_t module) const;
        /*
         * For debugging
         */
        void display();

    private:
        std::map<std::string,std::list<fusion_set_t>> ml_fusion_sets;
    };
}


#endif //MANAGETRANSITIONFUSIONSET_H
