// PetriNet.h: interface for the PetriNet class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PETRINET_H__91DDDBD6_8669_4EC2_A0F5_E20F2FAFDC37__INCLUDED_)
#define AFX_PETRINET_H__91DDDBD6_8669_4EC2_A0F5_E20F2FAFDC37__INCLUDED_


#include "misc.h"
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000



using std::vector;
using std::string;
namespace dss {
    class Transition;

    class ListMarquage;

    class MetaState;

    class SCC;

    class PetriNet {
    public:
        PetriNet()=default;
        virtual ~PetriNet()=default;

        string getSCCName(const SCC *scc) const;

        int addPlacesEntrees(string nom_transition, vector<string> liste_places_entrees, vector<int> liste_poids);

        uint32_t getPetriID() const;

        void setPetriID(const uint32_t index);

        string getMarquageName(Marking marquage) const;

        int addPlacesSorties(string nom_transition, vector<string> liste_places_sorties, vector<int> liste_poids);

        void fire(Transition &t);

        void setMarquage(const Marking & marquage);

        Marking getMarquage();

        Place getPlace(const int index) const;

        Transition *getTransitionPtr(const int code);
        Transition *getTransitionPtr(const string &name);
        void addListTransitions(vector<Transition> liste_transitions);

        void addListPlaces(vector<Place> liste_places);

        size_t getPlacesCount() const;

        MetaState *getMetaState(Marking marquage);
        void printMetaStateEx(MetaState *ms);

        uint32_t getModulesCount() const ;
        void setModulesCount(const uint32_t);
        void setSyncTransitions(const std::vector<std::string>& l_transitions);
        vector<string> getSyncTransitions() const;
        set<string> getSyncEnabled(const MetaState *);
        std::shared_ptr<ManageTransitionFusionSet> getManageTransitionFusionSet() const;
        std::set<FiringSyncTransition> fireSync(const string &, const MetaState *);
    private:
        uint32_t m_petri_id;
        uint32_t m_modules_count {};

        vector<Transition *> getListeTransitionsFranchissables();

        Place *getPlaceAdresse(string placename);

        vector<Place> m_places;
        vector<Transition> ml_transitions;
        std::shared_ptr<ManageTransitionFusionSet> m_manage_transition_fusion_set {std::make_shared<ManageTransitionFusionSet>()};


    };
}

#endif // !defined(AFX_PETRINET_H__91DDDBD6_8669_4EC2_A0F5_E20F2FAFDC37__INCLUDED_)
