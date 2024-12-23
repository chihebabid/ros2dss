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
        string getSCCName(SCC *scc);

        int addPlacesEntrees(string nom_transition, vector<string> liste_places_entrees, vector<int> liste_poids);

        Transition *getTransitionPtr(const string& nom_transition);

        uint32_t getPetriID() const;

        void setPetriID(const uint32_t index);

        string getMarquageName(Marking marquage);

        int addPlacesSorties(string nom_transition, vector<string> liste_places_sorties, vector<int> liste_poids);

        void tirer(Transition &t);

        void setMarquage(Marking *marquage);

        Marking getMarquage();

        Place getPlace(const int index);

        Transition *getTransitionPtr(const int code);

        void addListTransitions(vector<Transition> liste_transitions);

        void addListPlaces(vector<Place> liste_places);

        int getPlacesCount();

        PetriNet()=default;

        virtual ~PetriNet()=default;

        MetaState *getMetaState(Marking marquage);

        void printMetaStateEx(MetaState *ms);

        bool areTransitionsIncluded(const std::set<string> &list_transitions);

        uint32_t getModulesCount() const ;
        void setModulesCount(const uint32_t);


        void setSyncTransitions(const std::vector<std::string>& l_transitions);
        vector<string> getSyncTransitions() const;
    private:
        uint32_t m_petri_id;
        uint32_t m_modules_count {};

        vector<Transition *> getListeTransitionsFranchissables();

        Place *getPlaceAdresse(string placename);

        vector<Place> m_places;
        vector<Transition> ml_transitions;

    };
}

#endif // !defined(AFX_PETRINET_H__91DDDBD6_8669_4EC2_A0F5_E20F2FAFDC37__INCLUDED_)
