// PetriNet.cpp: implementation of the PetriNet class.
//
//////////////////////////////////////////////////////////////////////
#include "misc.h"

namespace dss {
    typedef vector<element_t> Pile;
    typedef vector<PElement> PStack;


    /*
     * Return number of places
     */
    size_t PetriNet::getPlacesCount() const {
        return m_places.size();
    }

    void PetriNet::addListPlaces(vector<Place> liste_places) {
        for (size_t i {}; i < liste_places.size(); i++)
            m_places.push_back(liste_places[i]);
    }

    void PetriNet::addListTransitions(vector<Transition> liste_transitions) {
        for (unsigned int i = 0; i < liste_transitions.size(); i++) {
            liste_transitions[i].setPetri(this->getPetriID());
            ml_transitions.push_back(liste_transitions[i]);
        }
    }


    ////////////////////////////////////////////
    //  Retourner l'a&dresse d'une place
    ////////////////////////////////////////////
    Place *PetriNet::getPlaceAdresse(string placename) {
        int indice = -1;
        for (size_t i {}; i < m_places.size() && indice == -1; i++)
            if (placename == m_places[i].getName()) indice = i;
        if (indice == -1) printf("Place not found!\n");
        return &m_places[indice];
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Retourner un pointeur sur une transition ayant le code pass� en param�tre //
    ///////////////////////////////////////////////////////////////////////////////
    Transition *PetriNet::getTransitionPtr(const int code) {
        int indice = -1;
        for (size_t i {}; i < ml_transitions.size() && indice == -1; i++) {
            if (code == ml_transitions[i].getCode()) indice = i;
        }
        return &ml_transitions[indice];
    }

    Place PetriNet::getPlace(const int index) const {
        return m_places[index];
    }

    ////////////////////////////////
    // Renvoyer le marquage courant
    ////////////////////////////////
    Marking PetriNet::getMarquage() {
        Marking vecteur{m_places.size()};
        for (size_t i {}; i < m_places.size(); i++) {
            vecteur.add8BitsValue(m_places[i].getTokens(), i);
        }

        return vecteur;
    }

    void PetriNet::setMarquage(const Marking &marquage) {
        for (size_t i {}; i < m_places.size(); i++) {
            m_places[i].setTokens(marquage.get8BitsValue(i));
        }
    }

    /*
     * Return enabled transitions
     */
    vector<Transition *> PetriNet::getListeTransitionsFranchissables() {
        vector<Transition *> liste_transitions;
        for (size_t i {}; i < ml_transitions.size(); i++)
            if (ml_transitions[i].isFranchissable()) {
                liste_transitions.push_back(&ml_transitions.at(i));
            }
        return liste_transitions;
    }

    void PetriNet::fire(Transition &t) {
        Transition *transition = getTransitionPtr(t.getCode());
        if (transition->isFranchissable()) transition->fire();
    }


    int PetriNet::addPlacesEntrees(string nom_transition, vector<string> liste_places_entrees,
                                   vector<int> liste_poids) {
        // Localisation de l'indice de la transition
        int indice = -1;
        for (size_t i {}; i < ml_transitions.size() && indice == -1; i++) {
            if (ml_transitions[i].getName() == nom_transition)
                indice = i;
        }
        if (indice != -1) {
            for (size_t i {}; i < liste_places_entrees.size(); i++) {
                ml_transitions[indice].addPlaceEntree(getPlaceAdresse(liste_places_entrees[i]), liste_poids[i]);
                //printf("%s, ",liste_places_entrees[i].mot.c_str());
            }
            return 0;
        }
        cout << "\nErreur d'ajout d'une relation!\n";
        return -1;
    }

    ///////////////////////////////////////////////////////////
    // Ajouter les places d'entr�es � une transition
    ///////////////////////////////////////////////////////////
    int PetriNet::addPlacesSorties(string nom_transition, vector<string> liste_places_sorties,
                                   vector<int> liste_poids) {
        // Localisation de l'indice de la transition
        int indice = -1;
        for (size_t i {}; i < ml_transitions.size() && indice == -1; i++) {
            if (ml_transitions[i].getName() == nom_transition)
                indice = i;
        }
        if (indice != -1) {
            // Ajout de places de sortie
            for (size_t j {}; j < liste_places_sorties.size(); ++j) {
                ml_transitions[indice].addPlaceSortie(getPlaceAdresse(liste_places_sorties[j]), liste_poids[j]);
                //printf("%s, ",liste_places_sorties[j].mot.c_str());
            }
            return 0;
        }
        printf("ERROR : bad relation!\n");
        return -1;
    }

    /////////////////////////////////////
    /// Renvoyer le nom d'un marquage ///
    /////////////////////////////////////
    string PetriNet::getMarquageName(Marking marquage) const {
        string resultat = "(";
        char chaine[100] = "\0";
        for (size_t j {}; j < getPlacesCount(); ++j) {
            if (marquage.get8BitsValue(j) != 0) {
                if (marquage.get8BitsValue(j) != 1) {
                    sprintf(chaine, "%d", (int) marquage.get8BitsValue(j));
                    resultat += chaine;
                }
                resultat += getPlace(j).getName();
                resultat += ".";
            }
        }
        if (resultat.length()>1) resultat[resultat.length()-1]= ')';
        else resultat+=")";
        return resultat;
    }


    void PetriNet::setPetriID(const uint32_t index) {
        m_petri_id = index;
    }

    /*
     * @brief return id of module
     */
    uint32_t PetriNet::getPetriID() const {
        return m_petri_id;
    }


    /*
     * Build a metastat from a marking
     */

    MetaState *PetriNet::getMetaState(Marking marquage) {
        //vector<Marquage> list_marq_inserted;
        PStack pstack;
        PElement elt;
        auto *ms = new MetaState(getModulesCount());
        elt.marquage = new Marking(marquage);
        setMarquage(marquage);
        elt.liste_transitions = getListeTransitionsFranchissables();
        pstack.push_back(elt);

        ms->insertMarking(elt.marquage);

        while (pstack.size() > 0) {
            PElement current_elt = pstack.back();
            pstack.pop_back();

            while (current_elt.liste_transitions.size() > 0) {
                Transition *transition = current_elt.liste_transitions.back();
                current_elt.liste_transitions.pop_back();

                // On doit fire la transition correspondante apr�s avoir pr�ciser le marquage
                setMarquage(*(current_elt.marquage));
                //cout<<"\n The old  marquage="<<getMarquageName(getMarquage())<<endl;
                fire(*transition); // Franchissement

                Marking *new_state;
                new_state = new Marking(getMarquage());
                Marking *old_state = ms->insertMarking(new_state);
                if (!old_state) current_elt.marquage->addSucc(transition, new_state);
                else {
                    delete new_state;
                    new_state = nullptr;
                    current_elt.marquage->addSucc(transition, old_state);
                }
                if (!old_state) {
                    elt.marquage = new_state;
                    setMarquage(*(elt.marquage));
                    elt.liste_transitions = getListeTransitionsFranchissables();
                    if (elt.liste_transitions.size() > 0) pstack.push_back(elt);
                }
            }
        }

        ms->computeSCCTarjan();
        return ms;
    }


    void PetriNet::printMetaStateEx(MetaState *ms) {
        auto list_states = ms->getListMarkings();

        cout << "Number of states: " << list_states.size() << endl;
        cout << "Number of arcs: " << ms->getArcCount() << endl;
        //for (ms->
        /*for (int i=0;i<list_arcs->size();i++) {
            cout<<"Arc n°"<<i<<": ("<<getMarquageName(*list_arcs->at(i).getSource())<<" ===";
            cout<<list_arcs->at(i).getTransition()->getName()<<"==> ";
            cout<<getMarquageName(*list_arcs->at(i).getDestination())<<")"<<endl;
        }
        vector<SCC*>* list_sccs=ms->getListSCCs();
        cout<<"Number of SCCs: "<<list_sccs->size()<<endl;
        for (int i=0;i<list_sccs->size();i++) {
            cout<<"SCC n°"<<i<<endl;
            vector<Marking*>* lscc_states=list_sccs->at(i)->getListStates();
            for (int j=0;j<lscc_states->size();j++) {
                cout<<getMarquageName(*lscc_states->at(j))<<" , ";
            }
            cout<<endl;
        }*/
    }


    string PetriNet::getSCCName(const SCC *scc) const {
        auto name {getMarquageName(*(*(scc->getListStates()))[0])};
        for (auto & c : name)
            c=std::toupper(c);
        return name;
    }

    uint32_t PetriNet::getModulesCount() const {
        return m_modules_count;
    }

    void PetriNet::setModulesCount(const uint32_t v) {
        m_modules_count = v;
    }

    void PetriNet::setSyncTransitions(const std::vector<std::string>& list_transitions) {
        for (const auto &transition_name: list_transitions) {
            Transition *transition = getTransitionPtr(transition_name);
            if (transition != nullptr) {
                transition->setSync(true);
            }
        }
    }

    vector<string> PetriNet::getSyncTransitions() const {
        vector<string> list_transitions;
        for (const auto &transition: ml_transitions) {
            if (transition.isSync()) {
                list_transitions.push_back(transition.getName());
                m_manage_transition_fusion_set->add_fusion_set(transition.getName(),getPetriID());
            }
        }
        return list_transitions;
    }
    std::shared_ptr<ManageTransitionFusionSet> PetriNet::getManageTransitionFusionSet() const {
        return m_manage_transition_fusion_set;
    }

    set<string> PetriNet::getSyncEnabled(const MetaState* ms)  {
        set<string> res;
        const auto & l_markings {ms->getListMarkings()};
        for (const auto & marking : l_markings) {
            setMarquage(*marking);
            for (const auto & t : ml_transitions) {
                if (t.isSync() and t.isLocallyFirable()) res.insert(t.getName());
            }
        }
        return res;
    }



    std::set<FiringSyncTransition> PetriNet::fireSync(const string &name, const MetaState *ms) {
        std::set<FiringSyncTransition> res;
        const auto & l_markings {ms->getListMarkings()};
        auto transition {getTransitionPtr(name)};
        if (transition==nullptr) { // Module is not synchronized
            MetaState* dest_ms {getMetaState(*(ms->getInitialMarking()))};
            dest_ms->setSCCName(getSCCName(dest_ms->getInitialSCC()),getPetriID());
            res.insert(FiringSyncTransition{ms->getInitialSCC(),name,dest_ms->getInitialSCC()});
            return res;
        }
        // Module is synchronized on the transition
        for (const auto & marking : l_markings) {
           setMarquage(*marking);
            auto source_SCC {marking->getSCCContainer()};
            MetaState* dest_ms {nullptr};
            if (transition->isLocallyFirable()) {
                transition->fire();
                dest_ms=getMetaState(getMarquage());
                dest_ms->setSCCName(getSCCName(dest_ms->getInitialSCC()),getPetriID());
                // Check the existence of dest_ms in res
                auto check {find_if(res.begin(),res.end(),[dest_ms](const FiringSyncTransition &fst){
                    return *(dest_ms->getInitialSCC())==*(fst.getDestSCC());
                })};
                if (check==res.end()) { // Found
                    res.insert(FiringSyncTransition{source_SCC,name,dest_ms->getInitialSCC()});
                }
                else { // Not found
                    res.insert(FiringSyncTransition{source_SCC,name,check->getDestSCC()});
                    delete dest_ms;
                    dest_ms=nullptr;
                }
            }
        }
        return res;
    }

    Transition *PetriNet::getTransitionPtr(const string &name) {
        for (auto & t : ml_transitions) {
            if (t.getName()==name) return &t;
        }
        return nullptr;
    }
}
