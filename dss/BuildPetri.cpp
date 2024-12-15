//
// Created by chiheb on 14/12/24.
//

#include "misc.h"

// ConstructPetriFromFile.cpp: implementation of the ConstructPetriFromFile class.
//
//////////////////////////////////////////////////////////////////////
namespace dss {
    BuildPetri::BuildPetri():fp(nullptr) {

    }

    BuildPetri::~BuildPetri() {

    }

    void BuildPetri::setFileName(const std::string& name) {
        m_nom_fichier = name;
    }

    std::string BuildPetri::getFileName() {
        return m_nom_fichier;
    }

    PetriNet *BuildPetri::getPetriNet() {
        PetriNet *Petri {nullptr};


        fp = fopen(m_nom_fichier.c_str(), "r");
        if (fp) {
            //Cr�er le r�seau de Petri
            std::cout << "Loading Petri net from file..." << std::endl;

            //Lecture de nombre de modules
            unsigned int nb_modules = 0;
            std::string mot;
            getNextWord();
            mot = getNextWord();
            nb_modules = atoi(mot.c_str());
            std::cout << "This modular Petri net consists of " << mot << " modules" << std::endl;
            std::cout << "Displaying informations related to the modular Petri net" << std::endl;
            //Initialisation
            PetriNet *petri;
            std::vector<Place> liste_places;
            std::vector<Transition> liste_transitions;
            // for (unsigned int ii = 0; ii < nb_modules; ii++) {
                int code_transition = 0;



                getNextWord();
                //
                // Lire les libell�s de places
                std::string chaine;
                petri = new PetriNet();
                // Sp�cifier le num du module
                petri->setNumero(ii);
                Place place;
                liste_places.clear();
                /******************************************/
                /** Ajout des places et de leurs marquages*/
                /******************************************/
                do {
                    int marquage = 00;
                    mot = getNextWord();
                    while (mot == "Marquage") {
                        mot = getNextWord();
                        marquage = atoi(mot.c_str());
                        //					printf("marquage :%d \n",marquage);

                        do {
                            mot = getNextWord();
                            if (mot != "Marquage" && mot != "End") {
                                place.setName(mot.c_str());
                                place.setTokens(marquage);
                                liste_places.push_back(place);
                                cout << "Added place: " << mot.c_str() << endl;
                            }
                        } while (mot != "Marquage" && mot != "End");
                    }
                } while (mot != "End");
                petri->addListPlaces(liste_places);
                /****************************************/
                /** Lecture de libell�s de transitions **/
                /****************************************/
                getNextWord();
                Transition transition;
                liste_transitions.clear();
                do {
                    mot = getNextWord();
                    if (mot != "End") {
                        transition.setName(mot);
                        transition.setCode(code_transition);
                        code_transition++;
                        //					printf("Transition ajout�e :%s*\n",mot.c_str());
                        liste_transitions.push_back(transition);
                    }
                } while (mot != "End");
                petri->addListTransitions(liste_transitions);

                /****************************************/
                /** Lecture des entr�es de transitions **/
                /****************************************/

                getNextWord();
                vector<string> liste_places_entrees;

                vector<int> liste_poids;
                string nom_transition;
                do {
                    nom_transition = getNextWord();
                    if (nom_transition != "End") {
                        std::cout << "Transition: " << nom_transition.c_str() << " has as place inputs ";
                        string item;
                        while (item != "]") {
                            item = getNextWord();
                            if (item == "-") {
                                string poids;
                                poids = getNextWord();
                                int poids_entier = atoi(poids.c_str());
                                liste_poids.pop_back();
                                liste_poids.push_back(poids_entier);
                            } else if (item != "[" && item != "]") {
                                std::cout << item.c_str() << "<";
                                liste_places_entrees.push_back(item);
                                liste_poids.push_back(1);
                            }
                        }
                        if (liste_places_entrees.size() == liste_poids.size())
                            petri->addPlacesEntrees(nom_transition, liste_places_entrees, liste_poids);
                        else std::cout << "Erreur liee au poids...";
                        liste_places_entrees.clear();
                        liste_poids.clear();
                    }
                    std::cout << std::endl;
                } while (nom_transition != "End");



                /****************************************/
                /** Lecture des sorties de transitions **/
                /****************************************/
                getNextWord();
                std::vector<std::string> liste_places_sorties;
                liste_poids.clear();
                do {
                    nom_transition = getNextWord();
                    if (nom_transition != "End") {
                       std::cout << "Transition: " << nom_transition.c_str() << " has as output places ";
                        std::string item;
                        while (item != "]") {
                            item = getNextWord();
                            if (item == "-") {
                                string poids;
                                poids = getNextWord();
                                int poids_entier = atoi(poids.c_str());
                                liste_poids.pop_back();
                                liste_poids.push_back(poids_entier);
                            } else if (item != "[" && item != "]") {
                                cout << ">" << item.c_str();
                                liste_places_sorties.push_back(item);
                                liste_poids.push_back(1);
                            }
                        }
                        if (liste_places_sorties.size() == liste_poids.size())
                            petri->addPlacesSorties(nom_transition, liste_places_sorties, liste_poids);
                        else std::cout << "erreur liee au poids...";
                        liste_places_sorties.clear();
                        liste_poids.clear();
                    }
                    std::cout << std::endl;
                } while (nom_transition != "End");


            /*************************************************/
            /** Lecture de transititions de syn,chronisation**/
            /*************************************************/
            std::cout << "Determining synchronised transitions..." << std::endl;
            getNextWord();
            std::string transition;
            std::vector<string> liste_sync;
            std::string temp;
            do {
                transition = getNextWord();
                if (transition != "End") {
                    cout << transition.c_str() << " ";
                    temp = transition;
                    liste_sync.emplace_back(std::move(temp));
                }

            } while (transition != "End");
           // MPetri->setSync(liste_sync);
        } else {
            cout << "Error: can't open file";
        }
        fclose(fp);
        return petri;
    }


    std::string BuildPetri::getNextWord() {
        std::string separateur = ",;(:)";
        std::string chaine;
        int c = '\0';
        while (c <= (char) 32 || separateur.find_first_of(c) != -1) {
            c = fgetc(fp);
        }
        bool quit = false;
        while (c > 32 && !quit) {
            if (separateur.find_first_of(c) != -1) {
                quit = true;
            } else {
                chaine += c;
                c = fgetc(fp);
            }
        }
        return chaine;
    }
}