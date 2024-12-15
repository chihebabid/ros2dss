//
// Created by chiheb on 14/12/24.
//

#ifndef BUILDPETRI_H
#define BUILDPETRI_H


namespace dss {
    class BuildPetri {
    public:
        PetriNet *getPetriNet();

        void setFileName(const std::string& name);

        std::string getFileName();

        BuildPetri();

        virtual ~BuildPetri();

    private:
        std::string getNextWord();

        FILE *fp;
        std::string m_nom_fichier;
    };
}


#endif //BUILDPETRI_H
