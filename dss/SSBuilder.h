//
// Created by chiheb on 17/12/24.
//

#ifndef SSBUILDER_H
#define SSBUILDER_H


namespace dss {
    class SSBuilder {
    public:
        SSBuilder(PetriNet *);
    private:
        PetriNet *m_petri_net;
        ModuleSS *m_ss {};
    };
}


#endif //SSBUILDER_H
