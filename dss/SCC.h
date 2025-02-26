#ifndef SCC_H
#define SCC_H
#include "misc.h"
namespace dss {


// This class represents a Strongly Connected Component
class MetaState;

class SCC {
    struct IteratorSucc {
        explicit IteratorSucc(SCC * ptr):m_ptr(ptr){
        }

        std::vector<std::pair<SCC * ,Transition*>>::iterator begin() {
            update();
            return m_succ.begin();
        }
        std::vector<std::pair<SCC * ,Transition*>>::iterator end() { return m_succ.end(); }
        std::vector<std::pair<SCC * ,Transition*>>& getSuccessors() {
            update();
            return m_succ;
        }
    private:
        void update();
        SCC *m_ptr;
        std::vector<std::pair<SCC *,Transition*>> m_succ;
    };

public:
    SCC();
    virtual ~SCC()=default;
    Marking *findMarking(Marking *m) const;
    size_t getCount() const;
    void addState(Marking *m);
    const vector<Marking *> *getListStates() const;
    bool isEqual(const SCC &scc);
    [[nodiscard]] uint32_t getId() const;
    [[nodiscard]] MetaState *getMetaState() const;
    void setMetaState(MetaState* ms);
    string getName(const PetriNet*) const;
    std::vector<std::pair<SCC * ,Transition*>>::iterator beginSucc() {
        return m_iterator_succ.begin();
    }
    std::vector<std::pair<SCC * ,Transition*>>::iterator endSucc() {
        return m_iterator_succ.end();
    }
    bool operator==(const SCC &o) const;
private:
    vector<Marking *> m_list;
    static uint32_t mCounter;
    uint32_t mId;
    MetaState *m_parentMetaState {nullptr};
    IteratorSucc m_iterator_succ {this};
};
}
#endif // SCC_H
