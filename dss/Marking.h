// BitsVector.h: interface for the BitsVector class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_BITSVECTOR_H__FCB7EB69_05AC_41F8_9A21_7807D91657A0__INCLUDED_)
#define AFX_BITSVECTOR_H__FCB7EB69_05AC_41F8_9A21_7807D91657A0__INCLUDED_

#if _MSC_VER > 1000
#pragma once

#endif // _MSC_VER > 1000




using namespace std;
typedef unsigned char Octet;


namespace dss {
    class Transition;

    class SCC;
class Marking {
public:
    int index; // Used in Tarjan algorithm
    int lowlink; // Used in Tarjan algorithm
    bool onstack; // Used in Tarjan algorithm
    Marking &operator=(const Marking &newvec);

    Marking(const Marking &value);

    Octet get8BitsValue(const int index) const;

    bool operator==(const Marking &newvec) const {
        return m_data==newvec.m_data;
    }



    void add8BitsValue(const Octet val,const int index);

    void addSucc(Transition *, Marking *);

    vector<pair<Transition *, Marking *>> &getListSucc();

    explicit Marking(const size_t);
    ~Marking()=default;

    SCC *getSCCContainer() const;
    void setSCCContainer(SCC *c);
    friend ostream &operator<<(ostream &stream, const Marking &);

private:
    MarkingArray m_data;
    vector<pair<Transition *, Marking *>> ml_succ;
    SCC *m_sccContainer;
};
}





#endif // !defined(AFX_BITSVECTOR_H__FCB7EB69_05AC_41F8_9A21_7807D91657A0__INCLUDED_)
