//
// Created by chiheb on 11/12/24.
//

#ifndef MARKINGARRAY_H
#define MARKINGARRAY_H
#include <cstring>
using  byte_t=unsigned char;
namespace dss {
    class MarkingArray {
    public:
        MarkingArray(size_t=0);
        MarkingArray(const MarkingArray &);
        MarkingArray(MarkingArray &&);
        MarkingArray& operator=(const MarkingArray &);
        MarkingArray& operator=(MarkingArray &&);
        byte_t& operator[](const size_t index);
        const byte_t& operator[](const size_t index) const;
        bool operator==(const MarkingArray &) const;
        size_t size() const;
        ~MarkingArray();
    private:
        byte_t* m_marking {nullptr};
        size_t m_size {0};
    };
}


#endif //MARKINGARRAY_H
