//
// Created by chiheb on 19/12/24.
//

#ifndef ARRAYMODEL_H
#define ARRAYMODEL_H


namespace dss {
    template<typename T>
    class ArrayModel {
    public:
        ArrayModel(size_t = 0);

        ArrayModel(const ArrayModel &);

        ArrayModel(ArrayModel &&);

        ArrayModel &operator=(const ArrayModel &);

        ArrayModel &operator=(ArrayModel &&);

        T &operator[](const size_t index);

        bool operator==(const ArrayModel &) const;

        size_t size() const;

        ~ArrayModel();

    private:
        T *m_data{nullptr};
        size_t m_size{0};
    };

    template <typename T>
       ArrayModel<T>::~ArrayModel() {
        if (m_data) delete []m_data;
    }

    template <typename T>
    ArrayModel<T>::ArrayModel(size_t s):m_size(s)  {
        m_data=new T[m_size];
    }

    template <typename T>
    T& ArrayModel<T>::operator[](const size_t index) {
        return m_data[index];
    }

    template <typename T>
    size_t ArrayModel<T>::size() const {
        return m_size;
    }

    template <typename T>
    bool ArrayModel<T>::operator==(const ArrayModel<T> &other) const {
        if  (m_size!=other.m_size) return false;
        for (size_t i{};i<m_size;++i) {
            if (m_data[i]!=other.m_data[i]) return false;
        }
        return true;
    }

    template <typename T>
    ArrayModel<T>::ArrayModel(const ArrayModel<T> &other) {
        m_size=other.m_size;
        m_data=new T[m_size];
        memcpy(m_data,other.m_data,m_size*sizeof(T));
    }

    template <typename T>
    ArrayModel<T>::ArrayModel(ArrayModel<T> &&other) {
        m_data=other.m_data;
        m_size=other.m_size;
        other.m_data=nullptr;
        other.m_size=0;
    }

    template <typename T>
    ArrayModel<T>& ArrayModel<T>::operator=(const ArrayModel<T> &other) {
        if (this==&other) return *this;
        for (size_t i{};i<m_size;++i) {
            m_data[i]=other.m_data[i];
        }
        return *this;
    }

    template <typename T>
    ArrayModel<T>& ArrayModel<T>::operator=(ArrayModel<T> &&other) {
        if (this==&other) return *this;
        if (m_data) delete []m_data;
        m_data=other.m_data;
        m_size=other.m_size;
        other.m_data=nullptr;
        other.m_size=0;
        return *this;
    }
}


#endif //ARRAYMODEL_H
