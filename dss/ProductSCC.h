#ifndef PRODUCTSCC_H
#define PRODUCTSCC_H

#include "misc.h"
namespace dss {
class ProductSCC {
public:
    ProductSCC()=default;
    virtual ~ProductSCC()=default;

    void addSCC(SCC *scc);

    bool operator==(const ProductSCC &product);



    SCC *getSCC(const size_t pos);

private:
    vector<SCC *> m_product;
};
}
#endif // PRODUCTSCC_H
