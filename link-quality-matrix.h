#include <vector>
#include <sstream>
#include <string>
#include <cstdlib>

using namespace std;

class LinkQualityMatrix {
public:
    LinkQualityMatrix(const std::string& filename);
    double GetLinkQuality(uint32_t node1, uint32_t node2) const;

private:
    std::vector<std::vector<double>> m_matrix;
};
