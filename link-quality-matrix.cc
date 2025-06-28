#include "link-quality-matrix.h"
#include <fstream>

LinkQualityMatrix::LinkQualityMatrix(const std::string& filename) {
    std::ifstream file(filename);
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::vector<double> row;
            // Parcourir la ligne et ajouter chaque valeur à la ligne
            // ...
            m_matrix.push_back(row);
        }
        file.close();
    }
}

double LinkQualityMatrix::GetLinkQuality(uint32_t node1, uint32_t node2) const {
    return m_matrix[node1][node2];
}
