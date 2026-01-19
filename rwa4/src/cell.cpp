#include "cell.hpp"
#include <cctype>

void Cell::markWall(char dir) noexcept {
    char d = static_cast<char>(std::tolower(static_cast<unsigned char>(dir)));
    switch (d) {
        case 'n': wallN = true; break;
        case 'e': wallE = true; break;
        case 's': wallS = true; break;
        case 'w': wallW = true; break;
        default:  break;
    }
}

bool Cell::hasWall(char dir) const noexcept {
    char d = static_cast<char>(std::tolower(static_cast<unsigned char>(dir)));
    switch (d) {
        case 'n': return wallN;
        case 'e': return wallE;
        case 's': return wallS;
        case 'w': return wallW;
        default:  return false;
    }
}