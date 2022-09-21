#include "../include/pos.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

namespace MAPF {
    Pos::Pos(int _x, int _y) : x(_x), y(_y) {}

    Pos::~Pos() {}

    void Pos::print() const {
        std::cout << "(" << std::right << std::setw(3) << x << ", " << std::right
                  << std::setw(3) << y << ")";
    }

    void Pos::println() const {
        print();
        std::cout << std::endl;
    }

    int Pos::manhattanDist(const Pos &pos) const {
        return std::abs(x - pos.x) + std::abs(y - pos.y);
    }

    float Pos::euclideanDist(const Pos &pos) const {
        float dx = x - pos.x;
        float dy = y - pos.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    bool Pos::operator==(const Pos &other) const {
        return x == other.x && y == other.y;
    }

    Pos Pos::operator+(const Pos &other) const {
        return Pos(x + other.x, y + other.y);
    }

    Pos Pos::operator-(const Pos &other) const {
        return Pos(x - other.x, y - other.y);
    }

    Pos Pos::operator*(const int i) const { return Pos(x * i, y * i); }

    void Pos::operator+=(const Pos &other) {
        x = x + other.x;
        y = y + other.y;
    }

    void Pos::operator-=(const Pos &other) {
        x = x - other.x;
        y = y - other.y;
    }

    void Pos::operator*=(const int i) {
        x = x * i;
        y = y * i;
    }
}