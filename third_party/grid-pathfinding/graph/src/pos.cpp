#include "../include/pos.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

namespace MAPF {
    Pos::Pos(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}

    Pos::~Pos() {}

    void Pos::print() const {
        std::cout << "(" << std::right << std::setw(3) << x << ", "
                         << std::right << std::setw(3) << y << ", "
                         << std::right << std::setw(3) << z << ")";
    }

    void Pos::println() const {
        print();
        std::cout << std::endl;
    }

    int Pos::manhattanDist(const Pos &pos) const {
        return std::abs(x - pos.x) + std::abs(y - pos.y) + std::abs(z - pos.z);
    }

    float Pos::euclideanDist(const Pos &pos) const {
        float dx = x - pos.x;
        float dy = y - pos.y;
        float dz = z - pos.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    bool Pos::operator==(const Pos &other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    Pos Pos::operator+(const Pos &other) const {
        return Pos(x + other.x, y + other.y, z + other.z);
    }

    Pos Pos::operator-(const Pos &other) const {
        return Pos(x - other.x, y - other.y, z - other.z);
    }

    Pos Pos::operator*(const int i) const { return Pos(x * i, y * i, z * i); }

    void Pos::operator+=(const Pos &other) {
        x = x + other.x;
        y = y + other.y;
        z = z + other.z;
    }

    void Pos::operator-=(const Pos &other) {
        x = x - other.x;
        y = y - other.y;
        z = z - other.z;
    }

    void Pos::operator*=(const int i) {
        x = x * i;
        y = y * i;
        z = z * i;
    }
}