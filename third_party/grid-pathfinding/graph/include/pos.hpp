#pragma once

namespace MAPF {
    struct Pos {
        int x;
        int y;
        int z;

        Pos(int _x, int _y, int _z);

        ~Pos();

        void print() const;

        void println() const;

        int manhattanDist(const Pos &pos) const;

        float euclideanDist(const Pos &pos) const;

        bool operator==(const Pos &other) const;

        Pos operator+(const Pos &other) const;

        Pos operator-(const Pos &other) const;

        Pos operator*(const int i) const;

        void operator+=(const Pos &other);

        void operator-=(const Pos &other);

        void operator*=(const int i);
    };
}
