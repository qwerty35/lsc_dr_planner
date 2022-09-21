#pragma once
#include <vector>

#include "pos.hpp"

namespace MAPF {
    struct Node;
    using Nodes = std::vector<Node*>;

    struct Node {
        const int id;
        const Pos pos;
        Nodes neighbor;

        Node(int _id, int x, int y);

        ~Node();

        int getDegree() const;

        int manhattanDist(const Node &node) const;

        int manhattanDist(const Node *const node) const;

        float euclideanDist(const Node &node) const;

        float euclideanDist(const Node *const node) const;

        void print() const;

        void println() const;

        bool operator==(const Node &v) const;

        bool operator!=(const Node &v) const;

        bool operator==(const Node *const v) const;

        bool operator!=(const Node *const v) const;
    };
}