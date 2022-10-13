#include "../include/node.hpp"

#include <iomanip>
#include <iostream>

namespace MAPF {
    Node::Node(int _id, int x, int y, int z)
            : id(_id), pos(Pos(x, y, z)) {}

    Node::~Node() {}

    int Node::getDegree() const { return neighbor.size(); }

    int Node::manhattanDist(const Node &node) const {
        return pos.manhattanDist(node.pos);
    }

    int Node::manhattanDist(const Node *const node) const {
        return pos.manhattanDist(node->pos);
    }

    float Node::euclideanDist(const Node &node) const {
        return pos.euclideanDist(node.pos);
    }

    float Node::euclideanDist(const Node *const node) const {
        return pos.euclideanDist(node->pos);
    }

    void Node::print() const {
        std::cout << "node[" << std::right << std::setw(6) << id << "]=<pos:";
        pos.print();
        std::cout << ", neigh: d=" << std::setw(1) << getDegree() << ", ";
        for (auto v: neighbor) {
            v->pos.print();
            std::cout << ", ";
        }
        std::cout << ">";
    }

    void Node::println() const {
        print();
        std::cout << std::endl;
    }

    bool Node::operator==(const Node &v) const { return v.id == id; };

    bool Node::operator!=(const Node &v) const { return v.id != id; };

    bool Node::operator==(const Node *const v) const { return v->id == id; };

    bool Node::operator!=(const Node *const v) const { return v->id != id; };
}