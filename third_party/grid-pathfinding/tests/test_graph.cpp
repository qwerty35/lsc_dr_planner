#include <graph.hpp>

#include "gtest/gtest.h"

TEST(Graph, loading)
{
  Grid G("../map/lak105d.map");
  ASSERT_TRUE(G.existNode(0));
  ASSERT_TRUE(G.existNode(455));
  ASSERT_TRUE(G.existNode(0, 0));
  ASSERT_FALSE(G.existNode(5));
  ASSERT_EQ(G.getNode(0)->getDegree(), 2);
  ASSERT_EQ(G.getV().size(), 443);
  ASSERT_EQ(G.getWidth(), 31);
  ASSERT_EQ(G.getHeight(), 25);
  ASSERT_EQ(G.getNodesSize(), 775);
}

TEST(Graph, small_filed)
{
  Grid G("../map/lak105d.map");
  Node* v = G.getNode(0);
  Node* u = G.getNode(455);
  ASSERT_EQ(v->manhattanDist(u), 35);
  ASSERT_TRUE(25 < v->euclideanDist(u) && v->euclideanDist(u) < 26);
  ASSERT_EQ(G.pathDist(v, u, false), 39);
  ASSERT_EQ(G.pathDist(v, u, true), 39);
}

TEST(Graph, large_field_without_cache)
{
  Grid G("../map/brc202d.map");
  Node* v = G.getNode(216, 138);
  Node* u = G.getNode(203, 303);
  ASSERT_EQ(G.pathDist(v, u, false), 782);
}

TEST(Graph, large_field_with_cache)
{
  Grid G("../map/brc202d.map");
  Node* v = G.getNode(216, 138);
  Node* u = G.getNode(203, 303);
  ASSERT_EQ(G.pathDist(v, u, true), 782);
}

TEST(Graph, huge_field_without_cache)
{
  Grid G("../map/ost000a.map");
  Node* v = G.getNode(273, 721);
  Node* u = G.getNode(84, 461);
  ASSERT_EQ(G.pathDist(v, u, false), 863);
}

TEST(Graph, huge_field_with_cache)
{
  Grid G("../map/ost000a.map");
  Node* v = G.getNode(273, 721);
  Node* u = G.getNode(84, 461);
  ASSERT_EQ(G.pathDist(v, u, true), 863);
}
