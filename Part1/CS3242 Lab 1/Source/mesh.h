#pragma once

// maximum number of vertices and triangles
#define MAXV 1000000
#define MAXT 1000000

#define PI 3.141592654
#define MAX_ANGLE 180
#define RAD_TO_DEGREES 180.0 / 3.141592654
#define STATS_ANGLE_INTERVAL 10
#define EPSILON 0.000001
#define EPSILON_2 0.000000000001
#define THIRD 1.0 / 3.0

#define X 0
#define Y 1
#define Z 2

#define STL_HEADER_LENGTH 80
#define STL_TCOUNT_LENGTH 4

#define FRONT_GOOD 0
#define BACK_GOOD 1
#define FRONT_BAD 2
#define BACK_BAD 3

#include "math.h"
#include <unordered_map>
#include <map>
#include <utility>
#include <vector>

using namespace std;

typedef int OrTri;
typedef int tIdx;

inline OrTri makeOrTri(tIdx t, int version) { return (t << 3) | version; };
inline tIdx idx(OrTri ot) { return ot >> 3; };
inline int ver(OrTri ot) { return ot & 0b111; };

inline OrTri enext(OrTri ot) {
    tIdx t = idx(ot);
    int version = ver(ot);

    if (version == 2) {
        return makeOrTri(t, 0);
    } else if (version == 5) {
        return makeOrTri(t, 3);
    } else {
        return makeOrTri(t, version + 1);
    }
};

inline OrTri sym(OrTri ot) {
    tIdx t = idx(ot);
    int version = ver(ot);

    if (version < 3) {
        return makeOrTri(t, version + 3);
    } else {
        return makeOrTri(t, version - 3);
    }
};

struct Point {
    double v[2];

    Point(double v0 = 0, double v1 = 0) {
        v[0] = v0;
        v[1] = v1;
    }

    bool operator == (const Point &other) const {
        double s = other.v[0] - v[0];
        double t = other.v[1] - v[1];
        return (s * s + t * t) < EPSILON_2;
    }
};

struct Point3 {
    double v[3];

    Point3(double (&v0), double (&v1), double (&v2)) {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
    }

    bool operator == (const Point3 &other) const {
        double s = other.v[0] - v[0];
        double t = other.v[1] - v[1];
        double u = other.v[2] - v[2];

        return (s * s + t * t + u * u) < EPSILON;
    }
};

struct Edge {
    int org;
    int dest;

    Edge(int org = 0, int dest = 0) : org(org), dest(dest) {}

    inline Edge reversed() {
        return Edge(dest, org);
    };

    bool operator == (const Edge &other) const {
        return org == other.org && dest == other.dest;
    }
};

struct hash_edge {
    size_t operator() (const Edge &edge) const {
        return edge.org * 31 + edge.dest;
    }
};

struct Triangle {
    int v[3];
    Edge edges[3];

    Triangle(int v0, int v1, int v2) {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
        edges[0] = Edge(v0, v1);
        edges[1] = Edge(v1, v2);
        edges[2] = Edge(v2, v0);
    }
};

struct Intersection {
    double start[3];
    double end[3];
};

class myObjType {
	int vcount = 0;
	int tcount = 0;

	double vlist[MAXV][3];     // vertices list
    double vnlist[MAXV][3];    // storing vertex normals

	int tlist[MAXT][3];        // triangle list
	int fnlist[MAXT][3];       // fnext list for future (not this assignment)
	double nlist[MAXT][3];     // storing triangle normals
	
	double lmax[3];            // the maximum coordinates of x,y,z
	double lmin[3];            // the minimum coordinates of x,y,z

	int statMinAngle[18];      // each bucket is  degrees has a 10 degree range from 0 to 180 degree
	int statMaxAngle[18];

    int xvcount = 0;           // count extra vertices after retriangulating the model
    int xtcount = 0;           // count extra triangle after retriangulating the model
    bool highlight[MAXT];      // for highlighting intersecting triangles
    bool focus[MAXT];          // for drawing selected triangles (to see intersecting triangles clearer)
    bool isFocused;            // true if triangles have been selected, false otherwise
    unordered_map<int, vector<Edge>> crossingLines; // interections lines of intersecting triangles

    int tempTlist[MAXT][3];    // triangle list after retriangulating the model
    double tempNlist[MAXT][3]; // normal list after retriangulating the model

    bool smoothShading = true; // enable for smooth shading
    float mat_diffuse[4][4] = {// diffuse material colours for good and bad faces
        { 0.1f, 0.5f, 0.8f, 1.0f },
        { 0.8f, 0.8f, 0.8f, 1.0f },
        { 0.9f, 0.5f, 0.2f, 1.0f },
        { 1.0f, 0.7f, 0.4f, 1.0f }
    };


public:
	myObjType() { vcount = 0; tcount = 0; };
	void readFile(char* filename);  // assumming file contains a manifold
    void readSTL(char* filename);
	void writeFile(char* filename);
    void reset();
	void draw();  
    void computeStat();
    void computeFnlist();
    void computeVnlist();
    int countComponents();
    int org(OrTri ot);
    int dest(OrTri ot);
    OrTri fnext(OrTri ot);

    void clearHighlight();
    void clearFocus();
    bool addFocus(tIdx t);
    void setThickness(double dt);
    void setSmoothShading(bool on);
    void setMatDiffuse(int face, float (&mat)[4]);
    void computeIntersections();
    void repairIntersections();
    void removeHiddenFaces();

private:
    bool raycast(tIdx t, double (&origin)[3], double (&direction)[3], double &facing);
    bool intersects(tIdx t1, tIdx t2, double (&start)[3], double (&end)[3]);

    bool isCoplanarIsect(double (&v0)[3], double (&v1)[3], double (&v2)[3],
                         double (&p)[3], double (&d)[3], double dx01, double dx02,
                         double (&isect)[2], double (&isectPoint0)[3], double (&isectPoint1)[3]);

    void isectPoints(double (&v0)[3], double (&v1)[3], double (&v2)[3],
                     double p0, double p1, double p2,
                     double d0, double d1, double d2,
                     double (&isect)[2], double (&isectPoint0)[3], double (&isectPoint1)[3]);

    void crossingLine(double (&isect1)[2], double (&isectPoint10)[3], double (&isectPoint11)[3], bool swapped1,
                      double (&isect2)[2], double (&isectPoint20)[3], double (&isectPoint21)[3], bool swapped2,
                      double (&start)[3], double (&end)[3]);
};

// Vector operations

inline void copy(int (&vector)[3], int (&result)[3]) {
    for (int coord = 0; coord < 3; coord++) {
        result[coord] = vector[coord];
    }
}

inline void copy(double (&vector)[3], double (&result)[3]) {
    for (int coord = 0; coord < 3; coord++) {
        result[coord] = vector[coord];
    }
}

inline void add(double (&vA)[3], double (&vB)[3], double (&result)[3]) {
    for (int coord = 0; coord < 3; coord++) {
        result[coord] = vA[coord] + vB[coord];
    }
}

inline void sub(double (&vA)[3], double (&vB)[3], double (&result)[3]) {
    for (int coord = 0; coord < 3; coord++) {
        result[coord] = vA[coord] - vB[coord];
    }
}

inline void scale(double (&vector)[3], double scale, double (&result)[3]) {
    for (int coord = 0; coord < 3; coord++) {
        result[coord] = vector[coord] * scale;
    }
}

inline void crossProduct(double (&vA)[3], double (&vB)[3], double (&result)[3]) {
    result[X] = vA[Y] * vB[Z] - vA[Z] * vB[Y];
    result[Y] = vA[Z] * vB[X] - vA[X] * vB[Z];
    result[Z] = vA[X] * vB[Y] - vA[Y] * vB[X];
}

inline double dotProduct(double (&vA)[3], double (&vB)[3]) {
    double result = 0;
    for (int coord = 0; coord < 3; coord++) {
        result += vA[coord] * vB[coord];
    }
    return result;
}

inline double magnitude(double (&vector)[3]) {
    double result = 0;
    for (int coord = 0; coord < 3; coord++) {
        result += vector[coord] * vector[coord];
    }
    return sqrt(result);
}

inline void normalize(double (&vector)[3], double (&result)[3]) {
    scale(vector, 1 / magnitude(vector), result);
}

inline double angleBetweenVectors(double (&vA)[3], double (&vB)[3]) {
    return acos(dotProduct(vA, vB) / (magnitude(vA) * magnitude(vB))) * RAD_TO_DEGREES;
}

inline bool equal(double (&vA)[3], double (&vB)[3]) {
    double result[3];
    sub(vA, vB, result);
    double sqm = dotProduct(result, result);
    return sqm < EPSILON;
}

inline double leftTurn(double (&vA)[2], double (&vB)[2], double (&vC)[2]) {
    return (vC[0] - vB[0]) * (vA[1] - vB[1]) - (vC[1] - vB[1]) * (vA[0] - vB[0]);
}

inline double incircle(double (&vA)[2], double (&vB)[2], double (&vC)[2], double (&vD)[2]) {
    double sqmA = vA[0] * vA[0] + vA[1] * vA[1];
    double sqmB = vB[0] * vB[0] + vB[1] * vB[1];
    double sqmC = vC[0] * vC[0] + vC[1] * vC[1];
    double sqmD = vD[0] * vD[0] + vD[1] * vD[1];

    double detA = leftTurn(vB, vC, vD);
    double detB = leftTurn(vA, vC, vD);
    double detC = leftTurn(vA, vB, vD);
    double detD = leftTurn(vA, vB, vC);

    return (sqmA * detA - sqmB * detB + sqmC * detC - sqmD * detD) * detD;
}

// Byte stream operations

inline int parseInt(char bytes[4]) {
    uint8_t *buffer = reinterpret_cast<uint8_t *>(bytes);

    // Fix for little endianness
    int value = (uint32_t)buffer[0]
                | ((uint32_t)buffer[1] << 8)
                | ((uint32_t)buffer[2] << 16)
                | ((uint32_t)buffer[3] << 24);
    return value;
}

inline double parseDouble(char bytes[4]) {
    uint8_t *buffer = reinterpret_cast<uint8_t *>(bytes);

    // Fix for little endianness
    int value = (uint32_t)buffer[0]
                | ((uint32_t)buffer[1] << 8)
                | ((uint32_t)buffer[2] << 16)
                | ((uint32_t)buffer[3] << 24);

    return (double)reinterpret_cast<float &>(value);
}

class Delaunay {
public:
    double superVertices[3][3];

    unordered_map<int, Point> vertices;
    vector<Triangle> triangles;

    Delaunay() { }

    Delaunay(int (&v)[3], double (&n)[3], double (&vA)[3], double (&vB)[3], double (&vC)[3], vector<Edge> &constraints) {
        triangles.push_back(Triangle(v[0], v[1], v[2]));

        copy(vA, superVertices[0]);
        copy(vB, superVertices[1]);
        copy(vC, superVertices[2]);

        double u[3];
        sub(vB, vA, s);
        sub(vC, vA, u);
        crossProduct(n, s, t);

        dotS = 1 / dotProduct(s, s);
        dotT = 1 / dotProduct(t, t);

        double dS = dotProduct(u, s) * dotS;
        double dT = dotProduct(u, t) * dotT;

        vertices.emplace(v[0], Point(0, 0));
        vertices.emplace(v[1], Point(1, 0));
        vertices.emplace(v[2], Point(dS, dT));
    }

    inline void insert(int d, double (&vD)[3]) {
        double u[3];
        sub(vD, superVertices[0], u);

        double dS = dotProduct(u, s) * dotS;
        double dT = dotProduct(u, t) * dotT;
        Point pointD = Point(dS, dT);
        vertices.emplace(d, pointD);

        vector<Triangle> temps;
        unordered_map<Edge, int, hash_edge> edges;
        for (auto &t : triangles) {
            if (incircle(vertices[t.v[0]].v, vertices[t.v[1]].v, vertices[t.v[2]].v, pointD.v) >= 0) {
                auto findE0 = edges.find(t.edges[0]);
                if (findE0 == edges.end()) {
                    findE0 = edges.find(t.edges[0].reversed());
                    if (findE0 == edges.end()) {
                        edges.emplace(t.edges[0], 1);
                    } else {
                        findE0->second++;
                    }
                } else {
                    findE0->second++;
                }

                auto findE1 = edges.find(t.edges[1]);
                if (findE1 == edges.end()) {
                    findE1 = edges.find(t.edges[1].reversed());
                    if (findE1 == edges.end()) {
                        edges.emplace(t.edges[1], 1);
                    } else {
                        findE1->second++;
                    }
                } else {
                    findE1->second++;
                }

                auto findE2 = edges.find(t.edges[2]);
                if (findE2 == edges.end()) {
                    findE2 = edges.find(t.edges[2].reversed());
                    if (findE2 == edges.end()) {
                        edges.emplace(t.edges[2], 1);
                    } else {
                        findE2->second++;
                    }
                } else {
                    findE2->second++;
                }
            } else {
                temps.push_back(t);
            }
        }

        Edge e;
        for (auto const& entry : edges) {
            if (entry.second > 1) {
                continue;
            }
            e = entry.first;
            double collinear = leftTurn(vertices[e.org].v, vertices[e.dest].v, pointD.v);
            if (collinear * collinear <= EPSILON_2) {
                continue;
            }
            temps.push_back(Triangle(e.org, e.dest, d));
        }
        triangles = temps;
    }

private:
    double s[3], dotS;
    double t[3], dotT;
};
