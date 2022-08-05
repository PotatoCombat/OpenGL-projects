#include "mesh.h"

#ifdef _WIN32
#include <Windows.h>
#include "GL\glut.h"
#define M_PI 3.141592654
#elif __APPLE__
#include <OpenGL/gl.h>
#include <GLUT/GLUT.h>
#endif

#include <array>
#include "math.h"
#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include "mesh.h"
#include <map>
#include <queue>
#include <iomanip>
#include <unordered_map>
#include <vector>
using namespace std;

#define TVERTEX(t, v) vlist[tlist[t][v]]
#define TVERTEX_N(t, v) vnlist[tlist[t][v]]


#define SWAP(a, b, swapped) \
    if (a > b) { \
        double c; \
        c = a; \
        a = b; \
        b = c; \
        swapped = true; \
    } else { \
        swapped = false; \
    }

void myObjType::draw() {

	glEnable(GL_LIGHTING);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glPushMatrix();
	double longestSide = 0.0;
	for (int i = 0; i < 3; i++)
		if ((lmax[i] - lmin[i]) > longestSide)
			longestSide = (lmax[i] - lmin[i]);
	glScalef(4.0 / longestSide, 4.0 / longestSide, 4.0 / longestSide);
	glTranslated(-(lmin[0] + lmax[0]) / 2.0, -(lmin[1] + lmax[1]) / 2.0, -(lmin[2] + lmax[2]) / 2.0);

	for (int i = 1; i <= tcount; i++)
	{
        // For drawing selected triangles to see intersecting triangles
        if (isFocused && !focus[i]) {
            continue;
        }

        if (highlight[i]) {
            // Draw intersecting triangles in orange
            glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse[FRONT_BAD]);
            glMaterialfv(GL_BACK, GL_DIFFUSE, mat_diffuse[BACK_BAD]);
        } else {
            // Draw normal triangles in blue
            glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse[FRONT_GOOD]);
            glMaterialfv(GL_BACK, GL_DIFFUSE, mat_diffuse[BACK_GOOD]);
        }

        if (smoothShading) {
            // Shade smoothly using vertex normals
            glBegin(GL_POLYGON);
            for (int j = 0; j < 3; j++) {
                glNormal3dv(TVERTEX_N(i, j));
                glVertex3dv(TVERTEX(i, j));
            }
            glEnd();

        } else {
            // Shade flatly using face normals
            glBegin(GL_POLYGON);
            glNormal3dv(nlist[i]);
            for (int j = 0; j < 3; j++) {
                glVertex3dv(TVERTEX(i, j));
            }
            glEnd();
        }
	}
	glDisable(GL_LIGHTING);
	glPopMatrix();
}

void myObjType::writeFile(char* filename)
{
    cout << "Writing " << filename << endl;
    ofstream outFile;
    outFile.open(filename);
    if (!outFile.is_open()) {
        cout << "We cannot write your file " << filename << endl;
        exit(1);
    }

    int i, j;

    for (i = 1; i <= vcount; i++) {
        outFile << "v";
        for (j = 0; j < 3; j++) {
            outFile << " " << vlist[i][j];
        }
        outFile << endl;
    }

    for (i = 1; i <= tcount; i++) {
        outFile << "f";
        for (j = 0; j < 3; j++) {
            outFile << " " << tlist[i][j];
        }
        outFile << endl;
    }

    outFile.close();
}

void myObjType::readFile(char* filename)
{
	cout << "Opening " << filename << endl;
	ifstream inFile;
	inFile.open(filename);
	if (!inFile.is_open()) {
		cout << "We cannot find your file " << filename << endl;
		exit(1);
	}

	string line;
	int i, j;
	bool firstVertex = 1;
	double currCood;

	while (getline(inFile, line))
	{
		if ((line[0] == 'v' || line[0] == 'f') && line[1] == ' ')
		{
			if (line[0] == 'v')
			{
				vcount++;
				i = 1;
				const char* linec = line.data();
				for (int k = 0; k < 3; k++) { // k is 0,1,2 for x,y,z
					while (linec[i] == ' ') i++;
					j = i;
					while (linec[j] != ' ') j++;
					currCood = vlist[vcount][k] = atof(line.substr(i, j - i).c_str());
					if (firstVertex) 
						lmin[k] = lmax[k] = currCood;
					else {
						if (lmin[k] > currCood)
							lmin[k] = currCood;
						if (lmax[k] < currCood)
							lmax[k] = currCood;
					}
					i = j;
				}

				firstVertex = 0;
			}
			if (line[0] == 'f')
			{
				tcount++;
				i = 1;
				const char* linec = line.data();
				for (int k = 0; k < 3; k++) {
					while (linec[i] == ' ') i++;
					j = i;
					while (linec[j] != ' ' && linec[j] != '\\') j++;
					tlist[tcount][k] = atof(line.substr(i, j - i).c_str());
					i = j;
					fnlist[tcount][k] = 0;
					while (linec[j] != ' ') j++;
				}
			}
		}
	}

	// We suggest you to compute the normals here
    double vA[3], vB[3], result[3];
    for (tIdx t = 1; t <= tcount; t++) {
        sub(vlist[tlist[t][1]], vlist[tlist[t][0]], vA);
        sub(vlist[tlist[t][2]], vlist[tlist[t][0]], vB);
        crossProduct(vA, vB, result);
        normalize(result, nlist[t]);
    }

    cout << "No. of vertices: " << vcount << endl;
    cout << "No. of triangles: " << tcount << endl;
    computeStat();
    computeFnlist();
    computeVnlist();
    cout << "No. of components: " << countComponents() << endl;
}

void myObjType::readSTL(char* filename)
{
    cout << "Opening STL " << filename << endl;
    ifstream inFile;
    inFile.open(filename, std::ios::in | std::ios::binary);
    if (!inFile.is_open()) {
        cout << "We cannot find your STL file " << filename << endl;
        exit(1);
    }

    char header[STL_HEADER_LENGTH + 1] = { NULL };
    char bytes[4];

    inFile.read(header, STL_HEADER_LENGTH);
    inFile.read(bytes, 4);

    tcount = parseInt(bytes);
    vcount = 0;

    int coord;
    bool firstVertex = 1;

    for (int tIdx = 1; tIdx <= tcount; tIdx++) {
        for (coord = 0; coord < 3; coord++) {
            inFile.read(bytes, 4);
            nlist[tIdx][coord] = parseDouble(bytes);
        }
        for (int vIdx = 0; vIdx < 3; vIdx++) {
            vcount++;
            tlist[tIdx][vIdx] = vcount;
            fnlist[tIdx][vIdx] = 0;

            for (coord = 0; coord < 3; coord++) {
                inFile.read(bytes, 4);
                double value = vlist[vcount][coord] = parseDouble(bytes);
                if (firstVertex) {
                    lmin[coord] = lmax[coord] = value;
                } else {
                    if (lmin[coord] > value) {
                        lmin[coord] = value;
                    }
                    if (lmax[coord] < value) {
                        lmax[coord] = value;
                    }
                }
            }

            firstVertex = 0;
        }

        inFile.read(header, 2); // Unused attribute byte count
    }

    cout << "Header: " << header << endl;
    cout << "No. of vertices: " << vcount << endl;
    cout << "No. of triangles: " << tcount << endl;
    computeStat();
    computeFnlist();
    computeVnlist();
    cout << "No. of components: " << countComponents() << endl;
}

void myObjType::computeStat()
{
	int i;
    double minAngle = MAX_ANGLE;
    double maxAngle = 0;

    // Computing angle statistics here
    double vectors[3][3];

    double angles[3];
    double tminAngle;
    double tmaxAngle;

    for (tIdx t = 1; t <= tcount; t++) {
        sub(vlist[tlist[t][1]], vlist[tlist[t][0]], vectors[0]);
        sub(vlist[tlist[t][2]], vlist[tlist[t][1]], vectors[1]);
        sub(vlist[tlist[t][0]], vlist[tlist[t][2]], vectors[2]);

        angles[0] = MAX_ANGLE - angleBetweenVectors(vectors[0], vectors[1]);
        angles[1] = MAX_ANGLE - angleBetweenVectors(vectors[1], vectors[2]);
        angles[2] = MAX_ANGLE - angleBetweenVectors(vectors[2], vectors[0]);

        tminAngle = MAX_ANGLE;
        tmaxAngle = 0;

        for (i = 0; i < 3; i++) {
            if (tminAngle > angles[i]) {
                tminAngle = angles[i];
            }
            if (tmaxAngle < angles[i]) {
                tmaxAngle = angles[i];
            }
        }

        statMinAngle[(int)floor(tminAngle / STATS_ANGLE_INTERVAL)]++;
        statMaxAngle[(int)floor(tmaxAngle / STATS_ANGLE_INTERVAL)]++;

        if (minAngle > tminAngle) {
            minAngle = tminAngle;
        }
        if (maxAngle < tmaxAngle) {
            maxAngle = tmaxAngle;
        }
    }

    cout << "Min. angle = " << minAngle << endl;
    cout << "Max. angle = " << maxAngle << endl;

	cout << "Statistics for Maximum Angles" << endl;
	for (i = 0; i < 18; i++)
		cout << statMaxAngle[i] << " ";
	cout << endl;
	cout << "Statistics for Minimum Angles" << endl;
	for (i = 0; i < 18; i++)
		cout << statMinAngle[i] << " ";
	cout << endl;
}

int myObjType::countComponents() {
    int count = 0;

    bool visitedTriangles[tcount + 1];
    memset(visitedTriangles, 0, (tcount + 1) * sizeof(bool));
    visitedTriangles[0] = true;

    queue<tIdx> nextTriangles;
    tIdx i, j;

    for (tIdx t = 1; t <= tcount; t++) {
        if (visitedTriangles[t]) {
            continue;
        }
        count++;

        nextTriangles.push(t);
        visitedTriangles[t] = true;

        while (!nextTriangles.empty()) {
            i = nextTriangles.front();
            nextTriangles.pop();

            for (const auto &ot : fnlist[i]) {
                j = idx(ot);
                if (!visitedTriangles[j]) {
                    visitedTriangles[j] = true;
                    nextTriangles.push(j);
                }
            }
        }
    }
    return count;
}

int myObjType::org(OrTri ot) {
    tIdx tIdx = idx(ot);
    int version = ver(ot) % 3;
    return tlist[tIdx][version];
}

int myObjType::dest(OrTri ot) {
    tIdx tIdx = idx(ot);
    int version = (ver(ot) + 1) % 3;
    return tlist[tIdx][version];
}

OrTri myObjType::fnext(OrTri ot) {
    tIdx tIdx = idx(ot);
    int version = ver(ot);
    if (version < 3) {
        return fnlist[tIdx][version];
    } else {
        return sym(fnlist[tIdx][version - 3]);
    }
}

void myObjType::computeFnlist() {
    unordered_map<Edge, vector<OrTri>, hash_edge> connectedOts;
    Edge edges[3];
    OrTri ot;

    // Map all ot pairs that share a given edge
    for (tIdx t = 1; t <= tcount; t++) {
        edges[0] = Edge(tlist[t][0], tlist[t][1]);
        edges[1] = Edge(tlist[t][1], tlist[t][2]);
        edges[2] = Edge(tlist[t][2], tlist[t][0]);

        for (int version = 0; version < 3; version++) {
            ot = makeOrTri(t, version);

            auto forwardResult = connectedOts.find(edges[version]);
            if (forwardResult == connectedOts.end()) {
                // Insert new map entries
                connectedOts.emplace(edges[version], vector<OrTri>{ ot });
                connectedOts.emplace(edges[version].reversed(), vector<OrTri>{ sym(ot) });
            } else {
                // Append existing map entries
                auto backwardResult = connectedOts.find(edges[version].reversed());
                forwardResult->second.emplace_back(ot);
                backwardResult->second.emplace_back(sym(ot));
            }
        }
    }

    // Create fnlist
    for (tIdx t = 1; t <= tcount; t++) {
        edges[0] = Edge(tlist[t][0], tlist[t][1]);
        edges[1] = Edge(tlist[t][1], tlist[t][2]);
        edges[2] = Edge(tlist[t][2], tlist[t][0]);

        for (int version = 0; version < 3; version++) {
            ot = makeOrTri(t, version);

            auto result = connectedOts.find(edges[version]);
            for (auto otherOt : result->second) {
                if (ot != otherOt) {
                    fnlist[t][version] = otherOt;
                    break;
                }
            }
        }
    }
}

void myObjType::computeVnlist() {
    int i;
    tIdx t;

    memset(vnlist, 0, MAXV * 3 * sizeof(double));
    for (t = 1; t <= tcount; t++) {
        for (i = 0; i < 3; i++) {
            add(nlist[t], TVERTEX_N(t, i), TVERTEX_N(t, i));
        }
    }

    // Create vnlist
    for (int vn = 1; vn <= vcount; vn++) {
        normalize(vnlist[vn], vnlist[vn]);
    }
}

void myObjType::reset() {
    vcount = 0;
    tcount = 0;

    memset(vlist, 0, MAXV * 3 * sizeof(double));
    memset(vnlist, 0, MAXV * 3 * sizeof(double));

    memset(tlist, 0, MAXT * 3 * sizeof(int));
    memset(fnlist, 0, MAXT * 3 * sizeof(int));
    memset(nlist, 0, MAXT * 3 * sizeof(double));

    memset(lmax, 0, 3 * sizeof(double));
    memset(lmin, 0, 3 * sizeof(double));

    memset(statMinAngle, 0, 18 * sizeof(int));
    memset(statMaxAngle, 0, 18 * sizeof(int));

    xvcount = 0;
    xtcount = 0;

    memset(highlight, 0, MAXT * sizeof(bool));
    memset(focus, 0, MAXT * sizeof(bool));
    isFocused = false;
    crossingLines.clear();

    memset(tempTlist, 0, MAXT * 3 * sizeof(int));
    memset(tempNlist, 0, MAXT * 3 * sizeof(double));
}

void myObjType::clearHighlight() {
    memset(highlight, 0, MAXT * sizeof(bool));
}

void myObjType::clearFocus() {
    memset(focus, 0, MAXT * sizeof(bool));
    isFocused = false;
}

bool myObjType::addFocus(tIdx t) {
    if (t <= 0 || t > tcount) {
        return false;
    }
    focus[t] = true;
    isFocused = true;
    return true;
}

void myObjType::setThickness(double dt) {
    if (dt == 0.0) {
        return;
    }

    double vt[3];
    for (int v = 1; v <= vcount; v++)
    {
        scale(vnlist[v], dt, vt);
        add(vlist[v], vt, vlist[v]);
    }
}

void myObjType::setSmoothShading(bool on) {
    smoothShading = on;
}

void myObjType::setMatDiffuse(int face, float (&mat)[4]) {
    for (int i = 0; i < 4; i++) {
        mat_diffuse[face][i] = mat[i];
    }
}

void myObjType::computeIntersections() {
    crossingLines.clear();
    clearHighlight();

    xvcount = vcount;

    double start[3], end[3];
    Edge data;
    int org, dest;

    for (tIdx i = 1; i <= tcount; i++) {
        for (tIdx j = i + 1; j <= tcount; j++) {
            if (intersects(i, j, start, end)) {
                cout << "Intersects: " << i << ", " << j << endl;
                org = 0;
                dest = 0;
                for (int k = vcount + 1; k <= xvcount; k++) {
                    if (equal(start, vlist[k])) {
                        org = k;
                        break;
                    }
                }
                for (int k = vcount + 1; k <= xvcount; k++) {
                    if (equal(end, vlist[k])) {
                        dest = k;
                        break;
                    }
                }
                if (org == 0) {
                    org = ++xvcount;
                    copy(start, vlist[org]);
                }
                if (dest == 0) {
                    dest = ++xvcount;
                    copy(end, vlist[dest]);
                }
                data = Edge(org, dest);

                // Insert crossing line for triangle i
                auto findI = crossingLines.find(i);
                if (findI == crossingLines.end()) {
                    crossingLines.emplace(i, vector<Edge>{ data });
                } else {
                    findI->second.emplace_back(data);
                }

                // Insert crossing line for triangle j
                auto findJ = crossingLines.find(j);
                if (findJ == crossingLines.end()) {
                    crossingLines.emplace(j, vector<Edge>{ data });
                } else {
                    findJ->second.emplace_back(data);
                }

                highlight[i] = true;
                highlight[j] = true;
            };
        }
    }

    int badTriangles = 0;
    for (tIdx t = 1; t <= tcount; t++) {
        if (highlight[t]) {
            badTriangles++;
        };
    }
    cout << "No. of intersections: " << ((xvcount - vcount) / 2) << endl;
    cout << "No. of bad triangles: " << badTriangles << endl;
}

void myObjType::repairIntersections() {
    xtcount = 0;

    // Copy good triangles
    for (tIdx i = 1; i <= tcount; i++) {
        if (highlight[i]) {
            continue;
        };
        xtcount++;
        copy(tlist[i], tempTlist[xtcount]);
        copy(nlist[i], tempNlist[xtcount]);
    }

    clearHighlight();

    tIdx t;
    Delaunay delaunay;

    for (auto &datas : crossingLines) {
        t = datas.first;
        delaunay = Delaunay(tlist[t], nlist[t], TVERTEX(t, 0), TVERTEX(t, 1), TVERTEX(t, 2), datas.second);

        for (auto &edge : datas.second) {
            delaunay.insert(edge.org, vlist[edge.org]);
            delaunay.insert(edge.dest, vlist[edge.dest]);
        }

        for (auto &triangle : delaunay.triangles) {
            xtcount++;
            copy(triangle.v, tempTlist[xtcount]);
            copy(nlist[t], tempNlist[xtcount]);
        }
    }

    memcpy(tlist, tempTlist, MAXT * 3 * sizeof(int));
    memcpy(nlist, tempNlist, MAXT * 3 * sizeof(double));

    vcount = xvcount;
    tcount = xtcount;

    computeVnlist();
}

void myObjType::removeHiddenFaces() {
    double mean[3];
    double facing;
    int totalWinding;
    int zero[3] = { 0, 0, 0 };
    vector<tIdx> badTs;

    for (tIdx i = 1; i <= tcount; i++) {
        totalWinding = 0;
        copy(TVERTEX(i, 0), mean);
        add(mean, TVERTEX(i, 1), mean);
        add(mean, TVERTEX(i, 2), mean);
        scale(mean, THIRD, mean);

        for (tIdx j = 1; j <= tcount; j++) {
            if (i == j) {
                continue;
            }
            if (raycast(j, mean, nlist[i], facing)) {
                if (facing > 0) {
                    totalWinding--;
                } else {
                    totalWinding++;
                }
            };
        }
        if (totalWinding < 0) {
            badTs.push_back(i);
        }
    }

    for (auto &t: badTs) {
        copy(zero, tlist[t]);
    }

    computeFnlist();
    computeVnlist();
}

bool myObjType::raycast(tIdx t, double (&origin)[3], double (&direction)[3], double &facing) {
    // Setup variables.
    double (&v0)[3] = vlist[tlist[t][0]];
    double (&v1)[3] = vlist[tlist[t][1]];
    double (&v2)[3] = vlist[tlist[t][2]];

    double e1[3], e2[3];
    sub(v1, v0, e1);
    sub(v2, v0, e2);

    double h[3], a;
    crossProduct(direction, e2, h);
    a = dotProduct(e1, h);

    if (-EPSILON < a && a < EPSILON) {
        return false;
    }

    double f = 1.0 / a;
    double s[3];
    sub(origin, v0, s);
    double u = f * dotProduct(s, h);
    if (u < 0.0 || u > 1.0) {
        return false;
    }

    double q[3];
    crossProduct(s, e1, q);
    double v = f * dotProduct(direction, q);
    if (v < 0.0 || (u + v) > 1.0) {
        return false;
    }

    double interval = f * dotProduct(e2, q);
    if (interval <= EPSILON) {
        return false;
    }
    facing = dotProduct(nlist[t], direction);
    return true;
}

bool myObjType::intersects(tIdx t1, tIdx t2, double (&start)[3], double (&end)[3]) {
    // Reject triangles that share a vertex.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (tlist[t1][i] == tlist[t2][j]) {
                return false;
            }
        }
    }

    // Setup variables.
    double (&n1)[3] = nlist[t1];
    double (&n2)[3] = nlist[t2];

    double (&v10)[3] = vlist[tlist[t1][0]];
    double (&v11)[3] = vlist[tlist[t1][1]];
    double (&v12)[3] = vlist[tlist[t1][2]];

    double (&v20)[3] = vlist[tlist[t2][0]];
    double (&v21)[3] = vlist[tlist[t2][1]];
    double (&v22)[3] = vlist[tlist[t2][2]];

    double d1 = -dotProduct(n1, v10);
    double d2 = -dotProduct(n2, v20);

    double dv1[3];
    dv1[0] = dotProduct(n2, v10) + d2;
    dv1[1] = dotProduct(n2, v11) + d2;
    dv1[2] = dotProduct(n2, v12) + d2;

    int countZeros = 0;
    if (fabs(dv1[0]) < EPSILON) {
        countZeros++;
        dv1[0] = 0.0;
    }
    if (fabs(dv1[1]) < EPSILON) {
        countZeros++;
        dv1[1] = 0.0;
    }
    if (fabs(dv1[2]) < EPSILON) {
        countZeros++;
        dv1[2] = 0.0;
    }
    if (countZeros > 1) {
        return false;
    }

    double dv2[3];
    dv2[0] = dotProduct(n1, v20) + d1;
    dv2[1] = dotProduct(n1, v21) + d1;
    dv2[2] = dotProduct(n1, v22) + d1;

    countZeros = 0;
    if (fabs(dv2[0]) < EPSILON) {
        countZeros++;
        dv2[0] = 0.0;
    }
    if (fabs(dv2[1]) < EPSILON) {
        countZeros++;
        dv2[1] = 0.0;
    }
    if (fabs(dv2[2]) < EPSILON) {
        countZeros++;
        dv2[2] = 0.0;
    }
    if (countZeros > 1) {
        return false;
    }

    double d1x01 = dv1[0] * dv1[1];
    double d1x02 = dv1[0] * dv1[2];

    double d2x01 = dv2[0] * dv2[1];
    double d2x02 = dv2[0] * dv2[2];

    // Reject triangles that are parallel.
    if (d1x01 > 0.0 && d1x02 > 0.0) {
        return false;
    }
    if (d2x01 > 0.0 && d2x02 > 0.0) {
        return false;
    }

    double direction[3];
    crossProduct(n1, n2, direction);

    int index = 0;
    double max = fabs(direction[0]);
    double b = fabs(direction[1]);
    double c = fabs(direction[2]);

    // Optimize by projecting onto coordinate axes.
    if (max < b) {
        max = b;
        index = 1;
    }
    if (max < c) {
        max = c;
        index = 2;
    }

    double pv1[3];
    pv1[0] = v10[index];
    pv1[1] = v11[index];
    pv1[2] = v12[index];

    double pv2[3];
    pv2[0] = v20[index];
    pv2[1] = v21[index];
    pv2[2] = v22[index];

    // Reject triangles that are coplanar.
    // Calculate intersection points at the same time.
    double isect1[2], isect10[3], isect11[3];
    if (isCoplanarIsect(v10, v11, v12, pv1, dv1, d1x01, d1x02, isect1, isect10, isect11)) {
        return false;
    };

    double isect2[2], isect20[3], isect21[3];
    if (isCoplanarIsect(v20, v21, v22, pv2, dv2, d2x01, d2x02, isect2, isect20, isect21)) {
        return false;
    };

    bool swapped1;
    bool swapped2;
    SWAP(isect1[0], isect1[1], swapped1);
    SWAP(isect2[0], isect2[1], swapped2);

    // Reject if crossing line does not have an overlap.
    if (isect1[1] < isect2[0] || isect2[1] < isect1[0]) {
        return false;
    }

    // Reorder intersection points.
    crossingLine(isect1, isect10, isect11, swapped1,
                 isect2, isect20, isect21, swapped2,
                 start, end);

    if (equal(start, end)) {
        return false;
    }
    return true;
}

bool myObjType::isCoplanarIsect(double (&v0)[3], double (&v1)[3], double (&v2)[3],
                                double (&p)[3], double (&d)[3], double dx01, double dx02,
                                double (&isect)[2], double (&isectPoint0)[3], double (&isectPoint1)[3]) {
    if (dx01 > 0.0) {
        isectPoints(v2, v0, v1, p[2], p[0], p[1], d[2], d[0], d[1], isect, isectPoint0, isectPoint1);
    } else if (dx02 > 0.0) {
        isectPoints(v1, v0, v2, p[1], p[0], p[2], d[1], d[0], d[2], isect, isectPoint0, isectPoint1);
    } else if (d[1] * d[2] > 0.0 || d[0] != 0.0) {
        isectPoints(v0, v1, v2, p[0], p[1], p[2], d[0], d[1], d[2], isect, isectPoint0, isectPoint1);
    } else if (d[1] != 0.0) {
        isectPoints(v1, v0, v2, p[1], p[0], p[2], d[1], d[0], d[2], isect, isectPoint0, isectPoint1);
    } else if (d[2] != 0.0) {
        isectPoints(v2, v0, v1, p[2], p[0], p[1], d[2], d[0], d[1], isect, isectPoint0, isectPoint1);
    } else {
        return true; // coplanar
    }
    return false;
}

void myObjType::isectPoints(double (&v0)[3], double (&v1)[3], double (&v2)[3],
                            double p0, double p1, double p2,
                            double d0, double d1, double d2,
                            double (&isect)[2], double (&isectPoint0)[3], double (&isectPoint1)[3]) {
    double triangleVector[3];
    double factor = d0 / (d0 - d1);
    isect[0] = p0 + (p1 - p0) * factor;
    sub(v1, v0, triangleVector);
    scale(triangleVector, factor, triangleVector);
    add(v0, triangleVector, isectPoint0);

    factor = d0 / (d0 - d2);
    isect[1] = p0 + (p2 - p0) * factor;
    sub(v2, v0, triangleVector);
    scale(triangleVector, factor, triangleVector);
    add(v0, triangleVector, isectPoint1);
}

void myObjType::crossingLine(double (&isect1)[2], double (&isectPoint10)[3], double (&isectPoint11)[3], bool swapped1,
                             double (&isect2)[2], double (&isectPoint20)[3], double (&isectPoint21)[3], bool swapped2,
                             double (&start)[3], double (&end)[3]) {
    if (isect2[0] < isect1[0]) {
        if (isect2[1] < isect1[1]) {
            if (swapped1) {
                copy(isectPoint11, start);
            } else {
                copy(isectPoint10, start);
            }
            if (swapped2) {
                copy(isectPoint20, end);
            } else {
                copy(isectPoint21, end);
            }
        } else {
            if (swapped1) {
                copy(isectPoint11, start);
                copy(isectPoint10, end);
            } else {
                copy(isectPoint10, start);
                copy(isectPoint11, end);
            }
        }
    } else {
        if (isect2[1] > isect1[1]) {
            if (swapped2) {
                copy(isectPoint21, start);
            } else {
                copy(isectPoint20, start);
            }
            if (swapped1) {
                copy(isectPoint10, end);
            } else {
                copy(isectPoint11, end);
            }
        } else {
            if (swapped2) {
                copy(isectPoint21, start);
                copy(isectPoint20, end);
            } else {
                copy(isectPoint20, start);
                copy(isectPoint21, end);
            }
        }
    }
}

